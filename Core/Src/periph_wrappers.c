#include "periph_wrappers.h"
#include "can.h"
#include "iwdg.h"
#include "tim.h"
#include "usart.h"
#include "stm32f4xx_hal.h"
#include "main.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "semphr.h"

static CAN_HandleTypeDef *handle_of(can_bus_t bus) {
    switch (bus) {
    case CAN_BUS_DRONECAN: return &hcan1;
    case CAN_BUS_ENGINE:   return &hcan2;
    default:               return 0;
    }
}

/* DEBUG: live snapshot of CAN1 error/status registers. Watch in VS Code
 * Live Expressions to diagnose bus-off / error-passive / no-ACK issues. */
volatile uint32_t g_can1_esr        = 0;   /* full ESR register */
volatile uint8_t  g_can1_tec        = 0;   /* TX error counter (ESR bits 23:16) */
volatile uint8_t  g_can1_rec        = 0;   /* RX error counter (ESR bits 15:8)  */
volatile uint8_t  g_can1_boff       = 0;   /* bus-off flag    (ESR bit 2)       */
volatile uint8_t  g_can1_epvf       = 0;   /* error-passive   (ESR bit 1)       */
volatile uint8_t  g_can1_ewgf       = 0;   /* error-warning   (ESR bit 0)       */
volatile uint32_t g_can1_tx_post_ok = 0;   /* HAL_CAN_AddTxMessage successes    */
volatile uint32_t g_can1_tx_post_no = 0;   /* TX attempts with no free mailbox  */

void can_hw_diag_snapshot(void) {
    uint32_t esr = hcan1.Instance->ESR;
    g_can1_esr  = esr;
    g_can1_tec  = (uint8_t)((esr >> 16) & 0xFF);
    g_can1_rec  = (uint8_t)((esr >>  8) & 0xFF);
    g_can1_boff = (uint8_t)((esr >>  2) & 0x01);
    g_can1_epvf = (uint8_t)((esr >>  1) & 0x01);
    g_can1_ewgf = (uint8_t)( esr        & 0x01);
}

void can_hw_init(can_bus_t bus) {
    CAN_HandleTypeDef *h = handle_of(bus);
    /* Filters are added by can_hw_add_filter; here we just enable IRQs and start. */
    HAL_CAN_ActivateNotification(h,
        CAN_IT_RX_FIFO0_MSG_PENDING |
        CAN_IT_TX_MAILBOX_EMPTY     |
        CAN_IT_BUSOFF               |
        CAN_IT_ERROR);
    HAL_CAN_Start(h);
}

bool can_hw_try_post(can_bus_t bus, const can_frame_t *f) {
    CAN_HandleTypeDef *h = handle_of(bus);
    if (HAL_CAN_GetTxMailboxesFreeLevel(h) == 0) {
        if (bus == CAN_BUS_DRONECAN) g_can1_tx_post_no++;
        return false;
    }

    CAN_TxHeaderTypeDef hdr = {0};
    if (f->ext) { hdr.ExtId = f->id; hdr.IDE = CAN_ID_EXT; }
    else        { hdr.StdId = f->id; hdr.IDE = CAN_ID_STD; }
    hdr.RTR = f->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    hdr.DLC = f->dlc;
    hdr.TransmitGlobalTime = DISABLE;

    uint32_t mbx;
    bool ok = (HAL_CAN_AddTxMessage(h, &hdr, (uint8_t *)f->data, &mbx) == HAL_OK);
    if (bus == CAN_BUS_DRONECAN) {
        if (ok) g_can1_tx_post_ok++;
        else    g_can1_tx_post_no++;
    }
    return ok;
}

uint8_t can_hw_tx_free(can_bus_t bus) {
    return (uint8_t)HAL_CAN_GetTxMailboxesFreeLevel(handle_of(bus));
}

int can_hw_add_filter(can_bus_t bus, uint32_t id, uint32_t mask, bool ext) {
    /* F4 filter banks 0..13 are CAN1, 14..27 are CAN2 (slave). Caller owns indexing.
     * For bring-up we use a static counter per bus. */
    static uint8_t next_bank[CAN_BUS_COUNT] = { 0, 14 };
    static const uint8_t bank_max[CAN_BUS_COUNT] = { 14, 28 };
    if (next_bank[bus] >= bank_max[bus]) return -1;

    CAN_FilterTypeDef cfg = {0};
    cfg.FilterBank           = next_bank[bus];
    cfg.FilterMode           = CAN_FILTERMODE_IDMASK;
    cfg.FilterScale          = CAN_FILTERSCALE_32BIT;
    cfg.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    cfg.SlaveStartFilterBank = 14;
    cfg.FilterActivation     = CAN_FILTER_ENABLE;
    if (ext) {
        cfg.FilterIdHigh     = (id   >> 13) & 0xFFFF;
        cfg.FilterIdLow      = ((id  <<  3) & 0xFFF8) | (1 << 2);   /* IDE=1 */
        cfg.FilterMaskIdHigh = (mask >> 13) & 0xFFFF;
        cfg.FilterMaskIdLow  = ((mask <<  3) & 0xFFF8) | (1 << 2);
    } else {
        cfg.FilterIdHigh     = (id   << 5) & 0xFFE0;
        cfg.FilterIdLow      = 0;
        cfg.FilterMaskIdHigh = (mask << 5) & 0xFFE0;
        cfg.FilterMaskIdLow  = 0;
    }
    if (HAL_CAN_ConfigFilter(handle_of(bus), &cfg) != HAL_OK) return -1;
    return next_bank[bus]++;
}

bool can_hw_read_fifo0(can_bus_t bus, can_frame_t *out) {
    CAN_HandleTypeDef *h = handle_of(bus);
    if (HAL_CAN_GetRxFifoFillLevel(h, CAN_RX_FIFO0) == 0) return false;

    CAN_RxHeaderTypeDef hdr;
    if (HAL_CAN_GetRxMessage(h, CAN_RX_FIFO0, &hdr, out->data) != HAL_OK) return false;
    out->ext = (hdr.IDE == CAN_ID_EXT);
    out->rtr = (hdr.RTR == CAN_RTR_REMOTE);
    out->dlc = (uint8_t)hdr.DLC;
    out->id  = out->ext ? hdr.ExtId : hdr.StdId;
    return true;
}

void pdb_hw_write(bool battery_closed, bool rectifier_closed) {
    HAL_GPIO_WritePin(PDB_BAT_GPIO_Port,  PDB_BAT_Pin,
                      battery_closed   ? GPIO_PIN_SET : GPIO_PIN_RESET);
    HAL_GPIO_WritePin(PDB_RECT_GPIO_Port, PDB_RECT_Pin,
                      rectifier_closed ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void watchdog_refresh(void) {
    HAL_IWDG_Refresh(&hiwdg);
}

void led_hw_toggle(void) {
    HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
}

/* ---- ESC / servo PWM on TIM1 ---------------------------------------- */

static uint32_t esc_hal_channel(esc_channel_t ch) {
    switch (ch) {
    case ESC_CH_ENGINE: return TIM_CHANNEL_1;
    case ESC_CH_LOAD:   return TIM_CHANNEL_3;
    default:            return TIM_CHANNEL_1;
    }
}

void esc_hw_init(esc_channel_t ch, uint16_t idle_us) {
    const uint32_t hal_ch = esc_hal_channel(ch);
    __HAL_TIM_SET_COMPARE(&htim1, hal_ch, idle_us);
    HAL_TIM_PWM_Start(&htim1, hal_ch);
}

void esc_hw_set_us(esc_channel_t ch, uint16_t pulse_us) {
    __HAL_TIM_SET_COMPARE(&htim1, esc_hal_channel(ch), pulse_us);
}

/* ---- ECU UART (USART2) ---------------------------------------------- */

static ecu_uart_status_t map_hal(HAL_StatusTypeDef st) {
    switch (st) {
    case HAL_OK:      return ECU_UART_OK;
    case HAL_TIMEOUT: return ECU_UART_TIMEOUT;
    default:          return ECU_UART_HAL_ERROR;
    }
}

ecu_uart_status_t ecu_uart_hw_send(const uint8_t *buf, uint16_t len, uint32_t timeout_ms) {
    return map_hal(HAL_UART_Transmit(&huart2, (uint8_t *)buf, len, timeout_ms));
}

/* DMA-based ECU receive ------------------------------------------------
 * HAL polling Receive can't keep up at 115200 inside FreeRTOS — SysTick
 * and other IRQs preempt for >87 µs between bytes, latching ORE and
 * killing the receive. DMA hardware fills the buffer at line rate with
 * zero CPU involvement, then fires the RxCplt callback we override below
 * to release a binary semaphore that the task is waiting on.
 *
 * Lifecycle per poll: acquire sem at timeout=0 to drain any late
 * "callback fired after we already timed out" event from the prior cycle,
 * start DMA, block on the sem with the caller's timeout, on timeout call
 * HAL_UART_AbortReceive to clean up the half-finished DMA. */

static StaticSemaphore_t s_ecu_rx_sem_cb;
static osSemaphoreId_t   s_ecu_rx_sem = NULL;

/* Diagnostic: latest UART error code seen by the HAL error callback.
 * Watch in Live Watch — non-zero indicates a problem on the link. */
volatile uint32_t g_ecu_uart_last_error = 0;

static void ecu_uart_ensure_sem(void) {
    if (s_ecu_rx_sem == NULL) {
        static const osSemaphoreAttr_t attr = {
            .name    = "ecu_rx",
            .cb_mem  = &s_ecu_rx_sem_cb,
            .cb_size = sizeof(s_ecu_rx_sem_cb),
        };
        s_ecu_rx_sem = osSemaphoreNew(1, 0, &attr);   /* binary, starts taken */
    }
}

ecu_uart_status_t ecu_uart_hw_recv(uint8_t *buf, uint16_t len, uint32_t timeout_ms) {
    ecu_uart_ensure_sem();

    /* Drain any leftover release from a late callback after a prior timeout. */
    (void)osSemaphoreAcquire(s_ecu_rx_sem, 0);

    HAL_StatusTypeDef st = HAL_UART_Receive_DMA(&huart2, buf, len);
    if (st != HAL_OK) {
        return (st == HAL_BUSY) ? ECU_UART_TIMEOUT : ECU_UART_HAL_ERROR;
    }

    if (osSemaphoreAcquire(s_ecu_rx_sem, timeout_ms) == osOK) {
        return ECU_UART_OK;
    }

    /* Sem wait timed out — DMA may still be in flight. Abort cleanly. */
    HAL_UART_AbortReceive(&huart2);
    return ECU_UART_TIMEOUT;
}

/* HAL weak override — fires from the DMA1_Stream5 ISR when the requested
 * byte count has landed in the buffer. Releases the receive semaphore so
 * the waiting task can return ECU_UART_OK. */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2 && s_ecu_rx_sem != NULL) {
        (void)osSemaphoreRelease(s_ecu_rx_sem);
    }
}

/* HAL weak override — captures UART error events for diagnostics. ORE
 * shouldn't happen on DMA RX, but FE/NE on a noisy link still might. */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    if (huart->Instance == USART2) {
        g_ecu_uart_last_error = huart->ErrorCode;
    }
}

void ecu_uart_hw_flush_rx(void) {
    /* Clear overrun/framing/noise flags and drain any byte sitting in DR.
     * Mirrors pyserial's reset_input_buffer() before each 'A' request. */
    __HAL_UART_CLEAR_OREFLAG(&huart2);
    while (__HAL_UART_GET_FLAG(&huart2, UART_FLAG_RXNE)) {
        (void)huart2.Instance->DR;
    }
}
