#include "periph_wrappers.h"
#include "can.h"
#include "iwdg.h"
#include "stm32f4xx_hal.h"
#include "main.h"

static CAN_HandleTypeDef *handle_of(can_bus_t bus) {
    switch (bus) {
    case CAN_BUS_AVIONICS:   return &hcan1;
    case CAN_BUS_POWERTRAIN: return &hcan2;
    default:               return 0;
    }
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
    if (HAL_CAN_GetTxMailboxesFreeLevel(h) == 0) return false;

    CAN_TxHeaderTypeDef hdr = {0};
    if (f->ext) { hdr.ExtId = f->id; hdr.IDE = CAN_ID_EXT; }
    else        { hdr.StdId = f->id; hdr.IDE = CAN_ID_STD; }
    hdr.RTR = f->rtr ? CAN_RTR_REMOTE : CAN_RTR_DATA;
    hdr.DLC = f->dlc;
    hdr.TransmitGlobalTime = DISABLE;

    uint32_t mbx;
    return HAL_CAN_AddTxMessage(h, &hdr, (uint8_t *)f->data, &mbx) == HAL_OK;
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
