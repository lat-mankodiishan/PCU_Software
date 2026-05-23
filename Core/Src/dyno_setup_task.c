/* dyno_setup_task — SSD-250A current sensor RX on CAN1, 50 Hz drain. */

#include "dyno_setup_task.h"
#include "can_manager.h"
#include "periph_wrappers.h"
#include "powertrain_state.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdint.h>
#include <string.h>

#define DYNO_DRAIN_PERIOD_MS    20      /* 50 Hz drain */
#define DYNO_RX_QUEUE_DEPTH     16

/* 5FX-3 = source, 4FX-2 = load; one HW filter bank per sensor block. */
#define DYNO_FILTER_5FX_ID     0x5F0u
#define DYNO_FILTER_5FX_MASK   0x7F8u
#define DYNO_FILTER_4FX_ID     0x4F0u
#define DYNO_FILTER_4FX_MASK   0x7F8u

#define SSD3_CURRENT           0x5F1u
#define SSD3_VBUS              0x5F3u
#define SSD3_POWER             0x5F5u
#define SSD3_ENERGY            0x5F6u

#define SSD2_CURRENT           0x4F1u
#define SSD2_VBUS              0x4F3u
#define SSD2_POWER             0x4F5u
#define SSD2_ENERGY            0x4F6u

#define SSD_CAN_ID_SET_COMMAND      0x3FAu
#define SSD_CMD_SET_MODE            0x12u
#define SSD_CMD_SET_READING_DELAY   0x16u

/* SetMode_config_t bits (ssd_250A_C.h). */
#define SSD_MODE_AUTO_SEND     (1u <<  8)
#define SSD_MODE_SEND_CURRENT  (1u <<  9)
#define SSD_MODE_SEND_VBUS     (1u << 11)
#define SSD_MODE_SEND_POWER    (1u << 13)
#define SSD_MODE_SEND_ENERGY   (1u << 14)

/* ms between auto-send sweeps; 100 ms => ~10 Hz per quantity. */
#define SSD_READING_DELAY_MS   100u

#define DYNO_BUS               CAN_BUS_DRONECAN

static StaticTask_t       s_tcb;
static StackType_t        s_stack[256];          /* 1 KB */

static StaticQueue_t      s_rxq_cb;
static can_frame_t        s_rxq_items[DYNO_RX_QUEUE_DEPTH];
static osMessageQueueId_t s_rx_q;

static void dyno_task(void *arg);

/* Debug counters. */
volatile uint32_t g_dyno_setmode_attempts = 0;
volatile uint32_t g_dyno_setmode_ok       = 0;
volatile uint32_t g_dyno_rx_frames        = 0;
volatile uint32_t g_dyno_rx_current       = 0;
volatile uint32_t g_dyno_rx_vbus          = 0;
volatile uint32_t g_dyno_rx_power         = 0;
volatile uint32_t g_dyno_rx_energy        = 0;

/* LE byte assembly, variable-length (from ssd_250A_C.c). */
static uint64_t le_u64(const uint8_t *data, uint8_t length) {
    uint64_t v = 0;
    for (uint8_t i = 0; i < length && i < 8; i++) {
        v |= ((uint64_t)data[i]) << (8 * i);
    }
    return v;
}

/* Volatile mode set; re-issued each PCU boot. Byte order [cmd, hi, lo]. */
static void dyno_send_setmode(uint16_t cfg) {
    can_frame_t f = { 0 };
    f.id  = SSD_CAN_ID_SET_COMMAND;
    f.dlc = 3;
    f.data[0] = SSD_CMD_SET_MODE;
    f.data[1] = (uint8_t)((cfg >> 8) & 0xFFu);
    f.data[2] = (uint8_t)(cfg        & 0xFFu);
    g_dyno_setmode_attempts++;
    if (can_mgr_send(DYNO_BUS, &f, 100)) g_dyno_setmode_ok++;
}

/* Set per-quantity broadcast period (ms). */
static void dyno_send_reading_delay(uint16_t ms) {
    can_frame_t f = { 0 };
    f.id  = SSD_CAN_ID_SET_COMMAND;
    f.dlc = 3;
    f.data[0] = SSD_CMD_SET_READING_DELAY;
    f.data[1] = (uint8_t)((ms >> 8) & 0xFFu);
    f.data[2] = (uint8_t)(ms        & 0xFFu);
    can_mgr_send(DYNO_BUS, &f, 100);
}

static void dyno_parse_frame(const can_frame_t *f) {
    if (f->ext || f->rtr) return;

    g_dyno_rx_frames++;

    osMutexAcquire(g_pt_mtx, osWaitForever);
    bool is_load = false;
    switch (f->id) {
    /* ---- Source (5FX-3) ---- */
    case SSD3_CURRENT:
        g_pt.dyno_current_mA = (int32_t)le_u64(f->data, f->dlc);
        g_dyno_rx_current++;
        break;
    case SSD3_VBUS:
        g_pt.dyno_vbus_mV = (int32_t)le_u64(f->data, f->dlc);
        g_dyno_rx_vbus++;
        break;
    case SSD3_POWER:
        g_pt.dyno_power_dW = (uint32_t)le_u64(f->data, f->dlc);
        g_dyno_rx_power++;
        break;
    case SSD3_ENERGY:
        g_pt.dyno_energy_Wh = le_u64(f->data, f->dlc);
        g_dyno_rx_energy++;
        break;
    /* ---- Load (4FX-2) ---- */
    case SSD2_CURRENT:
        g_pt.dyno_load_current_mA = (int32_t)le_u64(f->data, f->dlc);
        is_load = true;
        break;
    case SSD2_VBUS:
        g_pt.dyno_load_vbus_mV = (int32_t)le_u64(f->data, f->dlc);
        is_load = true;
        break;
    case SSD2_POWER:
        g_pt.dyno_load_power_dW = (uint32_t)le_u64(f->data, f->dlc);
        is_load = true;
        break;
    case SSD2_ENERGY:
        g_pt.dyno_load_energy_Wh = le_u64(f->data, f->dlc);
        is_load = true;
        break;
    default:
        osMutexRelease(g_pt_mtx);
        return;
    }
    if (is_load) g_pt.dyno_load_input_tick = osKernelGetTickCount();
    else         g_pt.dyno_input_tick      = osKernelGetTickCount();
    osMutexRelease(g_pt_mtx);
}

void dyno_setup_task_start(void) {
    static const osMessageQueueAttr_t qattr = {
        .name    = "dyno_rxq",
        .cb_mem  = &s_rxq_cb,
        .cb_size = sizeof(s_rxq_cb),
        .mq_mem  = s_rxq_items,
        .mq_size = sizeof(s_rxq_items),
    };
    s_rx_q = osMessageQueueNew(DYNO_RX_QUEUE_DEPTH,
                               sizeof(can_frame_t), &qattr);

    can_mgr_subscribe(DYNO_BUS, DYNO_FILTER_5FX_ID, DYNO_FILTER_5FX_MASK, false, s_rx_q);
    can_mgr_subscribe(DYNO_BUS, DYNO_FILTER_4FX_ID, DYNO_FILTER_4FX_MASK, false, s_rx_q);

    static const osThreadAttr_t tattr = {
        .name       = "dyno",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityNormal,         /* prio 3 */
    };
    osThreadNew(dyno_task, NULL, &tattr);
}

static void dyno_task(void *arg) {
    (void)arg;

    /* Startup grace for can_manager + CAN1 HW. */
    osDelay(500);

    const uint16_t cfg = SSD_MODE_AUTO_SEND
                       | SSD_MODE_SEND_CURRENT
                       | SSD_MODE_SEND_VBUS
                       | SSD_MODE_SEND_POWER
                       | SSD_MODE_SEND_ENERGY;

    /* Re-issue SetMode every 2 s; volatile, survives arbitration loss. */
    uint32_t setmode_next = osKernelGetTickCount();
    uint32_t next         = osKernelGetTickCount();

    for (;;) {
        if ((int32_t)(osKernelGetTickCount() - setmode_next) >= 0) {
            dyno_send_setmode(cfg);
            osDelay(20);
            dyno_send_reading_delay(SSD_READING_DELAY_MS);
            setmode_next += 2000;
        }

        can_frame_t f;
        while (osMessageQueueGet(s_rx_q, &f, NULL, 0) == osOK) {
            dyno_parse_frame(&f);
        }

        next += DYNO_DRAIN_PERIOD_MS;
        osDelayUntil(next);
    }
}
