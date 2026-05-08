/*
 * dyno_setup_task.c — SSD-250A current sensor subscriber for dyno bench tests.
 *
 * Connects to ID 5FX-3 SSD-250A on CAN1 (CAN_BUS_AVIONICS). The SSD broadcasts
 * native CAN 2.0B standard frames at IDs 0x5F1..0x5F7 — we subscribe via the
 * can_manager and drain the queue at 50 Hz, mirroring ecu_task's structure.
 *
 * Frame layout (from inverterCANComm/ssd_250A_C.c, taken as-is):
 *
 *   0x5F1 current   — int32 mA           LE,  DLC = 4
 *   0x5F3 vbus      — int32 mV           LE,  DLC = 4
 *   0x5F5 power     — uint32 dW (0.1 W)  LE,  DLC = 4
 *   0x5F6 energy    — uint64 Wh          LE,  DLC = 8
 *   0x5F2/4/7       — temperature/coulomb/errors (unused here, dropped)
 *
 * Bus bitrate: SSD-250A must be set to match CAN1 (500 kbps per
 * periph_wrappers.h comment) — use the SSD-side ssd_set_baudrate cmd
 * during commissioning.
 *
 * No mutex on the read side — log_task already takes g_pt_mtx around its
 * snapshot. We take the same mutex for our writes.
 */

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

/* SSD-250A sensor #3 (5FX-3) base ID. The seven readout IDs are 0x5F1..0x5F7;
 * filter on 0x5F0 with mask 0x7F8 catches 0x5F0..0x5F7 (one don't-care bank). */
#define DYNO_FILTER_ID         0x5F0u
#define DYNO_FILTER_MASK       0x7F8u

#define SSD3_CURRENT           0x5F1u
#define SSD3_VBUS              0x5F3u
#define SSD3_POWER             0x5F5u
#define SSD3_ENERGY            0x5F6u

/* SSD command channel — used to enable broadcast bits + set rate at boot. */
#define SSD_CAN_ID_SET_COMMAND      0x3FAu
#define SSD_CMD_SET_MODE            0x12u
#define SSD_CMD_SET_READING_DELAY   0x16u

/* SSD SetMode config_value bit positions (see SetMode_config_t in ssd_250A_C.h). */
#define SSD_MODE_AUTO_SEND     (1u <<  8)
#define SSD_MODE_SEND_CURRENT  (1u <<  9)
#define SSD_MODE_SEND_VBUS     (1u << 11)
#define SSD_MODE_SEND_POWER    (1u << 13)
#define SSD_MODE_SEND_ENERGY   (1u << 14)

/* Reading delay in ms between successive auto-send sweeps.
 * 100 ms ⇒ ~10 Hz per quantity (current, vbus, power, energy each
 * broadcast once per sweep). */
#define SSD_READING_DELAY_MS   100u

#define DYNO_BUS               CAN_BUS_AVIONICS

static StaticTask_t       s_tcb;
static StackType_t        s_stack[256];          /* 1 KB */

static StaticQueue_t      s_rxq_cb;
static can_frame_t        s_rxq_items[DYNO_RX_QUEUE_DEPTH];
static osMessageQueueId_t s_rx_q;

static void dyno_task(void *arg);

/* Debug counters — read via Live Expressions or STM32_Programmer_CLI -r32 */
volatile uint32_t g_dyno_setmode_attempts = 0;
volatile uint32_t g_dyno_setmode_ok       = 0;
volatile uint32_t g_dyno_rx_frames        = 0;
volatile uint32_t g_dyno_rx_current       = 0;
volatile uint32_t g_dyno_rx_vbus          = 0;
volatile uint32_t g_dyno_rx_power         = 0;
volatile uint32_t g_dyno_rx_energy        = 0;

/* Lifted verbatim from ssd_250A_C.c parsers — little-endian byte assembly,
 * variable-length to match the original. */
static uint64_t le_u64(const uint8_t *data, uint8_t length) {
    uint64_t v = 0;
    for (uint8_t i = 0; i < length && i < 8; i++) {
        v |= ((uint64_t)data[i]) << (8 * i);
    }
    return v;
}

/* Volatile mode change — does not call save_setting, so the SSD reverts on
 * its own power cycle. Re-issued on every PCU boot. Byte order matches
 * ssd_setmode() in the inverter project: [cmd, hi, lo]. */
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

/* Set the SSD's per-quantity broadcast period (ms). Same byte layout as
 * ssd_set_reading_delay() in the inverter project. */
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
    switch (f->id) {
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
    default:
        osMutexRelease(g_pt_mtx);
        return;       /* don't bump tick on unrecognised IDs */
    }
    g_pt.dyno_input_tick = osKernelGetTickCount();
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

    /* Standard 11-bit, base 0x5F0 with don't-care low 3 bits → 0x5F0..0x5F7. */
    can_mgr_subscribe(DYNO_BUS, DYNO_FILTER_ID, DYNO_FILTER_MASK, false, s_rx_q);

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

    /* Startup grace so can_manager + CAN1 HW are fully up before TX. */
    osDelay(500);

    const uint16_t cfg = SSD_MODE_AUTO_SEND
                       | SSD_MODE_SEND_CURRENT
                       | SSD_MODE_SEND_VBUS
                       | SSD_MODE_SEND_POWER
                       | SSD_MODE_SEND_ENERGY;

    /* Re-issue SetMode every 2 s. Volatile (no save_setting) — the SSD's
     * persistent config isn't disturbed; this re-applies after each PCU
     * reboot until the SSD itself power-cycles. Re-tx survives a single
     * bus arbitration loss. */
    uint32_t setmode_next = osKernelGetTickCount();
    uint32_t next         = osKernelGetTickCount();

    for (;;) {
        if ((int32_t)(osKernelGetTickCount() - setmode_next) >= 0) {
            dyno_send_setmode(cfg);
            osDelay(20);                            /* let mode settle on SSD */
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
