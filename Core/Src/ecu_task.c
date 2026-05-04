/*
 * ecu_task.c — Loweheiser MegaSquirt-style CAN broadcast subscriber.
 *
 * The Loweheiser EFI/ECU broadcasts engine telemetry directly on the CAN
 * bus using the MegaSquirt 'Simplified Dash Broadcasting' format documented
 * in "MS2 and MS3 CAN realtime data broadcasting format" (Loweheiser, Nov-2025).
 *
 * Five 8-byte standard-11-bit frames at the configurable base ID (default
 * 1512 / 0x5E8) are emitted at 50 Hz on MS3 (20 Hz on MS2). All multi-byte
 * fields are big-endian.
 *
 *   0x5E8 (1512)  [0..1] map (0.1 kPa)   [2..3] rpm (1 RPM)
 *                 [4..5] clt (0.1 °F)    [6..7] tps (0.1 %)
 *   0x5E9 (1513)  pw1 pw2 mat adv_deg
 *   0x5EA (1514)  afrtgt1 AFR1 EGOcor1 egt1 pwseq1
 *   0x5EB (1515)  batt sensors1 sensors2 knk_rtd
 *   0x5EC (1516)  VSS1 tc_retard launch_timing
 *
 * **Bus bitrate:** Loweheiser is fixed at 500 kbps. CAN2 (engine bus) and
 * the VESC must both be at 500k for this to coexist. See can.c, and set
 * "CAN baud rate = 500 kbit/s" in VESC Tool. Daly BMS (250k) cannot share
 * this bus; needs a separate physical bus.
 *
 * Today we decode the two fields that map to existing g_pt slots (RPM,
 * coolant-temp -> ecu_cht_C). The Stinger is air-cooled with the temp
 * sender on the head, so CLT here is effectively cylinder-head-temperature.
 * Fuel rate is not in simplified broadcast; enable Advanced broadcast
 * Group 52 ('fuelflow' cc/min) on the ECU and add a decoder below to
 * populate g_pt.ecu_fuel_rate_dg_s — see TODO.
 *
 * Other useful simplified-broadcast fields (TPS, MAP, MAT, batt, AFR,
 * advance, etc.) are decoded but discarded today because g_pt has no
 * slots for them. Extend g_pt and the switch below when needed.
 */

#include "ecu_task.h"
#include "can_manager.h"
#include "periph_wrappers.h"
#include "powertrain_state.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdint.h>

/* ----- Tunables ----- */

#define ECU_DRAIN_PERIOD_MS    20    /* 50 Hz drain — ECU sends at 50 Hz on MS3 */
#define ECU_STALE_MS         1000    /* 1 s of silence -> FAULT_ECU_STALE */
#define ECU_RX_QUEUE_DEPTH     16

/* MegaSquirt simplified-dash broadcast base ID. Configurable on the ECU
 * (CAN-Bus/Testmodes -> Dash Broadcasting -> Base CAN identifier). Default
 * is 1512 decimal = 0x5E8. */
#define MS_BASE_ID            0x5E8u

#define MS_ID_GRP0            (MS_BASE_ID + 0u)   /* map, rpm, clt, tps */
#define MS_ID_GRP1            (MS_BASE_ID + 1u)   /* pw1, pw2, mat, adv */
#define MS_ID_GRP2            (MS_BASE_ID + 2u)   /* afr*, egt1, pwseq1 */
#define MS_ID_GRP3            (MS_BASE_ID + 3u)   /* batt, sensors, knock */
#define MS_ID_GRP4            (MS_BASE_ID + 4u)   /* VSS, traction, launch */

/* Filter pattern: id=0x5E8, mask=0x7F8 -> matches 0x5E8..0x5EF
 * (covers all 5 simplified-broadcast IDs plus 3 unused ones we discard). */
#define ECU_FILTER_ID         MS_BASE_ID
#define ECU_FILTER_MASK       0x7F8u

#define ECU_BUS               CAN_BUS_AVIONICS

/* ----- Internal state ----- */

static StaticTask_t       s_tcb;
static StackType_t        s_stack[256];          /* 1 KB */

static StaticQueue_t      s_rxq_cb;
static can_frame_t        s_rxq_items[ECU_RX_QUEUE_DEPTH];
static osMessageQueueId_t s_rx_q;

static void ecu_task(void *arg);

/* ----- Helpers ----- */

static inline uint16_t get_u16_be(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | (uint16_t)p[1];
}
static inline int16_t get_i16_be(const uint8_t *p) {
    return (int16_t)get_u16_be(p);
}
static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

/* deg-F × 10 -> deg-C (integer). C = (F - 32) * 5/9.
 * raw is already 10×F, so C = (raw - 320) * 5/90.  Clamped to int16. */
static inline int16_t f10_to_c(int16_t raw_F10) {
    int32_t c = ((int32_t)raw_F10 - 320) * 5 / 90;
    return (int16_t)clamp_i32(c, INT16_MIN, INT16_MAX);
}

/* Decode one simplified-broadcast frame and update g_pt.ecu_*.
 * Returns true if any field was updated (caller clears FAULT_ECU_STALE). */
static bool parse_ecu_frame(const can_frame_t *f) {
    if (f->ext || f->dlc != 8 || f->rtr != 0) return false;

    bool any = false;
    osMutexAcquire(g_pt_mtx, osWaitForever);

    switch (f->id) {
    case MS_ID_GRP0: {
        /* bytes [2..3] = rpm  (1 RPM/LSB, unsigned)
         * bytes [4..5] = clt  (0.1 °F/LSB, signed) */
        const uint16_t rpm   = get_u16_be(&f->data[2]);
        const int16_t  clt10 = get_i16_be(&f->data[4]);
        g_pt.ecu_rpm   = rpm;
        g_pt.ecu_cht_C = f10_to_c(clt10);
        any = true;
        break;
    }
    case MS_ID_GRP1:
    case MS_ID_GRP2:
    case MS_ID_GRP3:
    case MS_ID_GRP4:
        /* TODO: when g_pt grows fields for MAP/TPS/MAT/batt/AFR/etc.,
         * decode here. For now the frames pass through silently to
         * keep the link alive without polluting g_pt. */
        any = true;       /* still counts toward liveness */
        break;
    default:
        break;
    }

    if (any) g_pt.ecu_input_tick = osKernelGetTickCount();
    osMutexRelease(g_pt_mtx);
    return any;
}

/* ----- Task ----- */

void ecu_task_start(void) {
    static const osMessageQueueAttr_t qattr = {
        .name    = "ecu_rxq",
        .cb_mem  = &s_rxq_cb,
        .cb_size = sizeof(s_rxq_cb),
        .mq_mem  = s_rxq_items,
        .mq_size = sizeof(s_rxq_items),
    };
    s_rx_q = osMessageQueueNew(ECU_RX_QUEUE_DEPTH,
                               sizeof(can_frame_t), &qattr);

    /* Standard 11-bit, base 0x5E8 with don't-care low 3 bits.
     * Filter passes 0x5E8..0x5EF (5 useful + 3 ignored). */
    can_mgr_subscribe(ECU_BUS, ECU_FILTER_ID, ECU_FILTER_MASK, false, s_rx_q);

    static const osThreadAttr_t tattr = {
        .name       = "ecu",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityNormal,         /* prio 3 */
    };
    osThreadNew(ecu_task, NULL, &tattr);
}

static void ecu_task(void *arg) {
    (void)arg;
    uint32_t next = osKernelGetTickCount();

    for (;;) {
        bool any_ok = false;

        can_frame_t f;
        while (osMessageQueueGet(s_rx_q, &f, NULL, 0) == osOK) {
            if (parse_ecu_frame(&f)) any_ok = true;
        }
        if (any_ok) pt_clear_fault(FAULT_ECU_STALE);

        /* Stale arms only after first successful decode so a never-
         * connected ECU doesn't constantly raise the fault. */
        osMutexAcquire(g_pt_mtx, osWaitForever);
        const uint32_t last = g_pt.ecu_input_tick;
        osMutexRelease(g_pt_mtx);
        if (last && (osKernelGetTickCount() - last) > ECU_STALE_MS) {
            pt_set_fault(FAULT_ECU_STALE);
        }

        next += ECU_DRAIN_PERIOD_MS;
        osDelayUntil(next);
    }
}
