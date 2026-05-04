/*
 * bms_task.c — Polled Daly Smart-BMS over CAN.
 *
 * Wire-protocol mirror of Daly-BMS-Interfacing/Core/Inc/Daly_BMS_CAN.h. The
 * driver in that repo uses blocking HAL calls; we keep the same on-the-wire
 * protocol but ride on top of can_manager (async TX queue, RX into a
 * subscribed osMessageQueue) so the task plays nicely with the rest of the
 * PCU.
 *
 * Protocol (29-bit IDs):
 *   TX (PCU -> BMS): 0x18 [cmd] 01 40   (priority, cmd, BMS=0x01, host=0x40)
 *   RX (BMS -> PCU): 0x18 [cmd] 40 01   (id swapped on response)
 *
 * Polling: round-robin through poll_seq[]. Each iteration:
 *   1) drain any stale frames from the RX queue
 *   2) send the request
 *   3) wait up to BMS_RESPONSE_TIMEOUT_MS for a response with matching cmd
 *   4) decode -> g_pt under mutex
 *   5) inter-command delay (per Daly driver)
 *
 * Architecture target: BMS lives on Bus 3 (battery path) on the H7. F405 has
 * no Bus 3, so for bring-up we listen on CAN_BUS_POWERTRAIN — adjust when the
 * H7 board is in.
 *
 * Sign convention on current:
 *   Daly raw -> A: (raw - 30000) * 0.1
 *   g_pt.bms_i_bat_cA wants 0.01 A LSB, signed (+ discharge).
 *   The driver doesn't document raw-delta sign explicitly. Verify against
 *   a known load on first bench test; flip DALY_CURRENT_SIGN if the meaning
 *   is inverted on your unit.
 */

#include "bms_task.h"
#include "can_manager.h"
#include "powertrain_state.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <string.h>
#include <stdint.h>

/* ------------ Tunables ------------ */

#define BMS_RX_QUEUE_DEPTH        16
#define BMS_RESPONSE_TIMEOUT_MS  200       /* per-command timeout */
#define BMS_INTER_CMD_DELAY_MS    25       /* matches the Daly driver */
#define BMS_STALE_MS            3000       /* declare BMS lost after 3 s of silence */

/* If your BMS reports current with the opposite sign of what we want,
 * flip this to -1. Default +1 = Daly raw delta is +discharge. */
#define DALY_CURRENT_SIGN         (+1)

/* ------------ Daly addressing & ID layout ------------ */

#define DALY_PREFIX               0x18u
#define DALY_ADDR_BMS_MASTER      0x01u
#define DALY_ADDR_UPPER_PC        0x40u

/* TX (host -> BMS) ID = 0x18 [cmd] 01 40 */
#define DALY_TX_ID(cmd)  (((uint32_t)DALY_PREFIX << 24)            | \
                          ((uint32_t)(cmd) << 16)                  | \
                          ((uint32_t)DALY_ADDR_BMS_MASTER <<  8)   | \
                          ((uint32_t)DALY_ADDR_UPPER_PC))

/* RX (BMS -> host) base ID = 0x18 00 40 01 ; mask off cmd byte. */
#define DALY_RX_ID_BASE  (((uint32_t)DALY_PREFIX << 24)            | \
                          ((uint32_t)DALY_ADDR_UPPER_PC <<  8)     | \
                          ((uint32_t)DALY_ADDR_BMS_MASTER))
#define DALY_RX_ID_MASK   0xFF00FFFFu

#define DALY_RX_CMD(id)   ((uint8_t)(((id) >> 16) & 0xFFu))

/* Daly command bytes (subset that maps to g_pt.bms_*) */
#define DALY_CMD_SOC_VIS          0x90u
#define DALY_CMD_TEMP_MM          0x92u

/* Conversion offsets (from the Daly driver) */
#define DALY_CURRENT_OFFSET      30000
#define DALY_TEMP_OFFSET            40

/* F405 bring-up bus. Move to CAN_BUS_BATTERY on H7. */
#define BMS_BUS                  CAN_BUS_POWERTRAIN

/* ------------ Internal state ------------ */

static StaticTask_t       s_tcb;
static StackType_t        s_stack[256];          /* 1 KB */

static StaticQueue_t      s_rxq_cb;
static can_frame_t        s_rxq_items[BMS_RX_QUEUE_DEPTH];
static osMessageQueueId_t s_rx_q;

static void bms_task(void *arg);

/* ------------ Helpers ------------ */

static inline uint16_t get_u16_be(const uint8_t *p) {
    return ((uint16_t)p[0] << 8) | (uint16_t)p[1];
}

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static void drain_rx_queue(void) {
    can_frame_t scratch;
    while (osMessageQueueGet(s_rx_q, &scratch, NULL, 0) == osOK) { /* discard */ }
}

static bool send_request(uint8_t cmd) {
    can_frame_t f;
    memset(&f, 0, sizeof(f));
    f.id  = DALY_TX_ID(cmd);
    f.ext = 1;
    f.rtr = 0;
    f.dlc = 8;
    /* data bytes are zero per the Daly driver */
    return can_mgr_send(BMS_BUS, &f, 0);
}

/* Wait up to timeout_ms for a response whose cmd byte matches.
 * Mismatched-cmd frames are discarded but still count toward timeout. */
static bool await_response(uint8_t cmd, can_frame_t *out, uint32_t timeout_ms) {
    const uint32_t start = osKernelGetTickCount();
    for (;;) {
        const uint32_t elapsed = osKernelGetTickCount() - start;
        if (elapsed >= timeout_ms) return false;
        const uint32_t remaining = timeout_ms - elapsed;

        if (osMessageQueueGet(s_rx_q, out, NULL, remaining) != osOK) {
            return false;
        }
        if (DALY_RX_CMD(out->id) == cmd) return true;
        /* else: not for us; keep waiting */
    }
}

/* ------------ Decoders ------------ */

static void decode_soc_vis(const uint8_t *d) {
    /* bytes 0..1 = total V (0.1 V LSB, big-endian)
     * bytes 4..5 = current raw (0.1 A LSB, offset 30000)
     * bytes 6..7 = SOC (0.1 % LSB) */
    const uint16_t v_raw   = get_u16_be(&d[0]);
    const uint16_t i_raw   = get_u16_be(&d[4]);
    const uint16_t soc_raw = get_u16_be(&d[6]);

    const uint32_t v_calc = (uint32_t)v_raw * 10u;       /* 0.1 V -> 0.01 V */
    const int32_t  i_dA   = (int32_t)i_raw - DALY_CURRENT_OFFSET;
    const int32_t  i_cA   = (DALY_CURRENT_SIGN) * i_dA * 10;  /* 0.1 A -> 0.01 A */
    const uint32_t s_calc = (uint32_t)soc_raw * 10u;     /* 0.1 % -> 0.01 % */

    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.bms_v_bat_cV   = (v_calc > 0xFFFFu) ? 0xFFFFu : (uint16_t)v_calc;
    g_pt.bms_i_bat_cA   = (int16_t)clamp_i32(i_cA, INT16_MIN, INT16_MAX);
    g_pt.bms_soc_pct    = (s_calc > 10000u) ? 10000u : (uint16_t)s_calc;
    g_pt.bms_input_tick = osKernelGetTickCount();
    osMutexRelease(g_pt_mtx);
}

static void decode_temp_mm(const uint8_t *d) {
    /* byte 0 = max cell temp raw (offset 40 -> °C). bytes 1..3 carry
     * sensor positions and min temp; we only need the max for g_pt. */
    const int32_t t_C = (int32_t)d[0] - DALY_TEMP_OFFSET;

    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.bms_max_cell_C = (int8_t)clamp_i32(t_C, INT8_MIN, INT8_MAX);
    g_pt.bms_input_tick = osKernelGetTickCount();
    osMutexRelease(g_pt_mtx);
}

/* Round-robin command list. Add 0x91/0x98/etc here when their decoders
 * land and corresponding fields are in g_pt. */
static const uint8_t poll_seq[] = {
    DALY_CMD_SOC_VIS,
    DALY_CMD_TEMP_MM,
};
#define POLL_SEQ_LEN ((uint8_t)(sizeof(poll_seq)/sizeof(poll_seq[0])))

/* ------------ Task ------------ */

void bms_task_start(void) {
    static const osMessageQueueAttr_t qattr = {
        .name    = "bms_rxq",
        .cb_mem  = &s_rxq_cb,
        .cb_size = sizeof(s_rxq_cb),
        .mq_mem  = s_rxq_items,
        .mq_size = sizeof(s_rxq_items),
    };
    s_rx_q = osMessageQueueNew(BMS_RX_QUEUE_DEPTH,
                               sizeof(can_frame_t), &qattr);

    /* Match every Daly response: 0x18 [any cmd] 40 01 */
    can_mgr_subscribe(BMS_BUS, DALY_RX_ID_BASE, DALY_RX_ID_MASK, true, s_rx_q);

    static const osThreadAttr_t tattr = {
        .name       = "bms",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityNormal,        /* prio 3 */
    };
    osThreadNew(bms_task, NULL, &tattr);
}

static void bms_task(void *arg) {
    (void)arg;
    uint8_t poll_idx = 0;

    for (;;) {
        const uint8_t cmd = poll_seq[poll_idx];
        bool ok = false;

        drain_rx_queue();
        if (send_request(cmd)) {
            can_frame_t resp;
            if (await_response(cmd, &resp, BMS_RESPONSE_TIMEOUT_MS) &&
                resp.dlc == 8) {
                switch (cmd) {
                case DALY_CMD_SOC_VIS:  decode_soc_vis(resp.data); ok = true; break;
                case DALY_CMD_TEMP_MM:  decode_temp_mm(resp.data); ok = true; break;
                default:                                                      break;
                }
            }
        }

        if (ok) pt_clear_fault(FAULT_BMS_STALE);

        /* Stale arms only after the first successful parse, so a
         * never-connected BMS doesn't constantly raise the fault. */
        osMutexAcquire(g_pt_mtx, osWaitForever);
        const uint32_t last = g_pt.bms_input_tick;
        osMutexRelease(g_pt_mtx);
        if (last && (osKernelGetTickCount() - last) > BMS_STALE_MS) {
            pt_set_fault(FAULT_BMS_STALE);
        }

        poll_idx = (uint8_t)((poll_idx + 1u) % POLL_SEQ_LEN);
        osDelay(BMS_INTER_CMD_DELAY_MS);
    }
}
