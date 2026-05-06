#include "rectifier_task.h"
#include "can_manager.h"
#include "vesc_proto.h"
#include "powertrain_state.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define RECT_PERIOD_MS         5            /* 200 Hz */
#define RECT_RX_QUEUE_DEPTH    8
#define RECT_STALE_MS          200

static StaticTask_t       s_tcb;
static StackType_t        s_stack[256];     /* 1 KB */

static StaticQueue_t      s_rxq_cb;
static can_frame_t        s_rxq_items[RECT_RX_QUEUE_DEPTH];
static osMessageQueueId_t s_rx_q;

static void rectifier_task(void *arg);

static inline int16_t clamp_i16(int16_t v, int16_t lo, int16_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void rectifier_task_start(void) {
    static const osMessageQueueAttr_t qattr = {
        .name    = "rect_rxq",
        .cb_mem  = &s_rxq_cb,
        .cb_size = sizeof(s_rxq_cb),
        .mq_mem  = s_rxq_items,
        .mq_size = sizeof(s_rxq_items),
    };
    s_rx_q = osMessageQueueNew(RECT_RX_QUEUE_DEPTH,
                               sizeof(can_frame_t),
                               &qattr);

    can_mgr_subscribe(CAN_BUS_ENGINE,
                      VESC_ID_GET_RECT_STATE_CONCISE,
                      0x7FFu,                    /* exact-match 11-bit */
                      false,
                      s_rx_q);

    static const osThreadAttr_t tattr = {
        .name       = "rectifier",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityAboveNormal,     /* 4 */
    };
    osThreadNew(rectifier_task, NULL, &tattr);
}

static void rectifier_task(void *arg) {
    (void)arg;
    uint8_t   seq  = 0;
    uint32_t  next = osKernelGetTickCount();
    can_frame_t f;

    for (;;) {
        /* --- RX drain ------------------------------------------------- */
        while (osMessageQueueGet(s_rx_q, &f, NULL, 0) == osOK) {
            vesc_rect_state_t s;
            if (vesc_proto_decode_rect_state_concise(&f, &s) != VESC_DECODE_OK)
                continue;
            osMutexAcquire(g_pt_mtx, osWaitForever);
            g_pt.rect_state      = s;
            g_pt.rect_state_tick = osKernelGetTickCount();
            osMutexRelease(g_pt_mtx);
            pt_clear_fault(FAULT_RECT_STALE);
        }

        /* --- TX setpoint snapshot, hardware-of-last-resort clamp ------ */
        vesc_curr_dem_t cmd;
        osMutexAcquire(g_pt_mtx, osWaitForever);
        cmd.I_rect_cmd_cA = clamp_i16(g_pt.I_rect_cmd_cA,
                                      -I_RECT_MAX_CA, I_RECT_MAX_CA);
        cmd.mode          = g_pt.mode;
        osMutexRelease(g_pt_mtx);
        cmd.seq = seq++;

        can_frame_t tx;
        vesc_proto_encode_curr_dem(&cmd, &tx);
        (void)can_mgr_send(CAN_BUS_ENGINE, &tx, 0);

        /* --- Staleness check ------------------------------------------ */
        osMutexAcquire(g_pt_mtx, osWaitForever);
        uint32_t last = g_pt.rect_state_tick;
        osMutexRelease(g_pt_mtx);
        if (last && (osKernelGetTickCount() - last) > RECT_STALE_MS) {
            pt_set_fault(FAULT_RECT_STALE);
        }

        next += RECT_PERIOD_MS;
        osDelayUntil(next);
    }
}
