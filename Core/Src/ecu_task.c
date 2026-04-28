/*
 * ecu_task.c — placeholder.
 *
 * HFE International ECU on the engine bus (CAN2). Protocol likely J1939
 * (29-bit IDs, PGN-based) but unconfirmed. Same shape as bms_task: match-all
 * subscribe + stub parser. Fill in parse_ecu_frame() and narrow the filter
 * once the ECU's PGN list is documented.
 */

#include "ecu_task.h"
#include "can_manager.h"
#include "powertrain_state.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define ECU_PERIOD_MS         100         /* 10 Hz */
#define ECU_STALE_MS         1000         /* declare ECU lost after 1 s */
#define ECU_RX_QUEUE_DEPTH      8

static StaticTask_t       s_tcb;
static StackType_t        s_stack[256];   /* 1 KB */

static StaticQueue_t      s_rxq_cb;
static can_frame_t        s_rxq_items[ECU_RX_QUEUE_DEPTH];
static osMessageQueueId_t s_rx_q;

static void ecu_task(void *arg);

/* TODO: fill in once the ECU CAN protocol is known.
 *  - decode the frame (PGN if J1939, else raw ID match)
 *  - acquire g_pt_mtx
 *  - update g_pt.ecu_rpm / ecu_fuel_rate_dg_s / ecu_cht_C
 *  - update g_pt.ecu_input_tick = osKernelGetTickCount()
 *  - release mutex
 *  - return true so the caller clears FAULT_ECU_STALE
 */
static bool parse_ecu_frame(const can_frame_t *f) {
    (void)f;
    return false;
}

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

    /* TODO: narrow to ECU PGN range once protocol is confirmed.
     * Today: match-all extended frames on the engine bus. */
    can_mgr_subscribe(CAN_BUS_ENGINE, 0u, 0u, true, s_rx_q);

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
        can_frame_t f;
        while (osMessageQueueGet(s_rx_q, &f, NULL, 0) == osOK) {
            if (parse_ecu_frame(&f)) {
                pt_clear_fault(FAULT_ECU_STALE);
            }
        }

        osMutexAcquire(g_pt_mtx, osWaitForever);
        uint32_t last = g_pt.ecu_input_tick;
        osMutexRelease(g_pt_mtx);
        if (last && (osKernelGetTickCount() - last) > ECU_STALE_MS) {
            pt_set_fault(FAULT_ECU_STALE);
        }

        next += ECU_PERIOD_MS;
        osDelayUntil(next);
    }
}
