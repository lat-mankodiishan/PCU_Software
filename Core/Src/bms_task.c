/*
 * bms_task.c — placeholder.
 *
 * Subscribes match-all on the battery-side CAN bus, drains RX into a TODO
 * parser stub, and runs a stale-detection check. When the BMS protocol is
 * known: narrow the can_mgr_subscribe filter to the BMS message IDs and
 * fill in parse_bms_frame() to populate g_pt.bms_*.
 *
 * Architecture target: BMS lives on Bus 3 (battery path) on the H7. F405
 * has no Bus 3, so for bring-up we listen on CAN_BUS_ENGINE — adjust when
 * the H7 board is in.
 */

#include "bms_task.h"
#include "can_manager.h"
#include "powertrain_state.h"











#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define BMS_PERIOD_MS         100         /* 10 Hz */
#define BMS_STALE_MS         1000         /* declare BMS lost after 1 s */
#define BMS_RX_QUEUE_DEPTH      8

/* Where BMS lives on F405 bring-up. Move to CAN_BUS_BATTERY when H7 lands. */
#define BMS_BUS              CAN_BUS_ENGINE

static StaticTask_t       s_tcb;
static StackType_t        s_stack[256];   /* 1 KB */

static StaticQueue_t      s_rxq_cb;
static can_frame_t        s_rxq_items[BMS_RX_QUEUE_DEPTH];
static osMessageQueueId_t s_rx_q;

static void bms_task(void *arg);

/* TODO: fill in once the BMS CAN protocol is known.
 *  - decode the frame
 *  - acquire g_pt_mtx
 *  - update g_pt.bms_soc_pct / bms_v_bat_cV / bms_i_bat_cA / bms_max_cell_C
 *  - update g_pt.bms_input_tick = osKernelGetTickCount()
 *  - release mutex
 *  - return true on a successful parse so the caller can clear FAULT_BMS_STALE
 */
static bool parse_bms_frame(const can_frame_t *f) {
    (void)f;
    return false;
}

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

    /* TODO: narrow this filter to the BMS message IDs once protocol is known.
     * Today: match-all extended frames so any traffic shows up while bringing
     * up the bus. Consumes one HW filter bank. */
    can_mgr_subscribe(BMS_BUS, 0u, 0u, true, s_rx_q);

    static const osThreadAttr_t tattr = {
        .name       = "bms",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityNormal,         /* prio 3 */
    };
    osThreadNew(bms_task, NULL, &tattr);
}

static void bms_task(void *arg) {
    (void)arg;
    uint32_t next = osKernelGetTickCount();

    for (;;) {
        /* Drain RX. parse_bms_frame is a stub today — returns false → no
         * tick update → no fault clear. Once the parser is real, successful
         * decodes update g_pt and clear FAULT_BMS_STALE. */
        can_frame_t f;
        while (osMessageQueueGet(s_rx_q, &f, NULL, 0) == osOK) {
            if (parse_bms_frame(&f)) {
                pt_clear_fault(FAULT_BMS_STALE);
            }
        }

        /* Stale detection — arms itself on first successful parse. Until
         * then bms_input_tick == 0 and we don't raise the fault. */
        osMutexAcquire(g_pt_mtx, osWaitForever);
        uint32_t last = g_pt.bms_input_tick;
        osMutexRelease(g_pt_mtx);
        if (last && (osKernelGetTickCount() - last) > BMS_STALE_MS) {
            pt_set_fault(FAULT_BMS_STALE);
        }

        next += BMS_PERIOD_MS;
        osDelayUntil(next);
    }
}
