/* bms_task — parked; battery telemetry comes from FC-side Hobbywing ESC telem. */
#if 0

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

/* TODO: implement once BMS CAN protocol is known. */
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

    /* TODO: narrow filter to BMS IDs; match-all for bring-up. */
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
        can_frame_t f;
        while (osMessageQueueGet(s_rx_q, &f, NULL, 0) == osOK) {
            if (parse_bms_frame(&f)) {
                pt_clear_fault(FAULT_BMS_STALE);
            }
        }

        /* Stale detection arms on first parse (bms_input_tick == 0 until). */
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

#endif /* #if 0 — task disabled, see top-of-file note */
