#include "can_manager.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define TX_QUEUE_DEPTH      16
#define RX_DISPATCH_DEPTH   32
#define MAX_SUBS_PER_BUS    8

typedef struct {
    uint32_t           id;
    uint32_t           mask;
    bool               ext;
    bool               in_use;
    osMessageQueueId_t dest;
} sub_entry_t;

typedef struct {
    can_bus_t          bus;          /* echoed for the dispatch task */
    osMessageQueueId_t tx_q;
    osMessageQueueId_t rx_q;
    sub_entry_t        subs[MAX_SUBS_PER_BUS];
    volatile uint32_t  last_rx_tick;
} bus_ctx_t;

/* --- static allocation for two buses' TX/RX queues + the dispatch task --- */
static bus_ctx_t g_bus[CAN_BUS_COUNT];

static StaticTask_t s_disp_tcb;
static StackType_t  s_disp_stack[512];      /* 2 KB */
static osThreadId_t s_disp_th;

static void rx_dispatch_task(void *arg);

void can_mgr_init(void) {
    for (int b = 0; b < CAN_BUS_COUNT; ++b) {
        g_bus[b].bus = (can_bus_t)b;
        g_bus[b].tx_q = osMessageQueueNew(TX_QUEUE_DEPTH, sizeof(can_frame_t), NULL);
        g_bus[b].rx_q = osMessageQueueNew(RX_DISPATCH_DEPTH,
                                          sizeof(can_frame_t), NULL);
        for (int i = 0; i < MAX_SUBS_PER_BUS; ++i) g_bus[b].subs[i].in_use = false;
        can_hw_init((can_bus_t)b);
    }

    const osThreadAttr_t attr = {
        .name       = "can_disp",
        .cb_mem     = &s_disp_tcb,
        .cb_size    = sizeof(s_disp_tcb),
        .stack_mem  = s_disp_stack,
        .stack_size = sizeof(s_disp_stack),
        .priority   = osPriorityAboveNormal,    /* 4: matches detector task tier */
    };
    s_disp_th = osThreadNew(rx_dispatch_task, NULL, &attr);
}

bool can_mgr_send(can_bus_t bus, const can_frame_t *f, uint32_t timeout_ms) {
    if (bus >= CAN_BUS_COUNT) return false;

    /* Fast path: try to hand directly to a free mailbox. */
    taskENTER_CRITICAL();
    if (can_hw_tx_free(bus) > 0 && osMessageQueueGetCount(g_bus[bus].tx_q) == 0) {
        bool ok = can_hw_try_post(bus, f);
        taskEXIT_CRITICAL();
        if (ok) return true;
    } else {
        taskEXIT_CRITICAL();
    }
    /* Slow path: queue. ISR drains on TX-complete. */
    return osMessageQueuePut(g_bus[bus].tx_q, f, 0, timeout_ms) == osOK;
}

bool can_mgr_subscribe(can_bus_t bus, uint32_t id, uint32_t mask, bool ext,
                       osMessageQueueId_t dest_queue) {
    if (bus >= CAN_BUS_COUNT || dest_queue == NULL) return false;
    for (int i = 0; i < MAX_SUBS_PER_BUS; ++i) {
        if (!g_bus[bus].subs[i].in_use) {
            g_bus[bus].subs[i] = (sub_entry_t){
                .id = id, .mask = mask, .ext = ext,
                .in_use = true, .dest = dest_queue,
            };
            /* Optimization later: coalesce subscribers' filters. For bring-up
             * each subscribe consumes one HW filter bank. */
            return can_hw_add_filter(bus, id, mask, ext) >= 0;
        }
    }
    return false;
}

uint32_t can_mgr_last_rx_tick(can_bus_t bus) {
    return (bus < CAN_BUS_COUNT) ? g_bus[bus].last_rx_tick : 0;
}

/* --- ISR hooks ---------------------------------------------------------- */

void can_mgr_isr_rx_fifo0(can_bus_t bus) {
    can_frame_t f;
    BaseType_t  hpw = pdFALSE;
    while (can_hw_read_fifo0(bus, &f)) {
        g_bus[bus].last_rx_tick = osKernelGetTickCount();   /* coarse — OK in ISR */
        /* xQueueSendFromISR via CMSIS: osMessageQueuePut with timeout=0 is ISR-safe
         * in CMSIS-RTOS v2 when called from ISR context. */
        (void)osMessageQueuePut(g_bus[bus].rx_q, &f, 0, 0);
    }
    portYIELD_FROM_ISR(hpw);
}

void can_mgr_isr_tx_complete(can_bus_t bus) {
    can_frame_t f;
    while (can_hw_tx_free(bus) > 0) {
        if (osMessageQueueGet(g_bus[bus].tx_q, &f, NULL, 0) != osOK) break;
        if (!can_hw_try_post(bus, &f)) {
            /* mailbox vanished between check and post — requeue at head not
             * possible with FreeRTOS queues; tail-requeue is acceptable for
             * non-ordered traffic. Revisit if we need strict ordering. */
            (void)osMessageQueuePut(g_bus[bus].tx_q, &f, 0, 0);
            break;
        }
    }
}

/* --- RX dispatch task --------------------------------------------------- */

static void rx_dispatch_task(void *arg) {
    (void)arg;
    can_frame_t f;
    for (;;) {
        for (int b = 0; b < CAN_BUS_COUNT; ++b) {
            while (osMessageQueueGet(g_bus[b].rx_q, &f, NULL, 0) == osOK) {
                for (int i = 0; i < MAX_SUBS_PER_BUS; ++i) {
                    sub_entry_t *s = &g_bus[b].subs[i];
                    if (!s->in_use) continue;
                    if (s->ext != f.ext) continue;
                    if (((f.id ^ s->id) & s->mask) == 0) {
                        (void)osMessageQueuePut(s->dest, &f, 0, 0);
                    }
                }
            }
        }
        /* Light sleep — the queues will get drained via the next wakeup.
         * Replace with osThreadFlagsWait once we plumb a flag from the ISR. */
        osDelay(1);
    }
}
