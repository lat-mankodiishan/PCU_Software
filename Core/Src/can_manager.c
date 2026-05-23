#include "can_manager.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#define TX_QUEUE_DEPTH      64
#define RX_DISPATCH_DEPTH   64
#define MAX_SUBS_PER_BUS    8

typedef struct {
    uint32_t           id;
    uint32_t           mask;
    bool               ext;
    bool               in_use;
    osMessageQueueId_t dest;
} sub_entry_t;

typedef struct {
    can_bus_t          bus;
    osMessageQueueId_t tx_q;
    osMessageQueueId_t rx_q;
    sub_entry_t        subs[MAX_SUBS_PER_BUS];
    volatile uint32_t  last_rx_tick;
} bus_ctx_t;

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
        .priority   = osPriorityAboveNormal,    /* prio 4 */
    };
    s_disp_th = osThreadNew(rx_dispatch_task, NULL, &attr);
}

bool can_mgr_send(can_bus_t bus, const can_frame_t *f, uint32_t timeout_ms) {
    if (bus >= CAN_BUS_COUNT) return false;

    /* Fast path: direct mailbox handoff. */
    taskENTER_CRITICAL();
    if (can_hw_tx_free(bus) > 0 && osMessageQueueGetCount(g_bus[bus].tx_q) == 0) {
        bool ok = can_hw_try_post(bus, f);
        taskEXIT_CRITICAL();
        if (ok) return true;
    } else {
        taskEXIT_CRITICAL();
    }
    /* Slow path: queue; ISR drains on TX-complete. */
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
            /* TODO: coalesce filters; today each subscribe burns one bank. */
            return can_hw_add_filter(bus, id, mask, ext) >= 0;
        }
    }
    return false;
}

uint32_t can_mgr_last_rx_tick(can_bus_t bus) {
    return (bus < CAN_BUS_COUNT) ? g_bus[bus].last_rx_tick : 0;
}

/* ---- ISR hooks ---- */

void can_mgr_isr_rx_fifo0(can_bus_t bus) {
    can_frame_t f;
    BaseType_t  hpw = pdFALSE;
    while (can_hw_read_fifo0(bus, &f)) {
        g_bus[bus].last_rx_tick = osKernelGetTickCount();
        /* osMessageQueuePut(timeout=0) is ISR-safe in CMSIS-RTOS v2. */
        (void)osMessageQueuePut(g_bus[bus].rx_q, &f, 0, 0);
    }
    portYIELD_FROM_ISR(hpw);
}

void can_mgr_isr_tx_complete(can_bus_t bus) {
    can_frame_t f;
    while (can_hw_tx_free(bus) > 0) {
        if (osMessageQueueGet(g_bus[bus].tx_q, &f, NULL, 0) != osOK) break;
        if (!can_hw_try_post(bus, &f)) {
            /* Tail-requeue; FreeRTOS has no head-requeue. */
            (void)osMessageQueuePut(g_bus[bus].tx_q, &f, 0, 0);
            break;
        }
    }
}

/* ---- RX dispatch task ---- */

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
        /* TODO: replace polling with osThreadFlagsWait from ISR. */
        osDelay(1);
    }
}
