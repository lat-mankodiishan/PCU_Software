/* rtos_stats — DWT CYCCNT source + 5 s SWO dump of per-task CPU%/stack HWM. */

#include "rtos_stats.h"
#include "powertrain_state.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"
#include <stdio.h>
#include <string.h>

/* ---- DWT cycle counter ---- */

void rtos_stats_init_counter(void) {
    /* Enable TRCENA + CYCCNT; idempotent. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t rtos_stats_counter(void) {
    return DWT->CYCCNT;
}

/* ---- Periodic dump task ---- */

#define STATS_PERIOD_MS    5000u
#define STATS_MAX_TASKS    16u

static StaticTask_t s_tcb;
static StackType_t  s_stack[512];    /* 2 KB */

static TaskStatus_t s_status_now [STATS_MAX_TASKS];
static TaskStatus_t s_status_prev[STATS_MAX_TASKS];
static UBaseType_t  s_n_prev        = 0;
static uint32_t     s_total_prev    = 0;

static void stats_task(void *arg);

void rtos_stats_task_start(void) {
    static const osThreadAttr_t tattr = {
        .name       = "rtos_stats",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityLow,         /* prio 2 */
    };
    osThreadNew(stats_task, NULL, &tattr);
}

static const TaskStatus_t *find_by_handle(const TaskStatus_t *arr, UBaseType_t n,
                                          TaskHandle_t h) {
    for (UBaseType_t i = 0; i < n; ++i) {
        if (arr[i].xHandle == h) return &arr[i];
    }
    return NULL;
}

static void stats_task(void *arg) {
    (void)arg;

    /* First window only captures; dump starts on second tick. */
    uint32_t next = osKernelGetTickCount();

    for (;;) {
        uint32_t     total_now = 0;
        UBaseType_t  n_now     = uxTaskGetSystemState(s_status_now,
                                                      STATS_MAX_TASKS,
                                                      &total_now);

        if (s_n_prev > 0) {
            const uint32_t total_delta = total_now - s_total_prev;
            uint32_t idle_delta = 0;

            if (total_delta > 0) {
                printf("[rtos-stats] tasks=%u window-ticks=%lu\r\n",
                       (unsigned)n_now, (unsigned long)total_delta);
                printf("  task              stack-hwm-words  cpu%%\r\n");

                for (UBaseType_t i = 0; i < n_now; ++i) {
                    const TaskStatus_t *cur  = &s_status_now[i];
                    const TaskStatus_t *prev = find_by_handle(s_status_prev,
                                                              s_n_prev,
                                                              cur->xHandle);
                    uint32_t task_delta = cur->ulRunTimeCounter
                        - (prev ? prev->ulRunTimeCounter : 0);

                    /* 64-bit mul to avoid % overflow. */
                    uint32_t pct = (uint32_t)(((uint64_t)task_delta * 100u)
                                              / total_delta);

                    if (strcmp(cur->pcTaskName, "IDLE") == 0) {
                        idle_delta = task_delta;
                    }

                    printf("  %-16s  %14u  %4u\r\n",
                           cur->pcTaskName,
                           (unsigned)cur->usStackHighWaterMark,
                           (unsigned)pct);
                }

                /* load = 100 - idle%. */
                uint32_t idle_pct = (uint32_t)(((uint64_t)idle_delta * 100u)
                                               / total_delta);
                uint8_t  load_pct = (idle_pct >= 100u) ? 0u
                                                       : (uint8_t)(100u - idle_pct);

                osMutexAcquire(g_pt_mtx, osWaitForever);
                g_pt.cpu.load_pct = load_pct;
                g_pt.cpu.tick     = osKernelGetTickCount();
                osMutexRelease(g_pt_mtx);
            }
        }

        memcpy(s_status_prev, s_status_now, sizeof(s_status_now[0]) * n_now);
        s_n_prev     = n_now;
        s_total_prev = total_now;

        next += STATS_PERIOD_MS;
        osDelayUntil(next);
    }
}
