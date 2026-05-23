/*
 * rtos_stats.c — FreeRTOS runtime-stats counter source + periodic dump task.
 *
 * The scheduler calls rtos_stats_init_counter() once at start (via
 * portCONFIGURE_TIMER_FOR_RUN_TIME_STATS) and rtos_stats_counter() at every
 * context switch (via portGET_RUN_TIME_COUNTER_VALUE). Each task accumulates
 * cycles spent on-CPU into TCB->ulRunTimeCounter.
 *
 * The dump task wakes every 5 s, snapshots all task counters with
 * uxTaskGetSystemState, diffs against the previous snapshot, and prints
 * a per-task CPU % + stack high-water-mark table over SWO. It also writes
 * the IDLE-derived CPU load into g_pt.cpu_load_pct so other consumers
 * (log_task, Live Watch) can see it without parsing the printf stream.
 *
 * Cost: ~0 in steady state (DWT CYCCNT increments in hardware; the
 * stats task is osPriorityLow so it only runs when nothing else needs the
 * CPU). The 5-second printf is the only meaningful expense and it sits
 * below all real work tasks.
 */

#include "rtos_stats.h"
#include "powertrain_state.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "stm32f4xx_hal.h"      /* CoreDebug, DWT */
#include <stdio.h>
#include <string.h>

/* --- DWT cycle counter -------------------------------------------------- */

void rtos_stats_init_counter(void) {
    /* Enable trace + DWT clocking, reset and start CYCCNT. Idempotent.
     * Required before the scheduler reads it for the first time. */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT       = 0;
    DWT->CTRL        |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t rtos_stats_counter(void) {
    return DWT->CYCCNT;
}

/* --- Periodic dump task ------------------------------------------------- */

#define STATS_PERIOD_MS    5000u
#define STATS_MAX_TASKS    16u       /* upper bound; grows with task count */

static StaticTask_t s_tcb;
static StackType_t  s_stack[512];    /* 2 KB — printf + 2× status arrays need room */

/* Two snapshots so we can diff per-task run-time counters across windows. */
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
        .priority   = osPriorityLow,         /* prio 2 — below all real work */
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

    /* Don't dump during the first window — prev counters are uninitialized.
     * Just capture, then dump on the second tick onward. */
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

                    /* Use 64-bit multiply to avoid overflow when computing %. */
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

                /* Total CPU load = 100 - idle %. Stored as uint8. */
                uint32_t idle_pct = (uint32_t)(((uint64_t)idle_delta * 100u)
                                               / total_delta);
                uint8_t  load_pct = (idle_pct >= 100u) ? 0u
                                                       : (uint8_t)(100u - idle_pct);

                osMutexAcquire(g_pt_mtx, osWaitForever);
                g_pt.cpu_load_pct  = load_pct;
                g_pt.cpu_load_tick = osKernelGetTickCount();
                osMutexRelease(g_pt_mtx);
            }
        }

        /* Capture for next window. */
        memcpy(s_status_prev, s_status_now, sizeof(s_status_now[0]) * n_now);
        s_n_prev     = n_now;
        s_total_prev = total_now;

        next += STATS_PERIOD_MS;
        osDelayUntil(next);
    }
}
