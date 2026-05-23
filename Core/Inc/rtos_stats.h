#ifndef RTOS_STATS_H
#define RTOS_STATS_H

#include <stdint.h>

/* Cortex-M4 DWT cycle-counter shim used as FreeRTOS run-time-stats source.
 * Wired into the scheduler via portCONFIGURE_TIMER_FOR_RUN_TIME_STATS /
 * portGET_RUN_TIME_COUNTER_VALUE in FreeRTOSConfig.h. Safe to call before
 * the scheduler starts. */
void     rtos_stats_init_counter(void);
uint32_t rtos_stats_counter(void);

/* Create the periodic stats-dump task (low priority). Prints per-task CPU
 * % and stack high-water-mark via SWO every ~5 s, and mirrors the latest
 * overall CPU load into g_pt.cpu_load_pct / cpu_load_tick. */
void rtos_stats_task_start(void);

#endif /* RTOS_STATS_H */
