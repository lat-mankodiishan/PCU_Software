#ifndef RTOS_STATS_H
#define RTOS_STATS_H

#include <stdint.h>

/* DWT CYCCNT shim for FreeRTOS run-time stats. */
void     rtos_stats_init_counter(void);
uint32_t rtos_stats_counter(void);

/* Periodic 5 s SWO dump + mirror to g_pt.cpu_load_pct. */
void rtos_stats_task_start(void);

#endif /* RTOS_STATS_H */
