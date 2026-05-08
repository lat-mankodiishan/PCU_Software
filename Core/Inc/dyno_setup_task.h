#ifndef DYNO_SETUP_TASK_H
#define DYNO_SETUP_TASK_H

/* Dyno bench-test task. Subscribes to SSD-250A current sensor (ID 5FX-3)
 * broadcasts on CAN1 and populates g_pt.dyno_*. No control authority —
 * read-only telemetry for bench characterization. */
void dyno_setup_task_start(void);

#endif /* DYNO_SETUP_TASK_H */
