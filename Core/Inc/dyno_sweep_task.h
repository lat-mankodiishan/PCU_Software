#ifndef DYNO_SWEEP_TASK_H
#define DYNO_SWEEP_TASK_H

/* Bench-only task: invokes one experiment profile per edge-triggered start.
 * Edit EXPT_BOOT_PROFILE in dyno_sweep_task.c to switch profiles. */
void dyno_sweep_task_start(void);

#endif /* DYNO_SWEEP_TASK_H */
