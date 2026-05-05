#ifndef EXPERIMENT_TASK_H
#define EXPERIMENT_TASK_H

/* FreeRTOS task wrapper that invokes one selected profile at boot.
 * Edit EXPT_BOOT_PROFILE in experiment_task.c to switch profiles. */
void expt_task_start(void);

#endif /* EXPERIMENT_TASK_H */
