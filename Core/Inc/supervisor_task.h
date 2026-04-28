#ifndef SUPERVISOR_TASK_H
#define SUPERVISOR_TASK_H

/* Create the supervisor task. Call after pt_init() and rectifier_task_start(),
 * before osKernelStart(). */
void supervisor_task_start(void);

#endif /* SUPERVISOR_TASK_H */
