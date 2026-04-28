#ifndef RECTIFIER_TASK_H
#define RECTIFIER_TASK_H

/* Create the rectifier task and subscribe to GetRectStateConcise on CAN2.
 * Call once, AFTER pt_init() and can_mgr_init(), BEFORE osKernelStart(). */
void rectifier_task_start(void);

#endif /* RECTIFIER_TASK_H */
