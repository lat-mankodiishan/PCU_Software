#ifndef LOG_TASK_H
#define LOG_TASK_H

/* Create the log task. Call after pt_init() and sensor_task_start(),
 * before osKernelStart(). */
void log_task_start(void);

#endif /* LOG_TASK_H */
