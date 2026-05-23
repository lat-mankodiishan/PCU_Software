#ifndef THROTTLE_CTRL_TASK_H
#define THROTTLE_CTRL_TASK_H

/* PI on PA10/TIM1_CH3: 4FX -> 5FX, 100 Hz; runtime-tunable globals. */
void throttle_ctrl_task_start(void);

#endif /* THROTTLE_CTRL_TASK_H */
