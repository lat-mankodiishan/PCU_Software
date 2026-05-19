#ifndef THROTTLE_CTRL_TASK_H
#define THROTTLE_CTRL_TASK_H

/* PI closed-loop on PA10 (TIM1_CH3 PWM @ 25 kHz). Drives the dyno load
 * current (4FX feedback) to match the 5FX reference current. Loop runs at
 * 100 Hz; gain globals (g_throttle_kp_q15, g_throttle_ki_q15) and the
 * enable flag (g_pt.throttle_ctrl_enabled) are runtime-tunable. */
void throttle_ctrl_task_start(void);

#endif /* THROTTLE_CTRL_TASK_H */
