#ifndef DYNO_LOAD_TASK_H
#define DYNO_LOAD_TASK_H

/* Bench-only: 100 Hz PID drives load ESC PWM (PA10/TIM1_CH3) to make I_load
 * (ACS ch0) track g_pt.dyno_load_i_target_mA. Gated by g_pt.dyno_load_active. */
void dyno_load_task_start(void);

#endif /* DYNO_LOAD_TASK_H */
