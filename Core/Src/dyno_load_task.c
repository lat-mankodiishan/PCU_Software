/* dyno_load_task — PID on I_load (ACS ch0) -> load fan ESC PWM (PA10). */

#include "dyno_load_task.h"
#include "powertrain_state.h"
#include "periph_wrappers.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>
#include <stdbool.h>

#define LOOP_PERIOD_MS    10        /* 100 Hz */
#define ESC_MIN_US        1000u
#define ESC_MAX_US        2000u
#define ESC_ARM_HOLD_MS   2000

/* Which ACS772 channel is wired to I_load. Confirm against board before trusting. */
#ifndef DYNO_LOAD_ACS_CH
#define DYNO_LOAD_ACS_CH  0
#endif

/* PID gains: integer divisors. Bigger = weaker; 0 disables that term.
 * err is in mA, output increment is in us. kp_div = mA-per-us. */
volatile int32_t  g_dyno_load_kp_div          = 100;
volatile int32_t  g_dyno_load_ki_div          = 0;
volatile int32_t  g_dyno_load_kd_div          = 0;

/* Output cap (us); 1700 = 70 % of ESC range, conservative starting point. */
volatile uint16_t g_dyno_load_max_pulse_us    = 1700;

/* I_load EMA cutoff: Q15 alpha. ~3000 ≈ 1.5 Hz at 100 Hz tick. */
volatile uint16_t g_dyno_load_filt_alpha_q15  = 3000;

/* Non-zero bypasses PID; writes pulse directly. */
volatile uint16_t g_dyno_load_manual_pulse_us = 0;

volatile uint32_t g_dyno_load_loops           = 0;

static StaticTask_t s_tcb;
static StackType_t  s_stack[256];     /* 1 KB */

static int32_t s_integrator_mA = 0;
static int32_t s_last_err_mA   = 0;
static int32_t s_i_filt_mA     = 0;
static bool    s_filt_init     = false;
static uint16_t s_pulse        = ESC_MIN_US;

static void dyno_load_task(void *arg);

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

static inline void pwm_set_pulse(uint16_t pulse_us) {
    if (pulse_us < ESC_MIN_US) pulse_us = ESC_MIN_US;
    if (pulse_us > ESC_MAX_US) pulse_us = ESC_MAX_US;
    esc_hw_set_us(ESC_CH_LOAD, pulse_us);
}

static void pid_reset(void) {
    s_integrator_mA = 0;
    s_last_err_mA   = 0;
}

void dyno_load_task_start(void) {
    esc_hw_init(ESC_CH_LOAD, ESC_MIN_US);

    static const osThreadAttr_t tattr = {
        .name       = "dyno_load",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityAboveNormal,
    };
    osThreadNew(dyno_load_task, NULL, &tattr);
}

static void dyno_load_task(void *arg) {
    (void)arg;

    osDelay(ESC_ARM_HOLD_MS);

    uint32_t next = osKernelGetTickCount();
    for (;;) {
        bool    active;
        int32_t i_target_mA;
        int32_t i_meas_mA;
        osMutexAcquire(g_pt_mtx, osWaitForever);
        active      = g_pt.dyno_load_active;
        i_target_mA = g_pt.dyno_load_i_target_mA;
        i_meas_mA   = g_pt.current_sensor_mA[DYNO_LOAD_ACS_CH];
        osMutexRelease(g_pt_mtx);

        /* EMA on I_load; ride through ACS noise. */
        if (!s_filt_init) {
            s_i_filt_mA = i_meas_mA;
            s_filt_init = true;
        } else {
            int32_t err_in = i_meas_mA - s_i_filt_mA;
            int32_t adj    = (err_in * (int32_t)g_dyno_load_filt_alpha_q15) >> 15;
            s_i_filt_mA   += adj;
        }

        int32_t err = 0;

        if (g_dyno_load_manual_pulse_us != 0) {
            s_pulse = g_dyno_load_manual_pulse_us;
            pid_reset();
        } else if (!active) {
            s_pulse = ESC_MIN_US;
            pid_reset();
        } else {
            err = i_target_mA - s_i_filt_mA;

            const int32_t p_us = (g_dyno_load_kp_div > 0)
                                  ? (err / g_dyno_load_kp_div)
                                  : 0;

            /* I anti-windup: skip integrate when output pinned in error direction. */
            if (g_dyno_load_ki_div > 0) {
                const bool at_top = (s_pulse >= g_dyno_load_max_pulse_us);
                const bool at_bot = (s_pulse <= ESC_MIN_US);
                if (!(at_top && err > 0) && !(at_bot && err < 0)) {
                    s_integrator_mA += err;
                    const int32_t i_cap = g_dyno_load_ki_div *
                                          (int32_t)(g_dyno_load_max_pulse_us - ESC_MIN_US);
                    s_integrator_mA = clamp_i32(s_integrator_mA, -i_cap, i_cap);
                }
            }
            const int32_t i_us = (g_dyno_load_ki_div > 0)
                                  ? (s_integrator_mA / g_dyno_load_ki_div)
                                  : 0;

            const int32_t d_us = (g_dyno_load_kd_div > 0)
                                  ? ((err - s_last_err_mA) / g_dyno_load_kd_div)
                                  : 0;
            s_last_err_mA = err;

            int32_t u_above = p_us + i_us + d_us;
            const int32_t max_above = (int32_t)g_dyno_load_max_pulse_us - (int32_t)ESC_MIN_US;
            u_above = clamp_i32(u_above, 0, max_above);
            s_pulse = (uint16_t)((int32_t)ESC_MIN_US + u_above);
        }

        pwm_set_pulse(s_pulse);

        osMutexAcquire(g_pt_mtx, osWaitForever);
        g_pt.dyno_load_pwm_us  = s_pulse;
        g_pt.dyno_load_err_mA  = err;
        g_pt.dyno_load_i_filt_mA = s_i_filt_mA;
        g_pt.dyno_load_tick    = osKernelGetTickCount();
        osMutexRelease(g_pt_mtx);

        g_dyno_load_loops++;
        next += LOOP_PERIOD_MS;
        osDelayUntil(next);
    }
}
