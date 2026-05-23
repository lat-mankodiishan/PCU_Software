/* throttle_ctrl_task — PID on load current (4FX) vs filtered src (5FX). */

#include "throttle_ctrl_task.h"
#include "powertrain_state.h"
#include "periph_wrappers.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

#define LOOP_PERIOD_MS    10        /* 100 Hz */
#define ESC_MIN_US        1000u
#define ESC_MAX_US        2000u
#define ESC_ARM_HOLD_MS   2000

/* 50 samples x 10 ms = 500 ms window on 5FX. */
#define SRC_FILT_N        50

/* Engage when src_filt <= -engage_src_mA. */
volatile int32_t  g_throttle_engage_src_mA   = 5000;

/* PID divisors: bigger = weaker. Set to 0 to disable term. */
volatile int32_t  g_throttle_kp_div          = 100;
volatile int32_t  g_throttle_ki_div          = 0;
volatile int32_t  g_throttle_kd_div          = 0;

/* Pulse cap (us); 1700 = 70 %, 2000 = full authority. */
volatile uint16_t g_throttle_max_pulse_us    = 1700;

/* Non-zero bypasses PID; writes pulse directly. */
volatile uint16_t g_throttle_manual_pulse_us = 0;

volatile uint32_t g_throttle_loops = 0;

static StaticTask_t s_tcb;
static StackType_t  s_stack[256];     /* 1 KB */

static int32_t s_integrator_mA = 0;
static int32_t s_last_err_mA   = 0;

static int32_t s_src_buf[SRC_FILT_N];
static uint8_t s_src_idx = 0;
static int64_t s_src_sum = 0;

static uint16_t s_pulse = ESC_MIN_US;

static void throttle_task(void *arg);

static inline void pwm_set_pulse(uint16_t pulse_us) {
    if (pulse_us < ESC_MIN_US) pulse_us = ESC_MIN_US;
    if (pulse_us > ESC_MAX_US) pulse_us = ESC_MAX_US;
    esc_hw_set_us(ESC_CH_LOAD, pulse_us);
}

/* O(1) sliding window. */
static int32_t src_filter_step(int32_t new_sample) {
    s_src_sum -= s_src_buf[s_src_idx];
    s_src_buf[s_src_idx] = new_sample;
    s_src_sum += new_sample;
    s_src_idx = (uint8_t)((s_src_idx + 1) % SRC_FILT_N);
    return (int32_t)(s_src_sum / SRC_FILT_N);
}

static void pid_reset(void) {
    s_integrator_mA = 0;
    s_last_err_mA   = 0;
}

void throttle_ctrl_task_start(void) {
    /* Park CCR at idle. */
    esc_hw_init(ESC_CH_LOAD, ESC_MIN_US);

    static const osThreadAttr_t tattr = {
        .name       = "throttle",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityAboveNormal,
    };
    osThreadNew(throttle_task, NULL, &tattr);
}

static void throttle_task(void *arg) {
    (void)arg;

    osDelay(ESC_ARM_HOLD_MS);

    uint32_t next = osKernelGetTickCount();
    for (;;) {
        bool    enabled;
        int32_t src_mA, load_mA;
        osMutexAcquire(g_pt_mtx, osWaitForever);
        enabled = g_pt.throttle_ctrl_enabled;
        src_mA  = g_pt.dyno_current_mA;
        load_mA = g_pt.dyno_load_current_mA;
        osMutexRelease(g_pt_mtx);

        /* Always filter so window stays current when idle. */
        const int32_t src_filt_mA = src_filter_step(src_mA);
        int32_t err = 0;

        if (g_throttle_manual_pulse_us != 0) {
            s_pulse = g_throttle_manual_pulse_us;
            pid_reset();
        } else if (!enabled || src_filt_mA > -g_throttle_engage_src_mA) {
            s_pulse = ESC_MIN_US;
            pid_reset();
        } else {
            err = load_mA - src_filt_mA;

        const int32_t p_us = (g_throttle_kp_div > 0)
                              ? (err / g_throttle_kp_div)
                              : 0;

        /* I anti-windup: skip integrate when output pinned. */
        if (g_throttle_ki_div > 0) {
            const bool at_top = (s_pulse >= g_throttle_max_pulse_us);
            const bool at_bot = (s_pulse <= ESC_MIN_US);
            if (!(at_top && err > 0) && !(at_bot && err < 0)) {
                s_integrator_mA += err;
                const int32_t i_cap = g_throttle_ki_div *
                                      (int32_t)(g_throttle_max_pulse_us - ESC_MIN_US);
                if (s_integrator_mA >  i_cap) s_integrator_mA =  i_cap;
                if (s_integrator_mA < -i_cap) s_integrator_mA = -i_cap;
            }
        }
        const int32_t i_us = (g_throttle_ki_div > 0)
                              ? (s_integrator_mA / g_throttle_ki_div)
                              : 0;

        const int32_t d_us = (g_throttle_kd_div > 0)
                              ? ((err - s_last_err_mA) / g_throttle_kd_div)
                              : 0;
        s_last_err_mA = err;

            int32_t u_above = p_us + i_us + d_us;
            if (u_above < 0) u_above = 0;
            const int32_t max_above = (int32_t)g_throttle_max_pulse_us - (int32_t)ESC_MIN_US;
            if (u_above > max_above) u_above = max_above;
            s_pulse = (uint16_t)((int32_t)ESC_MIN_US + u_above);
        }

        pwm_set_pulse(s_pulse);

        osMutexAcquire(g_pt_mtx, osWaitForever);
        g_pt.throttle_pwm_duty       = s_pulse;
        g_pt.throttle_current_err_mA = err;
        g_pt.throttle_src_filt_mA    = src_filt_mA;
        g_pt.throttle_ctrl_tick      = osKernelGetTickCount();
        osMutexRelease(g_pt_mtx);

        g_throttle_loops++;
        next += LOOP_PERIOD_MS;
        osDelayUntil(next);
    }
}
