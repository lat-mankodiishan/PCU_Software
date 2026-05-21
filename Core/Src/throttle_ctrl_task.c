/*
 * throttle_ctrl_task.c — load-current tracking PID throttle.
 *
 * Drives PA10 / TIM1_CH3 with a 50 Hz RC-servo PWM (1000–2000 µs) for a
 * Hobbywing 200 A ESC. A PID closes the load current (4FX) onto the
 * filtered source current (5FX).
 *
 *   Sign convention on this rig:
 *     dyno_current_mA      (5FX) : POSITIVE motoring, NEGATIVE rectifying
 *     dyno_load_current_mA (4FX) : NEGATIVE when load is consuming
 *
 *   err = load_mA − src_filt_mA
 *      load less negative than src  →  err > 0  →  throttle ↑
 *      load equal to src            →  err = 0  →  hold
 *      load more negative than src  →  err < 0  →  throttle ↓
 *
 * Gains are stored as integer DIVISORS, so bigger number = WEAKER gain.
 * This avoids Q-format math and reads naturally:
 *
 *   "Every Kp_div_mA of error contributes 1 µs to the P term."
 *
 * Set a divisor to 0 to disable that term.
 */

#include "throttle_ctrl_task.h"
#include "powertrain_state.h"
#include "periph_wrappers.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdint.h>

#define LOOP_PERIOD_MS    10        /* 100 Hz tick */
#define ESC_MIN_US        1000u     /* idle / armed pulse */
#define ESC_MAX_US        2000u     /* full throttle pulse */
#define ESC_ARM_HOLD_MS   2000      /* hold idle this long after boot */

/* Moving-average filter on 5FX. 50 samples × 10 ms = 500 ms window.
 * Switched from a power-of-2 mask to a modulo to land exactly on 50. */
#define SRC_FILT_N        50

/* --- Tunables (volatile so they survive optimisation and can be poked) -- */

/* Engage threshold — magnitude of negative 5FX needed to start the loop. */
volatile int32_t  g_throttle_engage_src_mA   = 5000;   /* engage at src ≤ -5 A */

/* PID divisor gains. Bigger number = weaker contribution.
 *  Kp_div = 100   → 100 mA of error → 1 µs of P term
 *  Ki_div = 10000 → integral of 10 000 mA·tick → 1 µs of I term
 *  Kd_div = 0     → D term disabled (start without it, add if needed) */
volatile int32_t  g_throttle_kp_div          = 100;
volatile int32_t  g_throttle_ki_div          = 0;
volatile int32_t  g_throttle_kd_div          = 0;

/* Hard cap on the output pulse (µs). 1700 µs = 70 % throttle, leaving
 * 300 µs of headroom below ESC_MAX_US. Raise to 2000 for full authority. */
volatile uint16_t g_throttle_max_pulse_us    = 1700;

/* Bench-test manual override. Non-zero → bypass PID and write directly. */
volatile uint16_t g_throttle_manual_pulse_us = 0;

/* Diagnostic counter */
volatile uint32_t g_throttle_loops = 0;

/* --- Static state ------------------------------------------------------- */

static StaticTask_t s_tcb;
static StackType_t  s_stack[256];     /* 1 KB */

/* PID state */
static int32_t s_integrator_mA = 0;   /* sum of err over time */
static int32_t s_last_err_mA   = 0;   /* for D term */

/* Filter state (5FX moving average) */
static int32_t s_src_buf[SRC_FILT_N];
static uint8_t s_src_idx = 0;
static int64_t s_src_sum = 0;

/* PWM state */
static uint16_t s_pulse = ESC_MIN_US;

static void throttle_task(void *arg);

/* ----------------------------------------------------------------------- */

static inline void pwm_set_pulse(uint16_t pulse_us) {
    if (pulse_us < ESC_MIN_US) pulse_us = ESC_MIN_US;
    if (pulse_us > ESC_MAX_US) pulse_us = ESC_MAX_US;
    esc_hw_set_us(ESC_CH_LOAD, pulse_us);
}

/* One step of the moving average — O(1) sliding window with modulo wrap. */
static int32_t src_filter_step(int32_t new_sample) {
    s_src_sum -= s_src_buf[s_src_idx];
    s_src_buf[s_src_idx] = new_sample;
    s_src_sum += new_sample;
    s_src_idx = (uint8_t)((s_src_idx + 1) % SRC_FILT_N);
    return (int32_t)(s_src_sum / SRC_FILT_N);
}

/* Reset PID state (called when the loop disengages so re-engage is clean). */
static void pid_reset(void) {
    s_integrator_mA = 0;
    s_last_err_mA   = 0;
}

/* ----------------------------------------------------------------------- */

void throttle_ctrl_task_start(void) {
    /* Park CCR at the ESC idle pulse and start the channel. */
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

    /* ESC arming hold. */
    osDelay(ESC_ARM_HOLD_MS);

    uint32_t next = osKernelGetTickCount();
    for (;;) {
        /* --- Snapshot inputs --- */
        bool    enabled;
        int32_t src_mA, load_mA;
        osMutexAcquire(g_pt_mtx, osWaitForever);
        enabled = g_pt.throttle_ctrl_enabled;
        src_mA  = g_pt.dyno_current_mA;
        load_mA = g_pt.dyno_load_current_mA;
        osMutexRelease(g_pt_mtx);

        /* --- Filter 5FX (always, so window stays current even when idle) --- */
        const int32_t src_filt_mA = src_filter_step(src_mA);
        int32_t err = 0;     /* published below; computed only if PID runs */

        /* --- Manual override (bench-test) bypass --- */
        if (g_throttle_manual_pulse_us != 0) {
            s_pulse = g_throttle_manual_pulse_us;
            pid_reset();
        } else if (!enabled || src_filt_mA > -g_throttle_engage_src_mA) {
            /* Disabled, or motoring / not rectifying enough → idle. */
            s_pulse = ESC_MIN_US;
            pid_reset();
        } else {
            /* --- PID --- */
            err = load_mA - src_filt_mA;

        /* P term */
        const int32_t p_us = (g_throttle_kp_div > 0)
                              ? (err / g_throttle_kp_div)
                              : 0;

        /* I term — anti-windup: only integrate when output isn't pinned. */
        if (g_throttle_ki_div > 0) {
            const bool at_top = (s_pulse >= g_throttle_max_pulse_us);
            const bool at_bot = (s_pulse <= ESC_MIN_US);
            if (!(at_top && err > 0) && !(at_bot && err < 0)) {
                s_integrator_mA += err;
                /* Cap integrator so it can't blow past actuator authority. */
                const int32_t i_cap = g_throttle_ki_div *
                                      (int32_t)(g_throttle_max_pulse_us - ESC_MIN_US);
                if (s_integrator_mA >  i_cap) s_integrator_mA =  i_cap;
                if (s_integrator_mA < -i_cap) s_integrator_mA = -i_cap;
            }
        }
        const int32_t i_us = (g_throttle_ki_div > 0)
                              ? (s_integrator_mA / g_throttle_ki_div)
                              : 0;

        /* D term */
        const int32_t d_us = (g_throttle_kd_div > 0)
                              ? ((err - s_last_err_mA) / g_throttle_kd_div)
                              : 0;
        s_last_err_mA = err;

            /* Combine and clamp into ESC pulse range. */
            int32_t u_above = p_us + i_us + d_us;     /* µs above idle */
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
