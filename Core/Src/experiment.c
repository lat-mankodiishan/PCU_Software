#include "experiment.h"
#include "powertrain_state.h"
#include "cmsis_os2.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#define EXPT_TICK_MS  10        /* 100 Hz update during ramps and waits */

/* Aux GPIO controlled per-phase via expt_phase_t.aux_gpio. Configure
 * this pin as GPIO_Output in CubeMX (.ioc). If you change the port,
 * adjust both the port macro and ensure CubeMX has the new pin set up. */
#define EXPT_AUX_GPIO_PORT   GPIOA
#define EXPT_AUX_GPIO_PIN    GPIO_PIN_3

/* ---- g_pt accessors ---------------------------------------------------- */

static void mark_state(bool active, uint8_t phase_idx, const char *label) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.expt_active    = active;
    g_pt.expt_phase_idx = phase_idx;
    g_pt.expt_label     = label;
    osMutexRelease(g_pt_mtx);
}

static bool consume_advance(void) {
    bool v;
    osMutexAcquire(g_pt_mtx, osWaitForever);
    v = g_pt.expt_advance_req;
    if (v) g_pt.expt_advance_req = false;
    osMutexRelease(g_pt_mtx);
    return v;
}

static bool peek_abort(void) {
    bool v;
    osMutexAcquire(g_pt_mtx, osWaitForever);
    v = g_pt.expt_abort_req;
    osMutexRelease(g_pt_mtx);
    return v;
}

static void clear_flags(void) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.expt_advance_req = false;
    g_pt.expt_abort_req   = false;
    osMutexRelease(g_pt_mtx);
}

static int32_t prev_setpoint(rect_ctrl_mode_t mode) {
    int32_t v = 0;
    osMutexAcquire(g_pt_mtx, osWaitForever);
    switch (mode) {
    case RECT_CTRL_CURRENT: v = (int32_t)g_pt.I_rect_cmd_cA;    break;
    case RECT_CTRL_OMEGA:   v = g_pt.omega_e_cmd_erpm;          break;
    case RECT_CTRL_DUTY:    v = (int32_t)g_pt.duty_cmd_x10000;  break;
    default: break;
    }
    osMutexRelease(g_pt_mtx);
    return v;
}

static void publish(rect_ctrl_mode_t mode, int32_t value, vesc_mode_t pt_mode) {
    switch (mode) {
    case RECT_CTRL_CURRENT: pt_set_setpoint      ((int16_t)value, pt_mode); break;
    case RECT_CTRL_OMEGA:   pt_set_setpoint_omega(value,          pt_mode); break;
    case RECT_CTRL_DUTY:    pt_set_setpoint_duty ((int16_t)value, pt_mode); break;
    default: break;
    }
}

/* ---- Phase + profile execution ----------------------------------------- */

static void run_phase(const expt_phase_t *p, uint8_t idx) {
    mark_state(true, idx, p->label);

    /* Motor-type change (if requested) must happen BEFORE we start
     * publishing setpoints, so the VESC has a chance to reconfigure
     * before the first ramp value lands. The actual reconfig is async
     * (PCU TX → CAN → VESC RX → set_configuration), so we wait briefly
     * for the round-trip. PCU TXes 0x104 every 5 ms tick; 50 ms is
     * comfortably more than the worst-case re-init latency. */
    switch (p->motor_type) {
    case EXPT_MOTOR_TYPE_BLDC: pt_set_motor_type(VESC_MOTOR_TYPE_BLDC); osDelay(20); break;
    case EXPT_MOTOR_TYPE_DC:   pt_set_motor_type(VESC_MOTOR_TYPE_DC);   osDelay(20); break;
    case EXPT_MOTOR_TYPE_FOC:  pt_set_motor_type(VESC_MOTOR_TYPE_FOC);  osDelay(20); break;
    case EXPT_MOTOR_TYPE_KEEP:
    default: break;
    }

    /* Invert-direction change — bundled with motor_type for the BLDC<->FOC
     * direction-convention hand-off. Same reconfig wait pattern. */
    switch (p->invert_dir) {
    case EXPT_INVERT_DIR_NORMAL:   pt_set_invert_direction(false); osDelay(20); break;
    case EXPT_INVERT_DIR_INVERTED: pt_set_invert_direction(true);  osDelay(20); break;
    case EXPT_INVERT_DIR_KEEP:
    default: break;
    }

    /* Aux GPIO drive — single-write, no settle needed. */
    switch (p->aux_gpio) {
    case EXPT_AUX_GPIO_HIGH:
        HAL_GPIO_WritePin(EXPT_AUX_GPIO_PORT, EXPT_AUX_GPIO_PIN, GPIO_PIN_SET);
        break;
    case EXPT_AUX_GPIO_LOW:
        HAL_GPIO_WritePin(EXPT_AUX_GPIO_PORT, EXPT_AUX_GPIO_PIN, GPIO_PIN_RESET);
        break;
    case EXPT_AUX_GPIO_KEEP:
    default: break;
    }

    const int32_t prev = prev_setpoint(p->ctrl_mode);

    /* Ramp from prev to p->setpoint over p->ramp_ms (or step if 0). */
    if (p->ramp_ms > 0) {
        const uint32_t t0 = osKernelGetTickCount();
        uint32_t t = 0;
        while (t < p->ramp_ms && !peek_abort()) {
            const int32_t v = prev +
                (int32_t)(((int64_t)(p->setpoint - prev) * (int32_t)t)
                          / (int32_t)p->ramp_ms);
            publish(p->ctrl_mode, v, p->pt_mode);
            osDelay(EXPT_TICK_MS);
            t = osKernelGetTickCount() - t0;
        }
    }
    if (peek_abort()) return;
    publish(p->ctrl_mode, p->setpoint, p->pt_mode);

    /* Hold at the setpoint until trigger fires. */
    switch (p->hold_type) {
    case EXPT_HOLD_TIME: {
        const uint32_t t0 = osKernelGetTickCount();
        while ((osKernelGetTickCount() - t0) < p->hold_ms && !peek_abort()) {
            osDelay(EXPT_TICK_MS);
        }
        break;
    }
    case EXPT_HOLD_OP:
        (void)consume_advance();           /* drop any stale flag */
        for (;;) {
            if (peek_abort())       break;
            if (consume_advance())  break;
            osDelay(EXPT_TICK_MS);
        }
        break;
    case EXPT_HOLD_COND: {
        powertrain_state_t snap;
        for (;;) {
            if (peek_abort()) break;
            osMutexAcquire(g_pt_mtx, osWaitForever);
            snap = g_pt;
            osMutexRelease(g_pt_mtx);
            if (p->hold_cond && p->hold_cond(&snap)) break;
            osDelay(EXPT_TICK_MS);
        }
        break;
    }
    default: break;
    }
}

void expt_run(const expt_profile_t *profile) {
    if (!profile || !profile->phases || profile->n_phases == 0) return;
    clear_flags();

    do {
        for (uint8_t i = 0; i < profile->n_phases && !peek_abort(); ++i) {
            run_phase(&profile->phases[i], i);
        }
    } while (profile->loop && !peek_abort());

    /* Safe idle on completion / abort. Also drive aux GPIO LOW so any
     * external "test active" indicator clears. */
    pt_set_setpoint(0, VESC_MODE_IDLE);
    HAL_GPIO_WritePin(EXPT_AUX_GPIO_PORT, EXPT_AUX_GPIO_PIN, GPIO_PIN_RESET);
    mark_state(false, 0, NULL);
}

void expt_advance(void) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.expt_advance_req = true;
    osMutexRelease(g_pt_mtx);
}

void expt_abort(void) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.expt_abort_req = true;
    osMutexRelease(g_pt_mtx);
}
