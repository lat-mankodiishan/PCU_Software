#include "experiment.h"
#include "powertrain_state.h"
#include "cmsis_os2.h"
#include "main.h"
#include "stm32f4xx_hal.h"

#define EXPT_TICK_MS  10        /* 100 Hz */

/* ---- g_pt accessors ---- */

static void mark_state(bool active, uint8_t phase_idx, const char *label) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.expt.active    = active;
    g_pt.expt.phase_idx = phase_idx;
    g_pt.expt.label     = label;
    osMutexRelease(g_pt_mtx);
}

static bool consume_advance(void) {
    bool v;
    osMutexAcquire(g_pt_mtx, osWaitForever);
    v = g_pt.expt.advance_req;
    if (v) g_pt.expt.advance_req = false;
    osMutexRelease(g_pt_mtx);
    return v;
}

static bool peek_abort(void) {
    bool v;
    osMutexAcquire(g_pt_mtx, osWaitForever);
    v = g_pt.expt.abort_req;
    osMutexRelease(g_pt_mtx);
    return v;
}

static void clear_flags(void) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.expt.advance_req = false;
    g_pt.expt.abort_req   = false;
    osMutexRelease(g_pt_mtx);
}

static int32_t prev_setpoint(rect_ctrl_mode_t mode) {
    int32_t v = 0;
    osMutexAcquire(g_pt_mtx, osWaitForever);
    switch (mode) {
    case RECT_CTRL_CURRENT: v = (int32_t)g_pt.rect_cmd.I_cmd_cA;         break;
    case RECT_CTRL_OMEGA:   v = g_pt.rect_cmd.omega_e_cmd_erpm;          break;
    case RECT_CTRL_DUTY:    v = (int32_t)g_pt.rect_cmd.duty_cmd_x10000;  break;
    default: break;
    }
    osMutexRelease(g_pt_mtx);
    return v;
}

static void publish(rect_ctrl_mode_t mode, int32_t value, flight_mode_t pt_mode) {
    switch (mode) {
    case RECT_CTRL_CURRENT: pt_set_setpoint      ((int16_t)value, pt_mode); break;
    case RECT_CTRL_OMEGA:   pt_set_setpoint_omega(value,          pt_mode); break;
    case RECT_CTRL_DUTY:    pt_set_setpoint_duty ((int16_t)value, pt_mode); break;
    default: break;
    }
}

/* ---- Phase + profile execution ---- */

static void run_phase(const expt_phase_t *p, uint8_t idx) {
    mark_state(true, idx, p->label);

    /* Motor-type change before setpoints; wait covers async VESC reconfig. */
    switch (p->motor_type) {
    case EXPT_MOTOR_TYPE_BLDC: pt_set_motor_type(VESC_MOTOR_TYPE_BLDC); osDelay(20); break;
    case EXPT_MOTOR_TYPE_DC:   pt_set_motor_type(VESC_MOTOR_TYPE_DC);   osDelay(20); break;
    case EXPT_MOTOR_TYPE_FOC:  pt_set_motor_type(VESC_MOTOR_TYPE_FOC);  osDelay(20); break;
    case EXPT_MOTOR_TYPE_KEEP:
    default: break;
    }

    /* Invert-direction change; bundled with motor_type. */
    switch (p->invert_dir) {
    case EXPT_INVERT_DIR_NORMAL:   pt_set_invert_direction(false); osDelay(20); break;
    case EXPT_INVERT_DIR_INVERTED: pt_set_invert_direction(true);  osDelay(20); break;
    case EXPT_INVERT_DIR_KEEP:
    default: break;
    }

    const int32_t prev = prev_setpoint(p->ctrl_mode);

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

    switch (p->hold_type) {
    case EXPT_HOLD_TIME: {
        const uint32_t t0 = osKernelGetTickCount();
        while ((osKernelGetTickCount() - t0) < p->hold_ms && !peek_abort()) {
            osDelay(EXPT_TICK_MS);
        }
        break;
    }
    case EXPT_HOLD_OP:
        (void)consume_advance();           /* drop stale flag */
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

    pt_set_setpoint(0, MODE_IDLE);
    mark_state(false, 0, NULL);
}

void expt_advance(void) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.expt.advance_req = true;
    osMutexRelease(g_pt_mtx);
}

void expt_abort(void) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.expt.abort_req = true;
    osMutexRelease(g_pt_mtx);
}
