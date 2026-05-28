#include "powertrain_state.h"
#include "periph_wrappers.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

powertrain_state_t g_pt;
osMutexId_t        g_pt_mtx;

static StaticSemaphore_t s_pt_mtx_cb;

void pt_init(void) {
    memset(&g_pt, 0, sizeof(g_pt));
    g_pt.rect_cmd.mode              = MODE_IDLE;
    g_pt.engine.state               = ENGINE_OFF;
    g_pt.engine.req                 = ENGINE_STATE_REQ_NONE;
    g_pt.rect_cmd.motor_type        = VESC_MOTOR_TYPE_FOC;
    g_pt.engine_throttle.pulse_us   = ENGINE_THROTTLE_PULSE_MIN_US;

    static const osMutexAttr_t attr = {
        .name      = "g_pt_mtx",
        .attr_bits = osMutexPrioInherit,
        .cb_mem    = &s_pt_mtx_cb,
        .cb_size   = sizeof(s_pt_mtx_cb),
    };
    g_pt_mtx = osMutexNew(&attr);

    /* Park engine throttle ESC at min pulse before any setter runs. */
    esc_hw_init(ESC_CH_ENGINE, ENGINE_THROTTLE_PULSE_MIN_US);
}

void pt_set_setpoint(int16_t I_rect_cmd_cA, flight_mode_t mode) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.rect_cmd.ctrl_mode = RECT_CTRL_CURRENT;
    g_pt.rect_cmd.I_cmd_cA  = I_rect_cmd_cA;
    g_pt.rect_cmd.mode      = mode;
    osMutexRelease(g_pt_mtx);
}

void pt_set_setpoint_omega(int32_t omega_e_cmd_erpm, flight_mode_t mode) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.rect_cmd.ctrl_mode        = RECT_CTRL_OMEGA;
    g_pt.rect_cmd.omega_e_cmd_erpm = omega_e_cmd_erpm;
    g_pt.rect_cmd.mode             = mode;
    osMutexRelease(g_pt_mtx);
}

void pt_set_setpoint_duty(int16_t duty_cmd_x10000, flight_mode_t mode) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.rect_cmd.ctrl_mode       = RECT_CTRL_DUTY;
    g_pt.rect_cmd.duty_cmd_x10000 = duty_cmd_x10000;
    g_pt.rect_cmd.mode            = mode;
    osMutexRelease(g_pt_mtx);
}

void pt_set_motor_type(vesc_motor_type_t type) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.rect_cmd.motor_type = type;
    osMutexRelease(g_pt_mtx);
}

void pt_set_invert_direction(bool invert) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.rect_cmd.invert_direction = invert;
    osMutexRelease(g_pt_mtx);
}

void pt_set_engine_throttle_pct_x100(uint16_t pct_x100) {
    if (pct_x100 > 10000u) pct_x100 = 10000u;

    const uint16_t span_us  = ENGINE_THROTTLE_PULSE_MAX_US - ENGINE_THROTTLE_PULSE_MIN_US;
    const uint16_t pulse_us = (uint16_t)(ENGINE_THROTTLE_PULSE_MIN_US
                              + ((uint32_t)span_us * pct_x100) / 10000u);

    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.engine_throttle.applied_pct_x100 = pct_x100;
    g_pt.engine_throttle.pulse_us         = pulse_us;
    g_pt.engine_throttle.tick             = osKernelGetTickCount();
    osMutexRelease(g_pt_mtx);

    esc_hw_set_us(ESC_CH_ENGINE, pulse_us);
}

void pt_set_fault(uint16_t mask) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.fault_bits |= mask;
    osMutexRelease(g_pt_mtx);
}

void pt_clear_fault(uint16_t mask) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.fault_bits &= ~mask;
    osMutexRelease(g_pt_mtx);
}

uint16_t pt_get_faults(void) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    uint16_t v = g_pt.fault_bits;
    osMutexRelease(g_pt_mtx);
    return v;
}

void pt_set_fc_inputs(flight_mode_t flight_state, uint16_t throttle_dem_pct) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.fc.flight_state     = flight_state;
    g_pt.fc.throttle_dem_pct = throttle_dem_pct;
    g_pt.fc.tick             = osKernelGetTickCount();
    osMutexRelease(g_pt_mtx);
}

void pt_set_engine_state(engine_state_t s) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.engine.state      = s;
    g_pt.engine.state_tick = osKernelGetTickCount();
    osMutexRelease(g_pt_mtx);
}

/* Diagnostic: how many CRANK/RUN requests were rejected by the pre-flight gate. */
volatile uint32_t g_pt_preflight_reject = 0u;

#define PT_PREFLIGHT_RECT_STALE_MS   500u
#define PT_PREFLIGHT_FC_STALE_MS     500u
#define PT_PREFLIGHT_CRITICAL_FAULTS (FAULT_RECT_OFFLINE | FAULT_BUS1_BUSOFF | FAULT_BUS2_BUSOFF)

bool pt_preflight_ok(void) {
    uint32_t now = osKernelGetTickCount();
    osMutexAcquire(g_pt_mtx, osWaitForever);
    bool rect_ok = g_pt.rect.tick && (now - g_pt.rect.tick) < PT_PREFLIGHT_RECT_STALE_MS;
    bool fc_ok   = g_pt.fc.tick   && (now - g_pt.fc.tick)   < PT_PREFLIGHT_FC_STALE_MS;
    uint16_t f   = g_pt.fault_bits;
    osMutexRelease(g_pt_mtx);
    return rect_ok && fc_ok && ((f & PT_PREFLIGHT_CRITICAL_FAULTS) == 0u);
}

void pt_request_engine_state(engine_state_t s) {
    if ((s == ENGINE_CRANK || s == ENGINE_RUN) && !pt_preflight_ok()) {
        g_pt_preflight_reject++;
        return;
    }
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.engine.req      = s;
    g_pt.engine.req_tick = osKernelGetTickCount();
    osMutexRelease(g_pt_mtx);
}

void pt_set_contactor_cmds(bool battery_closed, bool rectifier_closed) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.contactor_cmd.battery   = battery_closed;
    g_pt.contactor_cmd.rectifier = rectifier_closed;
    osMutexRelease(g_pt_mtx);
}
