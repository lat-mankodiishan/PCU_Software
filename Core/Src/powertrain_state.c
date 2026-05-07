#include "powertrain_state.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include <string.h>

powertrain_state_t g_pt;
osMutexId_t        g_pt_mtx;

static StaticSemaphore_t s_pt_mtx_cb;

void pt_init(void) {
    memset(&g_pt, 0, sizeof(g_pt));
    g_pt.mode            = VESC_MODE_IDLE;
    g_pt.rect_motor_type = VESC_MOTOR_TYPE_FOC;     /* steady-state default */

    static const osMutexAttr_t attr = {
        .name      = "g_pt_mtx",
        .attr_bits = osMutexPrioInherit,
        .cb_mem    = &s_pt_mtx_cb,
        .cb_size   = sizeof(s_pt_mtx_cb),
    };
    g_pt_mtx = osMutexNew(&attr);
}

void pt_set_setpoint(int16_t I_rect_cmd_cA, vesc_mode_t mode) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.rect_ctrl_mode = RECT_CTRL_CURRENT;
    g_pt.I_rect_cmd_cA  = I_rect_cmd_cA;
    g_pt.mode           = mode;
    osMutexRelease(g_pt_mtx);
}

void pt_set_setpoint_omega(int32_t omega_e_cmd_erpm, vesc_mode_t mode) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.rect_ctrl_mode    = RECT_CTRL_OMEGA;
    g_pt.omega_e_cmd_erpm  = omega_e_cmd_erpm;
    g_pt.mode              = mode;
    osMutexRelease(g_pt_mtx);
}

void pt_set_setpoint_duty(int16_t duty_cmd_x10000, vesc_mode_t mode) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.rect_ctrl_mode    = RECT_CTRL_DUTY;
    g_pt.duty_cmd_x10000   = duty_cmd_x10000;
    g_pt.mode              = mode;
    osMutexRelease(g_pt_mtx);
}

void pt_set_motor_type(vesc_motor_type_t type) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.rect_motor_type = type;
    osMutexRelease(g_pt_mtx);
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

void pt_set_fc_inputs(vesc_mode_t flight_state, uint16_t throttle_dem_pct) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.fc_flight_state     = flight_state;
    g_pt.fc_throttle_dem_pct = throttle_dem_pct;
    g_pt.fc_input_tick       = osKernelGetTickCount();
    osMutexRelease(g_pt_mtx);
}

void pt_set_bms_inputs(uint16_t soc_pct) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.bms_soc_pct     = soc_pct;
    g_pt.bms_input_tick  = osKernelGetTickCount();
    osMutexRelease(g_pt_mtx);
}

void pt_set_contactor_cmds(bool battery_closed, bool rectifier_closed) {
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.contactor_battery_cmd    = battery_closed;
    g_pt.contactor_rectifier_cmd  = rectifier_closed;
    osMutexRelease(g_pt_mtx);
}