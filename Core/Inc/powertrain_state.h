#ifndef POWERTRAIN_STATE_H
#define POWERTRAIN_STATE_H

#include <stdint.h>
#include "cmsis_os2.h"
#include "vesc_proto.h"

/* Hard clamps mirrored on VESC side (hybrid_pcu.c). */
#define I_RECT_MAX_CA           16000      /* 160.00 A */
#define OMEGA_E_MAX_ERPM       100000      /* eRPM */
#define DUTY_MAX_X10000          9500      /* 95.00 % */

/* Engine throttle servo PA8/TIM1_CH1; 1100..1900 us = 0..100 %. */
#define ENGINE_THROTTLE_PULSE_MIN_US   1100u
#define ENGINE_THROTTLE_PULSE_MAX_US   1900u

typedef enum {
    FAULT_NONE             = 0u,
    FAULT_RECT_STALE       = 1u <<  0,
    FAULT_RECT_OFFLINE     = 1u <<  1,
    FAULT_FC_STALE         = 1u <<  2,
    FAULT_BUS1_BUSOFF      = 1u <<  3,
    FAULT_BUS2_BUSOFF      = 1u <<  4,
    FAULT_SUPERVISOR_HANG  = 1u <<  5,
    FAULT_BMS_STALE        = 1u <<  6,
    FAULT_ECU_STALE        = 1u <<  7,
} fault_id_t;

typedef struct {
    /* Setpoint: supervisor -> rectifier_task; rect_ctrl_mode picks the wire frame. */
    rect_ctrl_mode_t  rect_ctrl_mode;
    int16_t           I_rect_cmd_cA;       /* 0.01 A/LSB */
    int32_t           omega_e_cmd_erpm;    /* 1 eRPM/LSB */
    int16_t           duty_cmd_x10000;     /* 0.01 %/LSB */
    vesc_mode_t       mode;

    /* Drives 0x104 SendMotorTypeCmd; default FOC for regen. */
    vesc_motor_type_t rect_motor_type;

    /* Drives 0x105 SendInvertDirCmd; compensates BLDC<->FOC direction flip. */
    bool              rect_invert_direction;

    vesc_rect_state_t rect_state;
    uint32_t          rect_state_tick;

    vesc_mode_t       fc_flight_state;
    uint16_t          fc_throttle_dem_pct;     /* 0.01 %/LSB */
    uint32_t          fc_input_tick;

    uint16_t          bms_soc_pct;             /* 0.01 %/LSB */
    uint16_t          bms_v_bat_cV;            /* 0.01 V/LSB */
    int16_t           bms_i_bat_cA;            /* 0.01 A/LSB, +discharge */
    int8_t            bms_max_cell_C;          /* hottest cell C */
    uint32_t          bms_input_tick;

    /* ECU mirror; all temps are whole C (F->C done in ecu_task). */
    uint16_t          ecu_rpm;
    uint16_t          ecu_inj_pw_us;           /* MS2 pulseWidth1 */
    uint16_t          ecu_fuel_rate_dg_s;      /* 0.1 g/s; TODO: derive from PW*RPM */
    int16_t           ecu_cht_C;
    int16_t           ecu_iat_C;
    int16_t           ecu_egt_C;               /* MS2 adc6 x 1.222 */
    int16_t           ecu_tps_pct_x10;         /* 0.1 %/LSB */
    int16_t           ecu_map_kPa_x10;         /* 0.1 kPa/LSB */
    int16_t           ecu_batt_cV;             /* 0.01 V/LSB */
    uint8_t           ecu_engine_status;       /* bit0 ready, 1 crank, 2 startw, 3 warmup */
    uint8_t           ecu_sync_err_cnt;
    uint32_t          ecu_input_tick;

    volatile uint32_t supervisor_heartbeat;

    bool              contactor_battery_cmd;
    bool              contactor_rectifier_cmd;

    uint16_t          fault_bits;

    /* SSD-250A 5FX-3 source sensor on CAN1, native SSD LSBs. */
    int32_t           dyno_current_mA;       /* 1 mA/LSB, signed */
    int32_t           dyno_vbus_mV;
    uint32_t          dyno_power_dW;         /* 0.1 W/LSB */
    uint64_t          dyno_energy_Wh;
    uint32_t          dyno_input_tick;

    /* SSD-250A 4FX-2 load sensor; PI feedback against 5FX. */
    int32_t           dyno_load_current_mA;
    int32_t           dyno_load_vbus_mV;
    uint32_t          dyno_load_power_dW;
    uint64_t          dyno_load_energy_Wh;
    uint32_t          dyno_load_input_tick;

    /* Throttle PI: 4FX -> 5FX via PWM on PA10/TIM1_CH3. */
    bool              throttle_ctrl_enabled;
    uint16_t          throttle_pwm_duty;
    int32_t           throttle_current_err_mA;
    int32_t           throttle_src_filt_mA;
    uint32_t          throttle_ctrl_tick;

    /* Operator writes req_pct_x100; supervisor pushes to TIM1_CH1 each tick. */
    uint16_t          engine_throttle_req_pct_x100; /* 0..10000 */
    uint16_t          engine_throttle_pct_x100;
    uint16_t          engine_throttle_pulse_us;
    uint32_t          engine_throttle_tick;

    /* ACS772ECB-300B on ADS1262 AIN0/1/2; signed mA. */
    int32_t           current_sensor_mA[3];
    uint32_t          current_sensor_tick;

    /* From rtos_stats_task; idle-counter derived. */
    uint8_t           cpu_load_pct;
    uint32_t          cpu_load_tick;

    /* Experiment runner state; label points into static profile data. */
    bool              expt_active;
    uint8_t           expt_phase_idx;
    const char       *expt_label;
    bool              expt_advance_req;
    bool              expt_abort_req;
    bool              expt_start_req;
} powertrain_state_t;


extern powertrain_state_t g_pt;
extern osMutexId_t        g_pt_mtx;

/* Call once before osKernelStart. */
void     pt_init(void);

/* Setters: atomically set value + switch rect_ctrl_mode to matching type. */
void     pt_set_setpoint      (int16_t I_rect_cmd_cA,    vesc_mode_t mode);
void     pt_set_setpoint_omega(int32_t omega_e_cmd_erpm, vesc_mode_t mode);
void     pt_set_setpoint_duty (int16_t duty_cmd_x10000,  vesc_mode_t mode);

void     pt_set_motor_type    (vesc_motor_type_t type);
void     pt_set_invert_direction(bool invert);

/* 0..10000 (0.01 %/LSB); writes TIM1_CH1 CCR. */
void     pt_set_engine_throttle_pct_x100(uint16_t pct_x100);

void     pt_set_fault(uint16_t mask);
void     pt_clear_fault(uint16_t mask);
uint16_t pt_get_faults(void);

void pt_set_fc_inputs(vesc_mode_t flight_state, uint16_t throttle_dem_pct);
void pt_set_bms_inputs(uint16_t soc_pct);

void pt_set_contactor_cmds(bool battery_closed, bool rectifier_closed);


#endif /* POWERTRAIN_STATE_H */
