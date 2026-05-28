#ifndef POWERTRAIN_STATE_H
#define POWERTRAIN_STATE_H

#include <stdint.h>
#include "cmsis_os2.h"
#include "vesc_proto.h"

/* Hard clamps mirrored on VESC side (hybrid_pcu.c). */
#define I_RECT_MAX_CA           16000      /* 160.00 A */
#define OMEGA_E_MAX_ERPM       100000      /* eRPM */
#define DUTY_MAX_X10000          9500      /* 95.00 % */

/* Engine throttle servo PA8/TIM1_CH1; 1667..2879 us = 0..100 %.
 * Range scaled by 5/3.3 from 1100..1900 for 5 V PWM-as-DAC servo. */
#define ENGINE_THROTTLE_PULSE_MIN_US   1667u
#define ENGINE_THROTTLE_PULSE_MAX_US   2879u

typedef enum {
    FAULT_NONE             = 0u,
    FAULT_RECT_STALE       = 1u <<  0,
    FAULT_RECT_OFFLINE     = 1u <<  1,
    FAULT_FC_STALE         = 1u <<  2,
    FAULT_BUS1_BUSOFF      = 1u <<  3,
    FAULT_BUS2_BUSOFF      = 1u <<  4,
    FAULT_SUPERVISOR_HANG  = 1u <<  5,
    /* bit 6 reserved (was FAULT_BMS_STALE; BMS task disabled). */
    FAULT_ECU_STALE        = 1u <<  7,
} fault_id_t;

/* Engine lifecycle; orthogonal to flight_mode_t. fc_link sets this from an RC
 * channel (or Live Watch during bench). Control law only runs in ENGINE_RUN. */
typedef enum {
    ENGINE_OFF      = 0,
    ENGINE_CRANK    = 1,
    ENGINE_WARMUP   = 2,
    ENGINE_RUN      = 3,
    ENGINE_COOLDOWN = 4,
    ENGINE_FAULT    = 5,
} engine_state_t;

typedef struct {
    /* Setpoint: supervisor -> rectifier_task; rect_ctrl_mode picks the wire frame. */
    rect_ctrl_mode_t  rect_ctrl_mode;
    int16_t           I_rect_cmd_cA;       /* 0.01 A/LSB */
    int32_t           omega_e_cmd_erpm;    /* 1 eRPM/LSB */
    int16_t           duty_cmd_x10000;     /* 0.01 %/LSB */
    flight_mode_t       mode;

    /* Drives 0x104 SendMotorTypeCmd; default FOC for regen. */
    vesc_motor_type_t rect_motor_type;

    /* Drives 0x105 SendInvertDirCmd; compensates BLDC<->FOC direction flip. */
    bool              rect_invert_direction;

    vesc_rect_state_t rect_state;
    uint32_t          rect_state_tick;

    flight_mode_t       fc_flight_state;
    uint16_t          fc_throttle_dem_pct;     /* 0.01 %/LSB */
    uint32_t          fc_input_tick;

    /* ECU mirror; bookkeeping only — CHT/EGT come from MAX31855 (tc_C[]), not ECU. */
    uint16_t          ecu_rpm;
    uint8_t           ecu_engine_status;       /* bit0 ready, 1 crank, 2 startw, 3 warmup */
    uint32_t          ecu_input_tick;

    /* Engine lifecycle; written by fc_link or Live Watch. */
    engine_state_t    engine_state;
    uint32_t          engine_state_tick;

    /* External request; supervisor applies it (CRANK->RUN via swap). */
    engine_state_t    engine_state_req;
    uint32_t          engine_state_req_tick;

    /* V1 controller telemetry (mirrored each tick when engine_state == ENGINE_RUN). */
    int16_t           ctl_i_bat_filt_cA;     /* EMA-filtered ACS ch2 */
    int16_t           ctl_i_bat_ref_eff_cA;  /* setpoint after safety boost */
    int16_t           ctl_i_rect_demand_cA;  /* slew-limited PID output */
    uint32_t          ctl_p_rect_W;          /* I_rect_demand * V_bus */
    uint16_t          ctl_duty_x10000;       /* LUT output */
    uint16_t          ctl_theta_pct_x100;    /* LUT output */

    volatile uint32_t supervisor_heartbeat;

    bool              contactor_battery_cmd;
    bool              contactor_rectifier_cmd;

    uint16_t          fault_bits;

    /* Operator writes req_pct_x100; supervisor pushes to TIM1_CH1 each tick. */
    uint16_t          engine_throttle_req_pct_x100; /* 0..10000 */
    uint16_t          engine_throttle_pct_x100;
    uint16_t          engine_throttle_pulse_us;
    uint32_t          engine_throttle_tick;

    /* ACS772ECB-300B on ADS1262 AIN0/1/2; signed mA. */
    int32_t           current_sensor_mA[3];
    uint32_t          current_sensor_tick;

    /* 3x MAX31855 thermocouples on hspi2, whole C, signed. */
    int16_t           tc_C[3];
    bool              tc_valid[3];
    uint32_t          tc_input_tick;

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

    /* dyno_load_task (bench): PID on ACS ch0 (I_load) -> load ESC PWM. */
    bool              dyno_load_active;
    int32_t           dyno_load_i_target_mA;
    int32_t           dyno_load_i_filt_mA;
    int32_t           dyno_load_err_mA;
    uint16_t          dyno_load_pwm_us;
    uint32_t          dyno_load_tick;
} powertrain_state_t;


extern powertrain_state_t g_pt;
extern osMutexId_t        g_pt_mtx;

/* Call once before osKernelStart. */
void     pt_init(void);

/* Setters: atomically set value + switch rect_ctrl_mode to matching type. */
void     pt_set_setpoint      (int16_t I_rect_cmd_cA,    flight_mode_t mode);
void     pt_set_setpoint_omega(int32_t omega_e_cmd_erpm, flight_mode_t mode);
void     pt_set_setpoint_duty (int16_t duty_cmd_x10000,  flight_mode_t mode);

void     pt_set_motor_type    (vesc_motor_type_t type);
void     pt_set_invert_direction(bool invert);

/* 0..10000 (0.01 %/LSB); writes TIM1_CH1 CCR. */
void     pt_set_engine_throttle_pct_x100(uint16_t pct_x100);

void     pt_set_fault(uint16_t mask);
void     pt_clear_fault(uint16_t mask);
uint16_t pt_get_faults(void);

void pt_set_fc_inputs(flight_mode_t flight_state, uint16_t throttle_dem_pct);
void pt_set_engine_state(engine_state_t s);
void pt_request_engine_state(engine_state_t s);

/* Pre-power gate: VESC + FC alive, no critical faults. CRANK/RUN rejected if false. */
bool pt_preflight_ok(void);

/* Sentinel for engine_state_req meaning "no pending request". */
#define ENGINE_STATE_REQ_NONE ((engine_state_t)0xFF)

void pt_set_contactor_cmds(bool battery_closed, bool rectifier_closed);


#endif /* POWERTRAIN_STATE_H */
