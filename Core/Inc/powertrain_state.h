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
#define ENGINE_THROTTLE_PULSE_MIN_US   1000u
#define ENGINE_THROTTLE_PULSE_MAX_US   2000u

typedef enum {
    FAULT_NONE                 = 0u,
    FAULT_RECT_STALE           = 1u <<  0,   /* DEGRADE — VESC quiet > 200 ms */
    FAULT_RECT_OFFLINE         = 1u <<  1,   /* FORCE_FAULT — VESC absent > 2 s */
    FAULT_FC_STALE             = 1u <<  2,   /* FORCE_OFF — FC NodeStatus > 1 s */
    FAULT_BUS1_BUSOFF          = 1u <<  3,   /* FORCE_FAULT — CAN1 in bus-off */
    FAULT_BUS2_BUSOFF          = 1u <<  4,   /* FORCE_FAULT — CAN2 in bus-off */
    FAULT_SUPERVISOR_HANG      = 1u <<  5,   /* FORCE_FAULT — heartbeat > 1 s */
    /* bit 6 reserved (was FAULT_BMS_STALE; BMS task disabled). */
    FAULT_ECU_STALE            = 1u <<  7,   /* WARN — ECU poll > 1 s */
    FAULT_FC_THROTTLE_STALE    = 1u <<  8,   /* FORCE_OFF — RawCommand > 500 ms */
    FAULT_CURRENT_SENSOR_STALE = 1u <<  9,   /* FORCE_OFF — ADS1262 > 500 ms */
    FAULT_TC_FAULT             = 1u << 10,   /* WARN — any tc_valid[]==false */
    FAULT_ENGINE_STALL         = 1u << 11,   /* FORCE_OFF — rpm low in RUN > 2 s */
    FAULT_ENGINE_OVERTEMP      = 1u << 12,   /* FORCE_OFF — cyl TC > 220 °C */
    FAULT_RECT_OVERTEMP        = 1u << 13,   /* FORCE_OFF — IGBT > 85 °C */
    FAULT_BUS_UNDERVOLT        = 1u << 14,   /* FORCE_OFF — V_dc < 40 V */
    FAULT_BUS_OVERVOLT         = 1u << 15,   /* FORCE_FAULT — V_dc > 60 V */
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

/* === Sub-struct definitions ===
 *
 * Each sub-struct groups fields with a single producer and a small set of
 * consumers. See the producer/consumer map in the cleanup plan for who
 * writes/reads what. All access goes through pt_* helpers in
 * powertrain_state.c; this file's sub-struct types let those helpers take
 * typed pointers instead of individual field arguments.
 */

/* Rectifier setpoint: supervisor / experiment.c -> rectifier_task.
 * ctrl_mode picks which one of I_cmd_cA / omega_e_cmd_erpm / duty_cmd_x10000
 * the rectifier task TXes on the wire. */
typedef struct {
    rect_ctrl_mode_t  ctrl_mode;
    int16_t           I_cmd_cA;            /* 0.01 A/LSB,  RECT_CTRL_CURRENT */
    int32_t           omega_e_cmd_erpm;    /* 1 eRPM/LSB,  RECT_CTRL_OMEGA   */
    int16_t           duty_cmd_x10000;     /* 0.01 %/LSB,  RECT_CTRL_DUTY    */
    flight_mode_t     mode;
    vesc_motor_type_t motor_type;          /* drives 0x104 SendMotorTypeCmd */
    bool              invert_direction;    /* drives 0x105 SendInvertDirCmd */
} pt_rect_cmd_t;

/* Rectifier telemetry: rectifier_task -> readers.
 *   state / tick         : decoded 0x201 GetRectStateConcise (V/I/RPM/IGBT/fault).
 *   state_ext / ext_tick : decoded 0x202 GetRectStateExtended (duty/Iq/Id/state).
 * ext_tick stays 0 until the matching VESC-side encoder is deployed. */
typedef struct {
    vesc_rect_state_t     state;
    uint32_t              tick;
    vesc_rect_state_ext_t state_ext;
    uint32_t              ext_tick;
} pt_rect_telem_t;

/* Flight-controller inputs: fc_link_task -> supervisor / log. */
typedef struct {
    flight_mode_t  flight_state;
    uint16_t       throttle_dem_pct;       /* 0.01 %/LSB */
    uint32_t       tick;
} pt_fc_t;

/* ECU mirror: ecu_task (Loweheiser UART2 DMA) -> log / fc_link.
 * Primary CHT/EGT come from MAX31855 (pt_tc_t); ECU fields are bookkeeping. */
typedef struct {
    uint16_t  rpm;
    uint8_t   engine_status;               /* bit0 ready, 1 crank, 2 startw, 3 warmup */
    uint16_t  seconds;                     /* since-boot counter, wraps at ~18 h */
    int16_t   coolant_C_x10;               /* coolant/CHT, 0.1 °C */
    int16_t   tps_pct_x10;                 /* throttle position, 0.1 % */
    int16_t   afr1_x10;                    /* AFR cyl 1, 0.1 AFR */
    int16_t   afr2_x10;                    /* AFR cyl 2, 0.1 AFR */
    uint16_t  pwmin0;                      /* raw PWM-in ch 0 (timer counts) */
    uint16_t  pwmin1;
    uint16_t  pwmin2;
    uint16_t  pwmin3;
    uint8_t   rc_throttle;                 /* clamped 0..100 %, derived from tps */
    uint32_t  tick;
} pt_ecu_t;

/* Engine lifecycle:
 *   state / state_tick : committed by supervisor via pt_set_engine_state.
 *   req   / req_tick   : set by fc_link (RC) or operator (Live Watch) via
 *                        pt_request_engine_state; supervisor consumes and
 *                        clears to ENGINE_STATE_REQ_NONE. */
typedef struct {
    engine_state_t  state;
    uint32_t        state_tick;
    engine_state_t  req;
    uint32_t        req_tick;
} pt_engine_state_t;

/* Engine throttle servo (PA8 TIM1_CH1, 1667..2879 us = 0..100 %).
 *   req_pct_x100     : supervisor V1 output OR operator override (Live Watch).
 *   applied_pct_x100 : mirror of last value pushed through
 *                      pt_set_engine_throttle_pct_x100 (which also writes
 *                      the PWM hardware).
 *   pulse_us         : computed from applied_pct_x100. */
typedef struct {
    uint16_t  req_pct_x100;                /* 0..10000 */
    uint16_t  applied_pct_x100;            /* 0..10000 */
    uint16_t  pulse_us;
    uint32_t  tick;
} pt_engine_throttle_t;

/* V1 controller telemetry mirror; supervisor writes each tick in ENGINE_RUN.
 * Read by log_task and the debugger for inspection — not in any control path. */
typedef struct {
    int16_t   i_bat_filt_cA;               /* EMA-filtered ACS ch2 */
    int16_t   i_bat_ref_eff_cA;            /* setpoint after safety boost */
    int16_t   i_rect_demand_cA;            /* slew-limited PID output */
    uint32_t  p_rect_W;                    /* I_rect_demand * V_bus */
    uint16_t  duty_x10000;                 /* LUT output */
    uint16_t  theta_pct_x100;              /* LUT output */
} pt_ctl_telem_t;

/* ACS772ECB-300B current sensors on ADS1262 AIN0/1/2; signed mA.
 * Written by sensor_task; ch2 feeds the V1 control law's I_bat input;
 * ch[DYNO_LOAD_ACS_CH] feeds dyno_load_task. */
typedef struct {
    int32_t   mA[3];
    uint32_t  tick;
} pt_acs_t;

/* 3x MAX31855 K-type thermocouples on hspi2; whole °C, signed. */
typedef struct {
    int16_t   C[3];
    bool      valid[3];
    uint32_t  tick;
} pt_tc_t;

/* CPU load from rtos_stats (idle-counter derived). */
typedef struct {
    uint8_t   load_pct;
    uint32_t  tick;
} pt_cpu_t;

/* Experiment runner state. label points into static profile data, so it
 * lives as long as the firmware does. The three _req flags are operator
 * hooks (Live Watch) — also poked by dyno_sweep_task which uses them as
 * its own state-machine signals. */
typedef struct {
    bool         active;
    uint8_t      phase_idx;
    const char  *label;
    bool         advance_req;
    bool         abort_req;
    bool         start_req;
} pt_expt_t;

/* dyno_load_task: PID on ACS ch0 (I_load) driving a load-ESC PWM.
 *   active / i_target_mA  : operator inputs (Live Watch or dyno_sweep_task).
 *   i_filt_mA / err_mA / pwm_us / tick : per-tick mirror by dyno_load_task. */
typedef struct {
    bool      active;
    int32_t   i_target_mA;
    int32_t   i_filt_mA;
    int32_t   err_mA;
    uint16_t  pwm_us;
    uint32_t  tick;
} pt_dyno_load_t;

/* Contactor commands: supervisor sets, pdb_task mirrors to GPIOs. */
typedef struct {
    bool  battery;
    bool  rectifier;
} pt_contactor_cmd_t;

/* === Aggregate ===
 *
 * Top-level state. Locked by g_pt_mtx (priority-inherited, static-allocated;
 * see pt_init). All cross-task access must go through pt_* helpers. */
typedef struct {
    pt_rect_cmd_t          rect_cmd;
    pt_rect_telem_t        rect;
    pt_fc_t                fc;
    pt_ecu_t               ecu;
    pt_engine_state_t      engine;
    pt_engine_throttle_t   engine_throttle;
    pt_ctl_telem_t         ctl;
    pt_acs_t               acs;
    pt_tc_t                tc;
    pt_cpu_t               cpu;
    pt_expt_t              expt;
    pt_dyno_load_t         dyno_load;
    pt_contactor_cmd_t     contactor_cmd;
    volatile uint32_t      supervisor_heartbeat;  /* TODO: convert to _Atomic in follow-up PR */
    uint16_t               fault_bits;
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
