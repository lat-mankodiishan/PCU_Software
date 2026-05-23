#ifndef POWERTRAIN_STATE_H
#define POWERTRAIN_STATE_H

#include <stdint.h>
#include "cmsis_os2.h"
#include "vesc_proto.h"

/* Hardware-of-last-resort clamps on rectifier setpoints. control_law
 * may apply tighter ceilings; these are the absolute walls.
 * Mirror these on the VESC side (hybrid_pcu.c). */
#define I_RECT_MAX_CA           16000      /* 160.00 A           */
#define OMEGA_E_MAX_ERPM       100000      /* 100k electrical RPM */
#define DUTY_MAX_X10000          9500      /* 95.00 % duty       */

/* Engine throttle servo (TIM1_CH1 / PA8). 1100 µs = 0 % (idle), 1900 µs
 * = 100 % (max). Standard 50 Hz servo signal; ECU's onboard governor
 * smooths command-to-rpm. Linear mapping done in pt_set_engine_throttle_*. */
#define ENGINE_THROTTLE_PULSE_MIN_US   1100u
#define ENGINE_THROTTLE_PULSE_MAX_US   1900u

/* Aggregated fault bits — OR'd from all detector tasks.
 * supervisor_task reads this each tick and decides actuator state. */
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
    /* Setpoint — supervisor → rectifier_task (5 ms)
     * rect_ctrl_mode picks which one of the three is on the wire. */
    rect_ctrl_mode_t  rect_ctrl_mode;
    int16_t           I_rect_cmd_cA;       /* 0.01 A LSB,  for RECT_CTRL_CURRENT */
    int32_t           omega_e_cmd_erpm;    /* 1 eRPM LSB,  for RECT_CTRL_OMEGA   */
    int16_t           duty_cmd_x10000;     /* 0.01 % LSB,  for RECT_CTRL_DUTY    */
    vesc_mode_t       mode;

    /* Motor-type command — VESC's mc_motor_type. Drives 0x104 SendMotorTypeCmd
     * (TXed on change + slow keep-alive). VESC dedups, so writing repeatedly
     * is harmless. Default = FOC for normal regen operation. */
    vesc_motor_type_t rect_motor_type;

    /* Invert-direction command — drives 0x105 SendInvertDirCmd. Same TX
     * pattern as motor_type. Use to compensate for the BLDC vs FOC
     * direction-convention flip in VESC when handing off between modes. */
    bool              rect_invert_direction;

    /* Telemetry — rectifier_task → readers */
    vesc_rect_state_t rect_state;
    uint32_t          rect_state_tick;

    /* External inputs — fc_link_task / bms_task / bench stub → supervisor */
    vesc_mode_t       fc_flight_state;
    uint16_t          fc_throttle_dem_pct;     /* 0.01 % LSB */
    uint32_t          fc_input_tick;

    /* BMS — bms_task writes when frames are parsed */
    uint16_t          bms_soc_pct;             /* 0.01 % LSB */
    uint16_t          bms_v_bat_cV;            /* 0.01 V LSB */
    int16_t           bms_i_bat_cA;            /* 0.01 A LSB, signed (+discharge) */
    int8_t            bms_max_cell_C;          /* hottest cell °C */
    uint32_t          bms_input_tick;

    /* ECU — ecu_task writes when MegaSquirt 'A' polls return cleanly.
     * All temp fields are whole °C regardless of firmware's CELSIUS macro
     * (ecu_task does the F→C conversion before mirroring). */
    uint16_t          ecu_rpm;                 /* engine RPM, 1 RPM/LSB */
    uint16_t          ecu_inj_pw_us;           /* injector pulse width, 1 µs/LSB (MS2 pulseWidth1) */
    uint16_t          ecu_fuel_rate_dg_s;      /* 0.1 g/s LSB — TODO: derive from PW × RPM × inj_flow */
    int16_t           ecu_cht_C;               /* coolant/cylinder-head temp, 1 °C/LSB (MS2 coolant) */
    int16_t           ecu_iat_C;               /* intake air temp, 1 °C/LSB (MS2 mat) */
    int16_t           ecu_egt_C;               /* exhaust gas temp probe 1, 1 °C/LSB (MS2 adc6 × 1.222) */
    int16_t           ecu_tps_pct_x10;         /* throttle position, 0.1 %/LSB (MS2 tps) */
    int16_t           ecu_map_kPa_x10;         /* manifold absolute pressure, 0.1 kPa/LSB (MS2 map) */
    int16_t           ecu_batt_cV;             /* ECU's view of supply voltage, 0.01 V/LSB (MS2 batteryVoltage) */
    uint8_t           ecu_engine_status;       /* MS2 engine byte (ready/crank/warmup/etc. flags) — see ecu_task.c */
    uint8_t           ecu_sync_err_cnt;        /* MS2 synccnt — trigger sync errors since boot */
    uint32_t          ecu_input_tick;

    /* Liveness counter — incremented by supervisor each tick.
     * pdb_task (future) watches for this to stop moving. */
    volatile uint32_t supervisor_heartbeat;

    /* Contactor commands — written by supervisor_task, mirrored to GPIOs
     * by pdb_task. true = closed. */
    bool              contactor_battery_cmd;
    bool              contactor_rectifier_cmd;

    /* Aggregated faults — OR of fault_id_t */
    uint16_t          fault_bits;

    /* Dyno test bench — SSD-250A current sensor (ID 5FX-3, on CAN1).
     * Native SSD broadcast LSBs preserved (no float conversion in PCU).
     * Decoder lives in dyno_setup_task.c. */
    int32_t           dyno_current_mA;       /* 1 mA   LSB, signed (charge/discharge) */
    int32_t           dyno_vbus_mV;          /* 1 mV   LSB                            */
    uint32_t          dyno_power_dW;         /* 0.1 W  LSB (= raw from 0x5F5)         */
    uint64_t          dyno_energy_Wh;        /* 1 Wh   LSB (= raw from 0x5F6)         */
    uint32_t          dyno_input_tick;       /* osKernelGetTickCount of last RX       */

    /* Dyno load sensor — second SSD-250A on ID block 4FX-2 (0x4F1..0x4F7).
     * Measures load-side current; used as feedback for the throttle PI loop
     * that drives the load to match the 5FX reference. */
    int32_t           dyno_load_current_mA;    /* 1 mA   LSB, signed */
    int32_t           dyno_load_vbus_mV;       /* 1 mV   LSB         */
    uint32_t          dyno_load_power_dW;      /* 0.1 W  LSB         */
    uint64_t          dyno_load_energy_Wh;     /* 1 Wh   LSB         */
    uint32_t          dyno_load_input_tick;    /* osKernelGetTickCount of last RX */

    /* Throttle PI controller (throttle_ctrl_task.c) — drives load current
     * (4FX) to match source current (5FX) via PWM on PA10 / TIM1_CH3.
     * Default disabled; flip throttle_ctrl_enabled = true at runtime
     * once both sensors are confirmed reading sane values. */
    bool              throttle_ctrl_enabled;   /* gate the loop                  */
    uint16_t          throttle_pwm_duty;       /* 0..1599 (TIM1 ARR = 1599)      */
    int32_t           throttle_current_err_mA; /* 4FX − 5FX_filt, debug          */
    int32_t           throttle_src_filt_mA;    /* 5FX after N-tap moving avg     */
    uint32_t          throttle_ctrl_tick;      /* last loop iteration timestamp  */

    /* Engine throttle servo (PA8 / TIM1_CH1). Operator writes the
     * REQUEST field (req_pct_x100) — supervisor picks it up each 10 ms
     * tick and pushes to hardware via pt_set_engine_throttle_pct_x100().
     * The setter also updates the other two mirror fields (pct_x100 +
     * pulse_us) for telemetry. Live-tune from Live Watch: write
     * engine_throttle_req_pct_x100 = 5000 → servo moves to 50 % within
     * one supervisor tick. */
    uint16_t          engine_throttle_req_pct_x100; /* operator setpoint, 0..10000 */
    uint16_t          engine_throttle_pct_x100;    /* mirror of last applied request */
    uint16_t          engine_throttle_pulse_us;    /* 1100..1900 µs, mirror of CCR1 */
    uint32_t          engine_throttle_tick;        /* last setter call timestamp */

    /* Onboard current sensors (ACS772ECB-300B, ±300 A, 6.66 mV/A) wired to
     * ADS1262 AIN0/AIN1/AIN2 vs AINCOM. Written by sensor_task after each
     * full ADC scan. mA values are signed; bidirectional sensor convention
     * (sign matches the sensor's IP+/IP- pin labeling on the PCB). */
    int32_t           current_sensor_mA[3];        /* I_1, I_2, I_3 in 1 mA LSB, signed */
    uint32_t          current_sensor_tick;         /* osKernelGetTickCount of last scan */

    /* RTOS health — written by rtos_stats_task every ~5 s. Derived from
     * the IDLE task's runtime counter delta vs the total counter delta.
     * The full per-task table is dumped over SWO (see rtos_stats.c). */
    uint8_t           cpu_load_pct;                /* 0..100, last 5-second window */
    uint32_t          cpu_load_tick;               /* osKernelGetTickCount of last update */

    /* Experiment runner state — written by experiment.c, read by supervisor
     * (gates the I_bat closed-loop) and log_task. expt_label points into
     * static-const profile data, so it lives as long as the firmware does.
     * The two _req flags are operator hooks: set from the debugger or via
     * expt_advance() / expt_abort(). */
    bool              expt_active;
    uint8_t           expt_phase_idx;
    const char       *expt_label;
    bool              expt_advance_req;
    bool              expt_abort_req;
    bool              expt_start_req;        /* set true to start the boot profile */
} powertrain_state_t;


extern powertrain_state_t g_pt;
extern osMutexId_t        g_pt_mtx;

/* Call once before osKernelStart. */
void     pt_init(void);

/* Setpoint write helpers — called by supervisor / bench stubs.
 * Each setter atomically sets the value AND switches rect_ctrl_mode to
 * the matching control type, so rectifier_task always TXes the frame
 * that corresponds to the most recent setter call. */
void     pt_set_setpoint      (int16_t I_rect_cmd_cA,    vesc_mode_t mode);
void     pt_set_setpoint_omega(int32_t omega_e_cmd_erpm, vesc_mode_t mode);
void     pt_set_setpoint_duty (int16_t duty_cmd_x10000,  vesc_mode_t mode);

/* Motor-type command — independent of setpoint. The next 0x104 frame TX
 * carries this value. Idempotent on the VESC side (dedups before re-init). */
void     pt_set_motor_type    (vesc_motor_type_t type);

/* Invert-direction command — independent of setpoint. The next 0x105 frame
 * TX carries this value. Idempotent on the VESC side (dedups before re-init).
 * Use to compensate for the BLDC<->FOC direction-convention flip in VESC. */
void     pt_set_invert_direction(bool invert);

/* Engine throttle servo setter. Argument is 0..10000 (0.01 % LSB); clamped
 * to that range. Computes the corresponding 1100..1900 µs pulse, mirrors
 * to g_pt for telemetry, and updates the TIM1_CH1 CCR via esc_hw_set_us. */
void     pt_set_engine_throttle_pct_x100(uint16_t pct_x100);

void     pt_set_fault(uint16_t mask);
void     pt_clear_fault(uint16_t mask);
uint16_t pt_get_faults(void);

void pt_set_fc_inputs(vesc_mode_t flight_state, uint16_t throttle_dem_pct);
void pt_set_bms_inputs(uint16_t soc_pct);

void pt_set_contactor_cmds(bool battery_closed, bool rectifier_closed);


#endif /* POWERTRAIN_STATE_H */
