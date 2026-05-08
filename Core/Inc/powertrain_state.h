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

    /* ECU — ecu_task writes when frames are parsed */
    uint16_t          ecu_rpm;                 /* engine RPM */
    uint16_t          ecu_fuel_rate_dg_s;      /* 0.1 g/s LSB */
    int16_t           ecu_cht_C;               /* cylinder head temp °C */
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

void     pt_set_fault(uint16_t mask);
void     pt_clear_fault(uint16_t mask);
uint16_t pt_get_faults(void);

void pt_set_fc_inputs(vesc_mode_t flight_state, uint16_t throttle_dem_pct);
void pt_set_bms_inputs(uint16_t soc_pct);

void pt_set_contactor_cmds(bool battery_closed, bool rectifier_closed);


#endif /* POWERTRAIN_STATE_H */
