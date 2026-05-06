#ifndef POWERTRAIN_STATE_H
#define POWERTRAIN_STATE_H

#include <stdint.h>
#include "cmsis_os2.h"
#include "vesc_proto.h"

/* Hardware-of-last-resort clamp on rectifier current command.
 * Tighten in control_law if you want a softer ceiling. */
#define I_RECT_MAX_CA           16000      /* 160.00 A */

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
    /* Setpoint — supervisor → rectifier_task (5 ms) */
    int16_t           I_rect_cmd_cA;
    vesc_mode_t       mode;

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
} powertrain_state_t;


extern powertrain_state_t g_pt;
extern osMutexId_t        g_pt_mtx;

/* Call once before osKernelStart. */
void     pt_init(void);

/* Setpoint write helper — called by supervisor / bench stubs. */
void     pt_set_setpoint(int16_t I_rect_cmd_cA, vesc_mode_t mode);

void     pt_set_fault(uint16_t mask);
void     pt_clear_fault(uint16_t mask);
uint16_t pt_get_faults(void);

void pt_set_fc_inputs(vesc_mode_t flight_state, uint16_t throttle_dem_pct);
void pt_set_bms_inputs(uint16_t soc_pct);

void pt_set_contactor_cmds(bool battery_closed, bool rectifier_closed);


#endif /* POWERTRAIN_STATE_H */
