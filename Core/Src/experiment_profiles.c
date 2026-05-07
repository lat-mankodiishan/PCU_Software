#include "experiment_profiles.h"

/* Phase labels must not contain commas — log_task emits them raw into CSV. */

/* --- Starter-generator manual bring-up ----------------------------------
 * Phase 0: motor the engine at 6000 eRPM. Operator advances throttle by
 *          hand until rect_state shows positive I_dc (generation), then
 *          pokes g_pt.expt_advance_req from the debugger.
 * Phase 1: switch to current control, draw 10 A from the alternator.
 *          Operator advances when the current-mode test is done.
 * Phase 2: ramp to 0 A, hold for shutdown. */
static const expt_phase_t starter_gen_phases[] = {
    { .ctrl_mode = RECT_CTRL_OMEGA,   .setpoint = 6000,
      .ramp_ms   = 2000, .hold_type = EXPT_HOLD_OP,
      .pt_mode   = VESC_MODE_TAKEOFF, .label = "motor 6000 eRPM" },

    { .ctrl_mode = RECT_CTRL_CURRENT, .setpoint = 1000,    /* 10.00 A */
      .ramp_ms   = 500,  .hold_type = EXPT_HOLD_OP,
      .pt_mode   = VESC_MODE_CRUISE,  .label = "generate 10A" },

    { .ctrl_mode = RECT_CTRL_CURRENT, .setpoint = 0,
      .ramp_ms   = 500,  .hold_type = EXPT_HOLD_OP,
      .pt_mode   = VESC_MODE_IDLE,    .label = "idle" },
};
const expt_profile_t starter_gen_profile = {
    .name     = "starter_gen",
    .phases   = starter_gen_phases,
    .n_phases = (uint8_t)(sizeof(starter_gen_phases) / sizeof(starter_gen_phases[0])),
    .loop     = false,
};

/* --- Phase 1: VESC-as-motor bring-up ------------------------------------
 * Single VESC + free-shaft motor + bench PSU. No coupling, no battery.
 * Walks through all three control modes (CURRENT, OMEGA, DUTY) at low,
 * safe setpoints. Every phase blocks on operator advance — read VESC Tool
 * RT data + the sniffer + multimeter between transitions.
 *
 * Bench safety:
 *   - PSU current-limited (≤2 A), V≈24 V.
 *   - Free shaft: motor will spin at whatever back-EMF balances. Watch RPM.
 *   - Each transition returns to a safe value before changing mode. */
static const expt_phase_t phase1_motor_only_phases[] = {
    /* P0: idle, current=0. Operator preps + advances when ready. */
    { .ctrl_mode = RECT_CTRL_CURRENT, .setpoint = 0,
      .ramp_ms   = 0,   .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .pt_mode   = VESC_MODE_IDLE,    .label = "P0 idle" },

    /* P1: small motoring current (+1.00 A). Verify shaft direction. */
    { .ctrl_mode = RECT_CTRL_CURRENT, .setpoint = 1000,        /*  +1.00 A */
      .ramp_ms   = 1000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .pt_mode   = VESC_MODE_TAKEOFF, .label = "P1 I=+1A motoring" },

    /* P2: small regen current (-1.00 A). Direction should reverse. */
    { .ctrl_mode = RECT_CTRL_CURRENT, .setpoint = -1000,       /*  -1.00 A */
      .ramp_ms   = 1000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .pt_mode   = VESC_MODE_LAND,    .label = "P2 I=-1A regen" },

    /* P3: ramp to zero before mode switch. */
    { .ctrl_mode = RECT_CTRL_CURRENT, .setpoint = 0,
      .ramp_ms   = 500,  .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .pt_mode   = VESC_MODE_IDLE,    .label = "P3 zero pre-omega" },

    /* P4: low speed setpoint, 2000 eRPM. Verify mech RPM = eRPM/pole-pairs.
     *      Needs FOC parameters identified in VESC Tool first. */
    { .ctrl_mode = RECT_CTRL_OMEGA,   .setpoint = 2000,       /* 2000 eRPM */
      .ramp_ms   = 2000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .pt_mode   = VESC_MODE_CRUISE,  .label = "P4 omega=2k eRPM" },

    /* P5: spindown to zero speed. */
    { .ctrl_mode = RECT_CTRL_OMEGA,   .setpoint = 0,
      .ramp_ms   = 2000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .pt_mode   = VESC_MODE_IDLE,    .label = "P5 omega=0" },

    /* P6: low duty cycle (10 %). Motor spins to back-EMF balance. */
    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 1000,       /* 10.00 %  */
      .ramp_ms   = 1000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .pt_mode   = VESC_MODE_TAKEOFF, .label = "P6 duty=10%" },

    /* P7: duty back to zero. expt_run() exits to (CURRENT, 0, IDLE). */
    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 0,
      .ramp_ms   = 1000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .pt_mode   = VESC_MODE_IDLE,    .label = "P7 duty=0 done" },
};
const expt_profile_t phase1_motor_only_profile = {
    .name     = "phase1_motor_only",
    .phases   = phase1_motor_only_phases,
    .n_phases = (uint8_t)(sizeof(phase1_motor_only_phases) / sizeof(phase1_motor_only_phases[0])),
    .loop     = false,
};

/* --- Automatic dyno sweep -----------------------------------------------
 * eRPM ladder: ramp up over ramp_ms, hold for hold_ms, then move to next.
 * Edit numbers freely. */
static const expt_phase_t dyno_sweep_phases[] = {
    { .ctrl_mode = RECT_CTRL_OMEGA, .setpoint = 2000,
      .ramp_ms   = 5000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .pt_mode   = VESC_MODE_CLIMB,  .label = "ramp 0-2k" },

    { .ctrl_mode = RECT_CTRL_OMEGA, .setpoint = 4000,
      .ramp_ms   = 3000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 10000,
      .pt_mode   = VESC_MODE_CLIMB,  .label = "hold 4k" },

    { .ctrl_mode = RECT_CTRL_OMEGA, .setpoint = 6000,
      .ramp_ms   = 3000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 10000,
      .pt_mode   = VESC_MODE_CRUISE, .label = "hold 6k" },

    { .ctrl_mode = RECT_CTRL_OMEGA, .setpoint = 0,
      .ramp_ms   = 5000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 1000,
      .pt_mode   = VESC_MODE_IDLE,   .label = "spindown" },
};
const expt_profile_t dyno_sweep_profile = {
    .name     = "dyno_sweep",
    .phases   = dyno_sweep_phases,
    .n_phases = (uint8_t)(sizeof(dyno_sweep_phases) / sizeof(dyno_sweep_phases[0])),
    .loop     = false,
};
