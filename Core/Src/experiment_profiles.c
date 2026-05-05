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
