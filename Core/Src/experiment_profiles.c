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

/* --- Phase 1: VESC-as-motor omega sweep ---------------------------------
 * Single VESC + free-shaft motor + bench PSU. No coupling, no battery.
 * Stepped sweep through omega setpoints from 1000 to 8000 eRPM.
 * Each level: linear ramp from previous, then 5 s hold. Spindown at end.
 *
 * Reminder on units:
 *   shaft mech RPM = setpoint_eRPM / pole_pairs
 *   (e.g. 8000 eRPM with 4 pole pairs = 2000 mech RPM at the shaft)
 *
 * Bench safety:
 *   - PSU current-limited (≤5 A) appropriate to the motor.
 *   - Free shaft will spin at the commanded eRPM.
 *   - VESC Speed PID Min ERPM in motor config must be ≤ smallest setpoint
 *     converted to mech RPM, else the loop refuses to engage. */
static const expt_phase_t phase1_motor_only_phases[] = {
    /* P0: idle pre-arm. */
    { .ctrl_mode = RECT_CTRL_OMEGA,   .setpoint = 0,
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_TIME, .hold_ms = 3000,
      .pt_mode   = VESC_MODE_IDLE,    .label = "P0 idle" },

    /* P1..P5: ladder up through omega setpoints. */
    { .ctrl_mode = RECT_CTRL_OMEGA,   .setpoint = 1000,       /* 1000 eRPM */
      .ramp_ms   = 2000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .pt_mode   = VESC_MODE_CRUISE,  .label = "P1 omega=1k" },

    { .ctrl_mode = RECT_CTRL_OMEGA,   .setpoint = 2000,
      .ramp_ms   = 2000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .pt_mode   = VESC_MODE_CRUISE,  .label = "P2 omega=2k" },

    { .ctrl_mode = RECT_CTRL_OMEGA,   .setpoint = 4000,
      .ramp_ms   = 3000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .pt_mode   = VESC_MODE_CRUISE,  .label = "P3 omega=4k" },

    { .ctrl_mode = RECT_CTRL_OMEGA,   .setpoint = 6000,
      .ramp_ms   = 3000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .pt_mode   = VESC_MODE_CRUISE,  .label = "P4 omega=6k" },

    { .ctrl_mode = RECT_CTRL_OMEGA,   .setpoint = 8000,
      .ramp_ms   = 3000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .pt_mode   = VESC_MODE_CRUISE,  .label = "P5 omega=8k" },

    /* P6: spindown to zero. expt_run() exits to (CURRENT, 0, IDLE). */
    { .ctrl_mode = RECT_CTRL_OMEGA,   .setpoint = 0,
      .ramp_ms   = 5000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 2000,
      .pt_mode   = VESC_MODE_IDLE,    .label = "P6 omega=0 done" },
};
const expt_profile_t phase1_motor_only_profile = {
    .name     = "phase1_motor_only",
    .phases   = phase1_motor_only_phases,
    .n_phases = (uint8_t)(sizeof(phase1_motor_only_phases) / sizeof(phase1_motor_only_phases[0])),
    .loop     = false,
};

/* --- Crank in BLDC, hand off to FOC at 2000 eRPM ------------------------
 * Demonstrates the 0x104 motor-type switch:
 *   P0: set BLDC, current=0, brief settle.
 *   P1: BLDC, ramp duty 0 -> 30 %. Hold until rect_state.gen_rpm > 2000
 *       (i.e. eRPM, since the field carries fabsf(mc_interface_get_rpm())
 *       which is electrical RPM in 7.x).
 *   P2: switch to FOC, command omega = 3000 eRPM. Step (no ramp from 0,
 *       motor is already spinning >= 2000 eRPM in BLDC).
 *   P3: FOC, omega 3000 -> 0 spindown.
 *
 * Bench notes:
 *   - 30 % duty into a stalled motor is aggressive. PSU current limit
 *     should be sized for the inrush; expect a brief spike before
 *     back-EMF builds.
 *   - BLDC sensorless startup needs the rotor to commutate cleanly on
 *     zero-cross detection. With light shaft load this works; with a
 *     heavy engine load you may need to tune commutation params first. */
static bool cond_gen_erpm_above_2k(const powertrain_state_t *pt) {
    return pt->rect_state.gen_rpm >= 2000;
}

static const expt_phase_t crank_to_foc_phases[] = {
    /* P0: set BLDC, current=0. Gives VESC time to reconfigure before P1. */
    { .ctrl_mode = RECT_CTRL_CURRENT, .setpoint = 0,
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_TIME, .hold_ms = 1000,
      .motor_type = EXPT_MOTOR_TYPE_BLDC,
      .pt_mode   = VESC_MODE_IDLE,    .label = "P0 set BLDC idle" },

    /* P1: BLDC duty ramp 0->30 %, hold until eRPM exceeds 2000. */
    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 1000,           /* 30.00 % */
      .ramp_ms   = 1000, .hold_type = EXPT_HOLD_COND,
      .hold_cond = cond_gen_erpm_above_2k,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,                          /* still BLDC */
      .pt_mode   = VESC_MODE_TAKEOFF, .label = "P1 BLDC duty=30% crank" },

    /* P2: switch to FOC, step to omega=3000 eRPM. Motor is already
     * spinning >2000 eRPM, so a step is the right transition — a ramp
     * from 0 would tell the speed PID to *decelerate* first. */
    { .ctrl_mode = RECT_CTRL_OMEGA,   .setpoint = 3000,
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_TIME, .hold_ms = 10000,
      .motor_type = EXPT_MOTOR_TYPE_FOC,
      .pt_mode   = VESC_MODE_CRUISE,  .label = "P2 FOC omega=3k" },

    /* P3: spindown. Stays in FOC. */
    { .ctrl_mode = RECT_CTRL_OMEGA,   .setpoint = 0,
      .ramp_ms   = 3000, .hold_type = EXPT_HOLD_TIME, .hold_ms = 1000,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,
      .pt_mode   = VESC_MODE_IDLE,    .label = "P3 FOC spindown" },
};
const expt_profile_t crank_to_foc_profile = {
    .name     = "crank_to_foc",
    .phases   = crank_to_foc_phases,
    .n_phases = (uint8_t)(sizeof(crank_to_foc_phases) / sizeof(crank_to_foc_phases[0])),
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
