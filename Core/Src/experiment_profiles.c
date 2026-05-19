#include "experiment_profiles.h"
#include "main.h"               /* GPIO_Pin defines */
#include "stm32f4xx_hal.h"      /* HAL_GPIO_ReadPin */

/* Phase labels must not contain commas — log_task emits them raw into CSV. */

/* ---- Button GPIO for HOLD_COND advance ---------------------------------
 * Switch wired between this pin and 3V3. Configured as INPUT in CubeMX
 * (.ioc), so MX_GPIO_Init() at boot owns the pin setup. Make sure the
 * CubeMX config has pull-down enabled (or use an external pull-down) —
 * otherwise an open switch floats and reads unpredictably. */
#define EXPT_BTN_PORT       GPIOA
#define EXPT_BTN_PIN        GPIO_PIN_7

static bool cond_button_pressed(const powertrain_state_t *pt) {
    (void)pt;
    return HAL_GPIO_ReadPin(EXPT_BTN_PORT, EXPT_BTN_PIN) == GPIO_PIN_SET;
}

/* --- Starter-generator dyno run -----------------------------------------
 * Single-switch handoff. Stay in BLDC at 5 % crank duty; manually
 * throttle the engine up in this phase. Once the engine is driving the
 * SG faster than the commanded duty you'll see regen on the bus —
 * that's the cue to flip the switch HIGH, which hands off to FOC.
 *
 *   P0: BLDC, 5 % duty — cranks through TDC, then becomes regen as the
 *       engine accelerates past the commanded speed. Held until switch
 *       goes HIGH.
 *   P1: BLDC, CURRENT = 0, 0.3 s — auto-bleed. Drops phase current to 0
 *       in BLDC mode BEFORE the motor-type swap, so FETs go into a
 *       controlled free-wheel state. Without this the BLDC→FOC reconfig
 *       happens while current is flowing → body-diode regen pulses and
 *       cogging when the FOC observer cold-starts.
 *   P2: FOC + INVERTED, CURRENT = 0, 1.0 s — observer locks onto the
 *       spinning rotor at zero torque before duty kicks in.
 *   P3: FOC, INVERTED, duty = 10 %, 5 s.
 *   P4: FOC, INVERTED, duty = 14 %, 5 s.
 *   P5: FOC, INVERTED, duty = 18 %, 5 s.
 *   (exit -> CURRENT 0 IDLE)
 *
 * Wire units: duty in 0.01 %/LSB (500 = 5.00 %, 1000 = 10.00 %, etc.). */
static const expt_phase_t starter_gen_phases[] = {
    /* P0: BLDC crank at 5 % duty. Throttle the engine up by hand here;
     * once you see regen on the bus, flip the switch HIGH to advance. */
    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 500,             /* 5.00 % */
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_COND,
      .hold_cond = cond_button_pressed,
      .motor_type = EXPT_MOTOR_TYPE_BLDC,
      .invert_dir = EXPT_INVERT_DIR_NORMAL,
      .pt_mode   = VESC_MODE_TAKEOFF, .label = "P0 BLDC duty=5pct wait switch" },

    /* P1: auto-bleed. Still BLDC + NORMAL, but switch to CURRENT = 0
     * for 300 ms so phase current decays to zero before the motor-type
     * swap. */
    { .ctrl_mode = RECT_CTRL_CURRENT, .setpoint = 0,
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_TIME, .hold_ms = 300,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,                          /* still BLDC */
      .invert_dir = EXPT_INVERT_DIR_KEEP,
      .pt_mode   = VESC_MODE_TAKEOFF, .label = "P1 BLDC bleed I=0" },

    /* P2: switch to FOC + INVERTED, current = 0. Observer locks onto
     * the engine-driven rotor before any duty is applied. 1.0 s gives
     * the back-EMF observer comfortable headroom to converge. */
    { .ctrl_mode = RECT_CTRL_CURRENT, .setpoint = 0,
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_TIME, .hold_ms = 1000,
      .motor_type = EXPT_MOTOR_TYPE_FOC,
      .invert_dir = EXPT_INVERT_DIR_INVERTED,
      .pt_mode   = VESC_MODE_TAKEOFF, .label = "P2 FOC inv observer lock" },

    /* P3..P5: duty ladder inside FOC, 0.5 s ramp + 5 s hold per step. */
    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 1000,            /* 10.00 % */
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,
      .invert_dir = EXPT_INVERT_DIR_KEEP,
      .pt_mode   = VESC_MODE_CRUISE,  .label = "P3 FOC inv duty=10pct" },

    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 1400,            /* 14.00 % */
      .ramp_ms   = 500,  .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,
      .invert_dir = EXPT_INVERT_DIR_KEEP,
      .pt_mode   = VESC_MODE_CRUISE,  .label = "P4 FOC inv duty=14pct" },

    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 1800,            /* 18.00 % */
      .ramp_ms   = 500,  .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,
      .invert_dir = EXPT_INVERT_DIR_KEEP,
      .pt_mode   = VESC_MODE_CRUISE,  .label = "P5 FOC inv duty=18pct" },

    /* expt_run() exits to (CURRENT, 0, IDLE) automatically after P5 —
     * clean motor release without a duty-ramp-down regen spike. */
};
const expt_profile_t starter_gen_profile = {
    .name     = "starter_gen",
    .phases   = starter_gen_phases,
    .n_phases = (uint8_t)(sizeof(starter_gen_phases) / sizeof(starter_gen_phases[0])),
    .loop     = false,
};