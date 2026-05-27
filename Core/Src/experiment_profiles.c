#include "experiment_profiles.h"
#include "main.h"
#include "stm32f4xx_hal.h"

/* Phase labels must not contain commas (log_task CSV). */

/* PA7 switch to 3V3; CubeMX must set pull-down or wire external pull-down. */
#define EXPT_BTN_PORT       GPIOA
#define EXPT_BTN_PIN        GPIO_PIN_7

static bool cond_button_pressed(const powertrain_state_t *pt) {
    (void)pt;
    return HAL_GPIO_ReadPin(EXPT_BTN_PORT, EXPT_BTN_PIN) == GPIO_PIN_SET;
}

/* ---- starter_gen: BLDC crank -> bleed -> FOC observer lock -> duty ladder. */
static const expt_phase_t starter_gen_phases[] = {
    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 500,             /* 5.00 % */
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_COND,
      .hold_cond = cond_button_pressed,
      .motor_type = EXPT_MOTOR_TYPE_BLDC,
      .invert_dir = EXPT_INVERT_DIR_NORMAL,
      .pt_mode   = MODE_TAKEOFF, .label = "P0 BLDC duty=5pct wait switch" },

    /* P1: BLDC bleed; CURRENT=0 before motor-type swap to avoid body-diode regen. */
    { .ctrl_mode = RECT_CTRL_CURRENT, .setpoint = 0,
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_TIME, .hold_ms = 300,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,
      .invert_dir = EXPT_INVERT_DIR_KEEP,
      .pt_mode   = MODE_TAKEOFF, .label = "P1 BLDC bleed I=0" },

    /* P2: FOC INVERTED observer lock at zero torque. */
    { .ctrl_mode = RECT_CTRL_CURRENT, .setpoint = 0,
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_TIME, .hold_ms = 1000,
      .motor_type = EXPT_MOTOR_TYPE_FOC,
      .invert_dir = EXPT_INVERT_DIR_INVERTED,
      .pt_mode   = MODE_TAKEOFF, .label = "P2 FOC inv observer lock" },

    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 1000,            /* 10.00 % */
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,
      .invert_dir = EXPT_INVERT_DIR_KEEP,
      .pt_mode   = MODE_CRUISE,  .label = "P3 FOC inv duty=10pct" },

    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 1400,            /* 14.00 % */
      .ramp_ms   = 500,  .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,
      .invert_dir = EXPT_INVERT_DIR_KEEP,
      .pt_mode   = MODE_CRUISE,  .label = "P4 FOC inv duty=14pct" },

    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 1800,            /* 18.00 % */
      .ramp_ms   = 500,  .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,
      .invert_dir = EXPT_INVERT_DIR_KEEP,
      .pt_mode   = MODE_CRUISE,  .label = "P5 FOC inv duty=18pct" },
};
const expt_profile_t starter_gen_profile = {
    .name     = "starter_gen",
    .phases   = starter_gen_phases,
    .n_phases = (uint8_t)(sizeof(starter_gen_phases) / sizeof(starter_gen_phases[0])),
    .loop     = false,
};

/* ---- engine_throttle_sweep: higher-duty companion to starter_gen. */
static const expt_phase_t engine_throttle_sweep_phases[] = {
    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 4200,            /* 42.00 % */
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_COND,
      .hold_cond = cond_button_pressed,
      .motor_type = EXPT_MOTOR_TYPE_BLDC,
      .invert_dir = EXPT_INVERT_DIR_NORMAL,
      .pt_mode   = MODE_TAKEOFF, .label = "P0 BLDC duty=42pct wait switch" },

    { .ctrl_mode = RECT_CTRL_CURRENT, .setpoint = 0,
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_TIME, .hold_ms = 300,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,
      .invert_dir = EXPT_INVERT_DIR_KEEP,
      .pt_mode   = MODE_TAKEOFF, .label = "P1 BLDC bleed I=0" },

    { .ctrl_mode = RECT_CTRL_CURRENT, .setpoint = 0,
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_TIME, .hold_ms = 1000,
      .motor_type = EXPT_MOTOR_TYPE_FOC,
      .invert_dir = EXPT_INVERT_DIR_INVERTED,
      .pt_mode   = MODE_TAKEOFF, .label = "P2 FOC inv observer lock" },

    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 6000,            /* 60.00 % */
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_TIME, .hold_ms = 10000,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,
      .invert_dir = EXPT_INVERT_DIR_KEEP,
      .pt_mode   = MODE_CRUISE,  .label = "P3 FOC inv duty=60pct" },

    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 7500,            /* 75.00 % */
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_TIME, .hold_ms = 100000,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,
      .invert_dir = EXPT_INVERT_DIR_KEEP,
      .pt_mode   = MODE_CRUISE,  .label = "P4 FOC inv duty=75pct" },
};
const expt_profile_t engine_throttle_sweep_profile = {
    .name     = "engine_throttle_sweep",
    .phases   = engine_throttle_sweep_phases,
    .n_phases = (uint8_t)(sizeof(engine_throttle_sweep_phases) / sizeof(engine_throttle_sweep_phases[0])),
    .loop     = false,
};
