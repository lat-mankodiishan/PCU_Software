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

static bool cond_button_released(const powertrain_state_t *pt) {
    (void)pt;
    return HAL_GPIO_ReadPin(EXPT_BTN_PORT, EXPT_BTN_PIN) == GPIO_PIN_RESET;
}

/* --- Starter-generator dyno run -----------------------------------------
 * Cranks the SG in BLDC duty (fixed voltage, handles engine TDC spikes
 * cleanly), then on button press releases the motor so the engine can
 * spin the SG freely without BLDC commutation fighting it. Button
 * release advances to FOC handoff + duty ladder.
 *
 *   P0: BLDC, 5 % duty — cranks the engine through TDC. Held until the
 *       button GPIO goes HIGH (you press when engine has fired).
 *   P1: BLDC, CURRENT = 0 — released. Engine free-spools the SG; throttle
 *       up here. Held while button is HELD; advances when you release it.
 *       (Quick tap = blast through. Long press = throttle ramp window.)
 *   P2: FOC + INVERTED, CURRENT = 0, 0.5 s — observer locks onto the
 *       spinning rotor at zero torque before duty kicks in.
 *   P3: FOC, INVERTED, duty = 10 %, 5 s.
 *   P4: FOC, INVERTED, duty = 14 %, 5 s.
 *   P5: FOC, INVERTED, duty = 18 %, 5 s.
 *   (exit -> CURRENT 0 IDLE)
 *
 * Wire units: duty in 0.01 %/LSB (500 = 5.00 %, 1000 = 10.00 %, etc.).
 *
 * Why P1 is BLDC current=0 not duty=0: duty=0 leaves the FETs in a
 * synchronous-rectification configuration each commutation step, which
 * acts as a dynamic brake when the engine over-spins the rotor. CURRENT
 * mode at 0 A regulates phase current to zero so the rotor free-wheels
 * with minimal opposition. */
static const expt_phase_t starter_gen_phases[] = {
    /* P0: BLDC crank at 5 % duty. Hold until button PRESSED (HIGH).
     * Press the button the moment the engine fires.
     * PA3 driven HIGH — signals "cranking active". */
    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 500,             /* 5.00 % */
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_COND,
      .hold_cond = cond_button_pressed,
      .motor_type = EXPT_MOTOR_TYPE_BLDC,
      .invert_dir = EXPT_INVERT_DIR_NORMAL,
      .aux_gpio  = EXPT_AUX_GPIO_HIGH,
      .pt_mode   = VESC_MODE_TAKEOFF, .label = "P0 BLDC duty=5pct wait btn" },

    /* P1: BLDC release (current = 0). Hold while button is HELD HIGH;
     * advances when you release the button. Throttle up the engine
     * during this window. PA3 stays HIGH (KEEP). */
    { .ctrl_mode = RECT_CTRL_CURRENT, .setpoint = 0,
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_COND,
      .hold_cond = cond_button_released,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,                          /* still BLDC */
      .invert_dir = EXPT_INVERT_DIR_KEEP,
      .aux_gpio  = EXPT_AUX_GPIO_KEEP,                             /* stays HIGH */
      .pt_mode   = VESC_MODE_IDLE,    .label = "P1 BLDC release hold btn" },

    /* P2: switch to FOC + INVERTED, current still 0. Observer locks
     * onto the engine-driven rotor before any duty is applied.
     * PA3 driven LOW — signals "FOC takeover, no longer cranking". */
    { .ctrl_mode = RECT_CTRL_CURRENT, .setpoint = 0,
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_TIME, .hold_ms = 500,
      .motor_type = EXPT_MOTOR_TYPE_FOC,
      .invert_dir = EXPT_INVERT_DIR_INVERTED,
      .aux_gpio  = EXPT_AUX_GPIO_LOW,
      .pt_mode   = VESC_MODE_TAKEOFF, .label = "P2 FOC inv release lock" },

    /* P3..P5: duty ladder inside FOC, 0.5 s ramp + 5 s hold per step.
     * PA3 stays LOW (KEEP). */
    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 1000,            /* 10.00 % */
      .ramp_ms   = 0,    .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,
      .invert_dir = EXPT_INVERT_DIR_KEEP,
      .aux_gpio  = EXPT_AUX_GPIO_KEEP,
      .pt_mode   = VESC_MODE_CRUISE,  .label = "P3 FOC inv duty=10pct" },

    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 1400,            /* 14.00 % */
      .ramp_ms   = 500,  .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,
      .invert_dir = EXPT_INVERT_DIR_KEEP,
      .aux_gpio  = EXPT_AUX_GPIO_KEEP,
      .pt_mode   = VESC_MODE_CRUISE,  .label = "P4 FOC inv duty=14pct" },

    { .ctrl_mode = RECT_CTRL_DUTY,    .setpoint = 1800,            /* 18.00 % */
      .ramp_ms   = 500,  .hold_type = EXPT_HOLD_TIME, .hold_ms = 5000,
      .motor_type = EXPT_MOTOR_TYPE_KEEP,
      .invert_dir = EXPT_INVERT_DIR_KEEP,
      .aux_gpio  = EXPT_AUX_GPIO_KEEP,
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