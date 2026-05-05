#ifndef CONTROL_LAW_H
#define CONTROL_LAW_H

#include <stdint.h>
#include "vesc_proto.h"   /* vesc_mode_t */

/* All percent fields use 0.01 % LSB → 0..10000 = 0..100.00 % */

typedef struct {
    /* Modal limits */
    int16_t  I_rect_peak_cA;          /* used in TAKEOFF / CLIMB / LAND */

    /* CRUISE tracking */
    int16_t  I_load_full_cA;          /* expected load current at 100% throttle */
    uint16_t deadband_pct;            /* +-deadband on (throttle_dem - throttle_filt) */
    int16_t  trim_step_cA;            /* slew per call (cA per tick) */

    /* Low-SOC bias (additive in CRUISE only) */
    uint16_t soc_low_threshold_pct;   /* below this, raise setpoint */
    int16_t  soc_bias_cA;

    /* EMA filter on throttle, Q15 fixed-point alpha (0..32767) */
    uint16_t throttle_alpha_q15;

    /* I_bat closed-loop (control_law_step_ibat). Setpoint = 0 A on the
     * battery; rectifier command is the actuator. Reuses trim_step_cA
     * for the per-call slew limit. */
    uint16_t ibat_kp_q15;             /* Q15 P gain: i_bat error → cmd adj */
    int16_t  ibat_deadband_cA;        /* hold cmd while |i_bat| < this */
} ctl_params_t;

typedef struct {
    uint16_t throttle_filt_pct;       /* EMA filter state */
    int16_t  I_rect_cmd_cA;           /* integrator state — last setpoint */
} ctl_state_t;

typedef struct {
    vesc_mode_t mode;
    uint16_t    throttle_dem_pct;     /* from FC */
    uint16_t    soc_pct;              /* from BMS */
} ctl_inputs_t;

void    control_law_init(ctl_state_t *st);
void    control_law_default_params(ctl_params_t *p);

/* Pure function: returns new I_rect_cmd_cA, advances st in place. */
int16_t control_law_step(ctl_state_t *st,
                         const ctl_inputs_t *in,
                         const ctl_params_t *p);

/* I_bat = 0 closed-loop. Caller should gate on g_pt.bms_input_tick
 * advancing — Daly updates ~20 Hz; calling each supervisor tick (100 Hz)
 * would slew 5× per measurement and overshoot. */
int16_t control_law_step_ibat(ctl_state_t *st,
                              int16_t i_bat_cA,
                              const ctl_params_t *p);

#endif /* CONTROL_LAW_H */
