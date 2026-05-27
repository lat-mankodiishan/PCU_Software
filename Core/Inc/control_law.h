#ifndef CONTROL_LAW_H
#define CONTROL_LAW_H

#include <stdint.h>
#include <stdbool.h>
#include "vesc_proto.h"

/* Percent fields: 0.01 %/LSB (0..10000 = 0..100 %).
 * V1: I_bat PID -> I_rect_demand -> P_rect -> LUT -> (duty, theta_engine). */

typedef struct {
    /* PID gains; error = i_bat_meas - i_bat_ref (cA, +discharge). */
    int32_t  kp_q15;
    int32_t  ki_q15;                  /* per-tick */
    int32_t  kd_q15;                  /* per-tick */
    int32_t  i_term_max_cA;           /* +-anti-windup clamp */

    /* Setpoint + output shaping. */
    int16_t  i_bat_ref_cA;            /* +discharge target */
    int16_t  i_rect_max_cA;           /* hard clamp on demand */
    int16_t  i_rect_slew_cA;          /* max change per tick */

    /* Input filter on I_bat. */
    uint16_t i_bat_alpha_q15;         /* EMA */

    /* Anti-stall on gen_rpm. */
    uint16_t omega_min_rpm;           /* derate above floor */
    uint16_t omega_floor_rpm;         /* hard cut */

    /* Safety: raise I_bat_ref (battery picks up slack) when limits hit. */
    int16_t  i_rect_sat_margin_cA;    /* "demand near i_rect_max" margin */
    uint16_t theta_sat_x100;          /* "throttle near 100%" margin */
    int16_t  i_bat_ref_boost_cA;      /* added on saturation */
} ctl_v1_params_t;

typedef struct {
    int32_t  i_term_cA;               /* integrator */
    int16_t  i_bat_filt_cA;           /* EMA state */
    int16_t  i_bat_prev_cA;           /* for derivative */
    int16_t  i_rect_demand_cA;        /* slew-limited output */
    bool     init_done;
} ctl_v1_state_t;

typedef struct {
    int16_t  i_bat_meas_cA;           /* ACS ch2 */
    uint16_t v_bus_cV;                /* rect_state.V_dc_cV */
    uint16_t gen_rpm;                 /* rect_state.gen_rpm */
} ctl_v1_inputs_t;

typedef struct {
    int16_t  i_rect_demand_cA;
    uint16_t duty_x10000;
    uint16_t theta_engine_pct_x100;
    uint32_t p_rect_W;
    int16_t  i_bat_ref_eff_cA;        /* setpoint after safety boost */
} ctl_v1_output_t;

void control_law_v1_init(ctl_v1_state_t *st);
void control_law_v1_default_params(ctl_v1_params_t *p);

void control_law_v1_step(ctl_v1_state_t *st,
                         const ctl_v1_inputs_t *in,
                         const ctl_v1_params_t *p,
                         ctl_v1_output_t *out);

/* Placeholder LUT (linear); replace with bench-calibrated table. */
void control_law_v1_lut(uint32_t  p_rect_W,
                        uint16_t *duty_x10000,
                        uint16_t *theta_pct_x100);

#endif /* CONTROL_LAW_H */
