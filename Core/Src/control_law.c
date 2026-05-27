#include "control_law.h"
#include "powertrain_state.h"
#include <stdint.h>
#include <stdbool.h>

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

void control_law_v1_init(ctl_v1_state_t *st) {
    st->i_term_cA        = 0;
    st->i_bat_filt_cA    = 0;
    st->i_bat_prev_cA    = 0;
    st->i_rect_demand_cA = 0;
    st->init_done        = false;
}

void control_law_v1_default_params(ctl_v1_params_t *p) {
    /* Supervisor runs at 5 Hz; per-tick rates scaled 20x from 100 Hz originals. */
    p->kp_q15              = 16384;   /* 0.5 in Q15: 1 A err -> 0.5 A trim */
    p->ki_q15              =  3280;   /* ~0.5/s integrator gain */
    p->kd_q15              =     0;
    p->i_term_max_cA       =  5000;   /* +-50 A integrator clamp */

    p->i_bat_ref_cA        =   500;   /* +5 A discharge target */
    p->i_rect_max_cA       = I_RECT_MAX_CA;
    p->i_rect_slew_cA      =   400;   /* 400 cA/tick @ 5 Hz = 20 A/s */

    p->i_bat_alpha_q15     =  3000;   /* per-tick EMA */

    p->omega_min_rpm       =  2500;
    p->omega_floor_rpm     =  1500;

    p->i_rect_sat_margin_cA= 1000;    /* within 10 A of i_rect_max */
    p->theta_sat_x100      = 9500;    /* within 5 % of full throttle */
    p->i_bat_ref_boost_cA  =  1000;   /* +10 A on saturation */
}

/* Placeholder LUT — linear in P_rect until bench-calibrated.
 * Caps duty at 95 % and theta at 100 %.
 *
 *   P_rect [W]  ->  duty [x10000],  theta [pct_x100]
 *      0..4500 W  linear ramp to (9500, 10000)
 *
 * Replace the body with a piecewise table from a dyno sweep. */
#define V1_LUT_P_RECT_MAX_W   4500u
void control_law_v1_lut(uint32_t  p_rect_W,
                        uint16_t *duty_x10000,
                        uint16_t *theta_pct_x100) {
    uint32_t p = p_rect_W > V1_LUT_P_RECT_MAX_W ? V1_LUT_P_RECT_MAX_W : p_rect_W;
    *duty_x10000    = (uint16_t)((p * 9500u)  / V1_LUT_P_RECT_MAX_W);
    *theta_pct_x100 = (uint16_t)((p * 10000u) / V1_LUT_P_RECT_MAX_W);
}

void control_law_v1_step(ctl_v1_state_t *st,
                         const ctl_v1_inputs_t *in,
                         const ctl_v1_params_t *p,
                         ctl_v1_output_t *out) {
    /* ---- I_bat EMA ---- */
    if (!st->init_done) {
        st->i_bat_filt_cA = in->i_bat_meas_cA;
        st->i_bat_prev_cA = in->i_bat_meas_cA;
        st->init_done     = true;
    }
    int32_t err_in = (int32_t)in->i_bat_meas_cA - (int32_t)st->i_bat_filt_cA;
    int32_t adj    = (err_in * (int32_t)p->i_bat_alpha_q15) >> 15;
    int32_t filt   = (int32_t)st->i_bat_filt_cA + adj;
    filt = clamp_i32(filt, INT16_MIN, INT16_MAX);
    st->i_bat_filt_cA = (int16_t)filt;

    /* Effective I_bat_ref: boost on rectifier saturation. */
    int16_t i_bat_ref_eff = p->i_bat_ref_cA;
    if (st->i_rect_demand_cA >
        (int32_t)p->i_rect_max_cA - (int32_t)p->i_rect_sat_margin_cA) {
        i_bat_ref_eff = (int16_t)clamp_i32(
            (int32_t)i_bat_ref_eff + (int32_t)p->i_bat_ref_boost_cA,
            INT16_MIN, INT16_MAX);
    }

    /* ---- PID on I_bat error (filtered meas - ref). ---- *
     * Sign: i_bat_meas is +discharge. If meas > ref, battery is draining too
     * fast -> need MORE rectifier current -> positive demand adjustment.   */
    int32_t err  = (int32_t)st->i_bat_filt_cA - (int32_t)i_bat_ref_eff;
    int32_t p_term = (err * p->kp_q15) >> 15;

    int32_t i_term_new = st->i_term_cA + ((err * p->ki_q15) >> 15);
    i_term_new = clamp_i32(i_term_new, -p->i_term_max_cA, p->i_term_max_cA);
    st->i_term_cA = i_term_new;

    int32_t d_in  = (int32_t)st->i_bat_filt_cA - (int32_t)st->i_bat_prev_cA;
    int32_t d_term = (d_in * p->kd_q15) >> 15;
    st->i_bat_prev_cA = st->i_bat_filt_cA;

    int32_t pid_out_cA = p_term + i_term_new + d_term;
    /* Output is an ABSOLUTE demand, not a delta — bias by last tick. */
    int32_t target = (int32_t)st->i_rect_demand_cA + pid_out_cA;

    /* ---- Anti-stall RPM clamp. ---- */
    if (in->gen_rpm < p->omega_floor_rpm) {
        target = 0;
    } else if (in->gen_rpm < p->omega_min_rpm) {
        /* Linear derate omega_floor..omega_min. */
        uint32_t span    = (uint32_t)(p->omega_min_rpm - p->omega_floor_rpm);
        uint32_t over    = (uint32_t)(in->gen_rpm     - p->omega_floor_rpm);
        int32_t  ceil_cA = (int32_t)(((uint32_t)p->i_rect_max_cA * over) / (span ? span : 1u));
        if (target > ceil_cA) target = ceil_cA;
    }

    /* ---- Hard clamp + slew limit. ---- */
    target = clamp_i32(target, 0, p->i_rect_max_cA);

    int32_t delta = target - (int32_t)st->i_rect_demand_cA;
    if (delta >  p->i_rect_slew_cA) delta =  p->i_rect_slew_cA;
    if (delta < -p->i_rect_slew_cA) delta = -p->i_rect_slew_cA;
    st->i_rect_demand_cA = (int16_t)((int32_t)st->i_rect_demand_cA + delta);

    /* ---- P_rect = I_rect_demand [cA] * V_bus [cV] / 10000 -> W. ---- *
     * (cA * cV) / 10000 = (0.01 A * 0.01 V) / 10000 = W. Signed I, but
     * clamp >= 0 before mapping (LUT undefined for regen). */
    int32_t i_for_p = st->i_rect_demand_cA < 0 ? 0 : (int32_t)st->i_rect_demand_cA;
    uint32_t p_rect_W = (uint32_t)((i_for_p * (int32_t)in->v_bus_cV) / 10000);

    uint16_t duty_x10000, theta_pct_x100;
    control_law_v1_lut(p_rect_W, &duty_x10000, &theta_pct_x100);

    /* Throttle-saturation safety boost feeds back next tick (loop closure). */
    (void)p->theta_sat_x100;

    out->i_rect_demand_cA      = st->i_rect_demand_cA;
    out->duty_x10000           = duty_x10000;
    out->theta_engine_pct_x100 = theta_pct_x100;
    out->p_rect_W              = p_rect_W;
    out->i_bat_ref_eff_cA      = i_bat_ref_eff;
}
