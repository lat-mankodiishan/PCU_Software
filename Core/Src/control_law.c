#include "control_law.h"
#include "powertrain_state.h"
#include <stdint.h>

void control_law_init(ctl_state_t *st) {
    st->throttle_filt_pct = 0;
    st->I_rect_cmd_cA     = 0;
}

void control_law_default_params(ctl_params_t *p) {
    p->I_rect_peak_cA        = 12000;    /* 120.00 A peak in transient modes */
    p->I_load_full_cA        = 11500;    /* 115.00 A at full throttle  */
    p->deadband_pct          =   500;    /* 5.00 % */
    p->trim_step_cA          =    50;    /* 0.50 A/tick = 50 A/s at 100 Hz */
    p->soc_low_threshold_pct =  3000;    /* 30 % */
    p->soc_bias_cA           =  2000;    /* +20.00 A while SOC < threshold */
    p->throttle_alpha_q15    =  2000;    /* ~6 %/tick → ~1 Hz cutoff at 10 ms */

    p->ibat_kp_q15           =  6553;    /* ~0.2 in Q15: 1 A error → 0.2 A adj */
    p->ibat_deadband_cA      =    30;    /* ±0.3 A; ~3× Daly current LSB */
}

static inline int32_t clamp_i32(int32_t v, int32_t lo, int32_t hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

int16_t control_law_step(ctl_state_t *st,
                         const ctl_inputs_t *in,
                         const ctl_params_t *p) {
    /* ---- EMA filter on throttle (Q15) ---- */
    int32_t err_pct = (int32_t)in->throttle_dem_pct - (int32_t)st->throttle_filt_pct;
    int32_t adj     = (err_pct * (int32_t)p->throttle_alpha_q15) >> 15;
    int32_t filt    = clamp_i32((int32_t)st->throttle_filt_pct + adj, 0, 10000);
    st->throttle_filt_pct = (uint16_t)filt;

    int32_t out_cA;

    switch (in->mode) {
    case VESC_MODE_TAKEOFF:
    case VESC_MODE_CLIMB:
    case VESC_MODE_LAND:
        /* Battery is swing source; rectifier pinned at peak. */
        out_cA = p->I_rect_peak_cA;
        break;

    case VESC_MODE_CRUISE: {
        /* HP-filtered throttle: small transients ride on battery. */
        int32_t hp = (int32_t)in->throttle_dem_pct - (int32_t)st->throttle_filt_pct;
        int32_t hp_abs = hp < 0 ? -hp : hp;

        if ((uint32_t)hp_abs >= p->deadband_pct) {
            int32_t target_cA =
                ((int32_t)p->I_load_full_cA * (int32_t)in->throttle_dem_pct) / 10000;
            if (in->soc_pct < p->soc_low_threshold_pct) {
                target_cA += p->soc_bias_cA;
            }
            int32_t cur = st->I_rect_cmd_cA;
            if (target_cA > cur) {
                cur += p->trim_step_cA;
                if (cur > target_cA) cur = target_cA;
            } else if (target_cA < cur) {
                cur -= p->trim_step_cA;
                if (cur < target_cA) cur = target_cA;
            }
            out_cA = cur;
        } else {
            out_cA = st->I_rect_cmd_cA; /* hold inside deadband */
        }
        break;
    }

    case VESC_MODE_IDLE:
    case VESC_MODE_FAULT:
    default:
        out_cA = 0;
        break;
    }

    out_cA = clamp_i32(out_cA, INT16_MIN, INT16_MAX);
    st->I_rect_cmd_cA = (int16_t)out_cA;
    return st->I_rect_cmd_cA;
}

int16_t control_law_step_ibat(ctl_state_t *st,
                              int16_t i_bat_cA,
                              const ctl_params_t *p) {
    /* bms_i_bat_cA: +discharge. Adj sign matches i_bat directly. */
    int32_t i_abs = i_bat_cA < 0 ? -(int32_t)i_bat_cA : (int32_t)i_bat_cA;
    if (i_abs < (int32_t)p->ibat_deadband_cA) {
        return st->I_rect_cmd_cA;
    }

    int32_t adj = ((int32_t)i_bat_cA * (int32_t)p->ibat_kp_q15) >> 15;

    int32_t slew = (int32_t)p->trim_step_cA;
    if (adj >  slew) adj =  slew;
    if (adj < -slew) adj = -slew;

    int32_t cmd = (int32_t)st->I_rect_cmd_cA + adj;
    cmd = clamp_i32(cmd, 0, I_RECT_MAX_CA);

    st->I_rect_cmd_cA = (int16_t)cmd;
    return st->I_rect_cmd_cA;
}
