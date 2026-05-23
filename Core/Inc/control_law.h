#ifndef CONTROL_LAW_H
#define CONTROL_LAW_H

#include <stdint.h>
#include "vesc_proto.h"

/* Percent fields: 0.01 %/LSB (0..10000 = 0..100 %). */

typedef struct {
    int16_t  I_rect_peak_cA;          /* TAKEOFF/CLIMB/LAND */
    int16_t  I_load_full_cA;          /* at 100% throttle */
    uint16_t deadband_pct;
    int16_t  trim_step_cA;            /* slew per tick */
    uint16_t soc_low_threshold_pct;
    int16_t  soc_bias_cA;
    uint16_t throttle_alpha_q15;      /* EMA alpha Q15 */
    uint16_t ibat_kp_q15;             /* I_bat P gain Q15 */
    int16_t  ibat_deadband_cA;
} ctl_params_t;

typedef struct {
    uint16_t throttle_filt_pct;       /* EMA state */
    int16_t  I_rect_cmd_cA;           /* last setpoint */
} ctl_state_t;

typedef struct {
    vesc_mode_t mode;
    uint16_t    throttle_dem_pct;     /* from FC */
    uint16_t    soc_pct;              /* from BMS */
} ctl_inputs_t;

void    control_law_init(ctl_state_t *st);
void    control_law_default_params(ctl_params_t *p);

int16_t control_law_step(ctl_state_t *st,
                         const ctl_inputs_t *in,
                         const ctl_params_t *p);

/* Gate calls on bms_input_tick; Daly updates ~20 Hz. */
int16_t control_law_step_ibat(ctl_state_t *st,
                              int16_t i_bat_cA,
                              const ctl_params_t *p);

#endif /* CONTROL_LAW_H */
