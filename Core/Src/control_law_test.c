#include "control_law.h"
#include "control_law_test.h"
#include "main.h"

volatile int control_law_test_failed_id = 0;

#define TEST(cond, id)                                  \
    do {                                                \
        if (!(cond)) {                                  \
            control_law_test_failed_id = (id);          \
            Error_Handler();                            \
        }                                               \
    } while (0)

void control_law_self_test(void) {
    ctl_state_t  st;
    ctl_params_t p;
    ctl_inputs_t in;

    /* ---- T1..T3: TAKEOFF/CLIMB/LAND pin to peak ---- */
    control_law_init(&st);
    control_law_default_params(&p);
    in.throttle_dem_pct = 5000;
    in.soc_pct          = 5000;

    in.mode = VESC_MODE_TAKEOFF;
    TEST(control_law_step(&st, &in, &p) == p.I_rect_peak_cA, 1);

    in.mode = VESC_MODE_CLIMB;
    TEST(control_law_step(&st, &in, &p) == p.I_rect_peak_cA, 2);

    in.mode = VESC_MODE_LAND;
    TEST(control_law_step(&st, &in, &p) == p.I_rect_peak_cA, 3);

    /* ---- T4..T5: IDLE/FAULT zero ---- */
    in.mode = VESC_MODE_IDLE;
    TEST(control_law_step(&st, &in, &p) == 0, 4);

    in.mode = VESC_MODE_FAULT;
    TEST(control_law_step(&st, &in, &p) == 0, 5);

    /* ---- T6: CRUISE deadband holds setpoint ---- */
    control_law_init(&st);
    control_law_default_params(&p);
    in.mode             = VESC_MODE_CRUISE;
    in.throttle_dem_pct = 5000;
    in.soc_pct          = 5000;
    for (int i = 0; i < 500; ++i) (void)control_law_step(&st, &in, &p);
    int16_t held = st.I_rect_cmd_cA;
    in.throttle_dem_pct = 5100;
    TEST(control_law_step(&st, &in, &p) == held, 6);

    /* ---- T7: CRUISE outside deadband trims by trim_step ---- */
    control_law_init(&st);
    control_law_default_params(&p);
    in.mode             = VESC_MODE_CRUISE;
    in.throttle_dem_pct = 0;
    in.soc_pct          = 5000;
    for (int i = 0; i < 200; ++i) (void)control_law_step(&st, &in, &p);
    int16_t before = st.I_rect_cmd_cA;
    in.throttle_dem_pct = 5000;
    int16_t after = control_law_step(&st, &in, &p);
    TEST(after == before + p.trim_step_cA, 7);

    /* ---- T8: CRUISE no SOC bias, target = I_load_full*dem ---- */
    control_law_init(&st);
    control_law_default_params(&p);
    p.trim_step_cA       = 30000;
    in.mode              = VESC_MODE_CRUISE;
    in.throttle_dem_pct  = 5000;
    in.soc_pct           = 5000;
    st.throttle_filt_pct = 0;
    st.I_rect_cmd_cA     = 0;
    TEST(control_law_step(&st, &in, &p) == 5750, 8);

    /* ---- T9: CRUISE with SOC bias adds soc_bias_cA ---- */
    control_law_init(&st);
    control_law_default_params(&p);
    p.trim_step_cA       = 30000;
    in.mode              = VESC_MODE_CRUISE;
    in.throttle_dem_pct  = 5000;
    in.soc_pct           = 1000;
    st.throttle_filt_pct = 0;
    st.I_rect_cmd_cA     = 0;
    TEST(control_law_step(&st, &in, &p) == 7750, 9);

    /* ---- T10: EMA filter converges toward demand ---- */
    control_law_init(&st);
    control_law_default_params(&p);
    in.mode             = VESC_MODE_CRUISE;
    in.throttle_dem_pct = 7500;
    in.soc_pct          = 5000;
    for (int i = 0; i < 1000; ++i) (void)control_law_step(&st, &in, &p);
    /* Q15 rounding deadzone keeps filter ~5-15 counts shy of dem. */
    TEST(st.throttle_filt_pct >= 7400 &&
         st.throttle_filt_pct <= 7500, 10);

    control_law_test_failed_id = 0;
}
