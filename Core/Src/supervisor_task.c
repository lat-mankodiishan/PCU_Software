#include "supervisor_task.h"
#include "control_law.h"
#include "powertrain_state.h"
#include "periph_wrappers.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#define SUP_PERIOD_MS           200        /* 5 Hz, matches sensor_task */

/* Crank duty + CRANK->RUN motor-swap timings; copied from expt sweep profile. */
#define CRANK_BLDC_DUTY_X10000   4200      /* 42.00 % */
#define CRANK_THROTTLE_PCT_X100  4500      /* 45.00 % engine throttle while cranking */
#define SWAP_BLEED_MS             300      /* BLDC I=0 before motor_type swap */
#define SWAP_LOCK_MS             1000      /* FOC inverted I=0 observer lock */
#define SWAP_PRIME_MS             500      /* open-loop hold before V1 takes over */
#define PRIME_DUTY_X10000        6000      /* 60.00 % FOC inverted duty during PRIME */
#define PRIME_THROTTLE_PCT_X100  6000      /* 60.00 % engine throttle during PRIME */

/* Flip to 1 to put V1 controller back on the wire during ENGINE_RUN. */
#define RUN_USE_V1_CONTROLLER    0
#define RUN_DUTY_X10000          7500      /* 75.00 % fixed FOC duty in RUN */
#define RUN_THROTTLE_PCT_X100    6000      /* 60.00 % fixed engine throttle in RUN */

/* Fault detection thresholds (supervisor-owned faults). */
#define FC_THROTTLE_STALE_MS         2500u
#define CURRENT_SENSOR_STALE_MS      500u
#define RECT_OVERTEMP_C              85
#define BUS_UNDERVOLT_CV             4000u   /* 40.00 V — TUNE FOR BATTERY */
#define BUS_OVERVOLT_CV              6000u   /* 60.00 V — TUNE FOR BATTERY */
#define ENGINE_STALL_MIN_RPM         1500u
#define ENGINE_STALL_MS              2000u

/* Per-bit severity policy. Sorted by bit index 0..15. */
typedef enum {
    SEV_NONE        = 0,
    SEV_WARN        = 1,
    SEV_DEGRADE     = 2,
    SEV_FORCE_OFF   = 3,
    SEV_FORCE_FAULT = 4,
} fault_severity_t;

/* BENCH MODE: every fault demoted to WARN so detection runs (bits set in
 * NodeStatus + visible in MP / Live Watch) but no action is taken. Restore
 * per-bit severities below before flight. Intended-for-flight severity in the
 * trailing comment of each line. */
static const uint8_t s_fault_severity[16] = {
    [0]  = SEV_WARN,         /* FAULT_RECT_STALE            -> DEGRADE */
    [1]  = SEV_WARN,         /* FAULT_RECT_OFFLINE          -> FORCE_FAULT */
    [2]  = SEV_WARN,         /* FAULT_FC_STALE              -> FORCE_OFF */
    [3]  = SEV_WARN,         /* FAULT_BUS1_BUSOFF           -> FORCE_FAULT */
    [4]  = SEV_WARN,         /* FAULT_BUS2_BUSOFF           -> FORCE_FAULT */
    [5]  = SEV_WARN,         /* FAULT_SUPERVISOR_HANG       -> FORCE_FAULT */
    [6]  = SEV_NONE,         /* reserved */
    [7]  = SEV_WARN,         /* FAULT_ECU_STALE             -> WARN (same) */
    [8]  = SEV_WARN,         /* FAULT_FC_THROTTLE_STALE     -> FORCE_OFF */
    [9]  = SEV_WARN,         /* FAULT_CURRENT_SENSOR_STALE  -> FORCE_OFF */
    [10] = SEV_WARN,         /* FAULT_TC_FAULT              -> WARN (same) */
    [11] = SEV_WARN,         /* FAULT_ENGINE_STALL          -> FORCE_OFF */
    [12] = SEV_WARN,         /* FAULT_ENGINE_OVERTEMP       -> FORCE_OFF */
    [13] = SEV_WARN,         /* FAULT_RECT_OVERTEMP         -> FORCE_OFF */
    [14] = SEV_WARN,         /* FAULT_BUS_UNDERVOLT         -> FORCE_OFF */
    [15] = SEV_WARN,         /* FAULT_BUS_OVERVOLT          -> FORCE_FAULT */
};

static fault_severity_t worst_severity(uint16_t f) {
    fault_severity_t worst = SEV_NONE;
    for (uint8_t i = 0; i < 16; i++) {
        if ((f & (1u << i)) && s_fault_severity[i] > worst) {
            worst = (fault_severity_t)s_fault_severity[i];
        }
    }
    return worst;
}

/* GPIO switch fires CRANK->RUN edge; same PA7 as expt button (mutually
 * exclusive contexts — expt_active gates supervisor out of the wire). */
#define ENGINE_RUN_SWITCH_PORT   GPIOA
#define ENGINE_RUN_SWITCH_PIN    GPIO_PIN_7

/* Sub-state used only during CRANK->RUN transition; not exposed in engine_state. */
typedef enum {
    SWAP_IDLE  = 0,
    SWAP_BLEED = 1,
    SWAP_LOCK  = 2,
    SWAP_PRIME = 3,
} swap_phase_t;

static StaticTask_t s_tcb;
static StackType_t  s_stack[256];

#if RUN_USE_V1_CONTROLLER
static ctl_v1_state_t  s_v1_state;
static ctl_v1_params_t s_v1_params;
#endif

static engine_state_t s_prev_engine_state;
static swap_phase_t   s_swap_phase;
static uint32_t       s_swap_tick;
static bool           s_switch_prev;

static void supervisor_task(void *arg);

void supervisor_task_start(void) {
#if RUN_USE_V1_CONTROLLER
    control_law_v1_init(&s_v1_state);
    control_law_v1_default_params(&s_v1_params);
#endif
    s_prev_engine_state = ENGINE_OFF;
    s_swap_phase        = SWAP_IDLE;
    s_swap_tick         = 0;
    s_switch_prev       = false;

    static const osThreadAttr_t tattr = {
        .name       = "supervisor",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityHigh,
    };
    osThreadNew(supervisor_task, NULL, &tattr);
}

static void safe_idle(void) {
    /* OFF/FAULT: duty=0 + throttle=0. */
    pt_set_setpoint(0, MODE_IDLE);
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.engine_throttle.req_pct_x100 = 0;
    osMutexRelease(g_pt_mtx);
}

static void crank_hold(void) {
    pt_set_motor_type(VESC_MOTOR_TYPE_BLDC);
    pt_set_invert_direction(false);
    pt_set_setpoint_duty(CRANK_BLDC_DUTY_X10000, MODE_TAKEOFF);
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.engine_throttle.req_pct_x100 = CRANK_THROTTLE_PCT_X100;
    osMutexRelease(g_pt_mtx);
}

/* Returns true while a swap is in progress (it owns the wire this tick). */
static bool swap_step(uint32_t now) {
    const uint32_t elapsed = now - s_swap_tick;
    switch (s_swap_phase) {
    case SWAP_BLEED:
        /* BLDC I=0; avoids body-diode regen during motor_type swap. */
        pt_set_setpoint(0, MODE_TAKEOFF);
        if (elapsed >= SWAP_BLEED_MS) {
            pt_set_motor_type(VESC_MOTOR_TYPE_FOC);
            pt_set_invert_direction(true);
            s_swap_phase = SWAP_LOCK;
            s_swap_tick  = now;
        }
        return true;

    case SWAP_LOCK:
        /* FOC inverted I=0; observer locks before duty is applied. */
        pt_set_setpoint(0, MODE_TAKEOFF);
        if (elapsed >= SWAP_LOCK_MS) {
            s_swap_phase = SWAP_PRIME;
            s_swap_tick  = now;
        }
        return true;

    case SWAP_PRIME:
        /* Open-loop prime at PRIME_DUTY/PRIME_THROTTLE before V1 takes over. */
        pt_set_setpoint_duty(PRIME_DUTY_X10000, MODE_TAKEOFF);
        osMutexAcquire(g_pt_mtx, osWaitForever);
        g_pt.engine_throttle.req_pct_x100 = PRIME_THROTTLE_PCT_X100;
        osMutexRelease(g_pt_mtx);
        if (elapsed >= SWAP_PRIME_MS) {
            s_swap_phase = SWAP_IDLE;
            pt_set_engine_state(ENGINE_RUN);
            osMutexAcquire(g_pt_mtx, osWaitForever);
            g_pt.engine.req = ENGINE_STATE_REQ_NONE;
            osMutexRelease(g_pt_mtx);
        }
        return true;

    case SWAP_IDLE:
    default:
        return false;
    }
}

static void on_run_entry(void) {
    pt_set_motor_type(VESC_MOTOR_TYPE_FOC);
    pt_set_invert_direction(true);
#if RUN_USE_V1_CONTROLLER
    control_law_v1_init(&s_v1_state);
#endif
}

#if RUN_USE_V1_CONTROLLER
static void run_v1(uint32_t now) {
    (void)now;
    ctl_v1_inputs_t in;
    osMutexAcquire(g_pt_mtx, osWaitForever);
    /* ACS ch2 = I_bat (mA, +discharge). cA = mA/10. */
    in.i_bat_meas_cA  = (int16_t)(g_pt.acs.mA[2] / 10);
    in.v_bus_cV       = g_pt.rect.state.V_dc_cV;
    in.gen_rpm        = g_pt.rect.state.gen_rpm;
    osMutexRelease(g_pt_mtx);

    ctl_v1_output_t out;
    control_law_v1_step(&s_v1_state, &in, &s_v1_params, &out);

    pt_set_setpoint_duty((int16_t)out.duty_x10000, MODE_CRUISE);

    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.engine_throttle.req_pct_x100 = out.theta_engine_pct_x100;
    g_pt.ctl.i_bat_filt_cA            = s_v1_state.i_bat_filt_cA;
    g_pt.ctl.i_bat_ref_eff_cA         = out.i_bat_ref_eff_cA;
    g_pt.ctl.i_rect_demand_cA         = out.i_rect_demand_cA;
    g_pt.ctl.p_rect_W                 = out.p_rect_W;
    g_pt.ctl.duty_x10000              = out.duty_x10000;
    g_pt.ctl.theta_pct_x100           = out.theta_engine_pct_x100;
    osMutexRelease(g_pt_mtx);
}
#else
/* Open-loop fixed operating point: bench bring-up before V1 is trusted. */
static void run_fixed_setpoint(void) {
    pt_set_setpoint_duty(RUN_DUTY_X10000, MODE_CRUISE);
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.engine_throttle.req_pct_x100 = RUN_THROTTLE_PCT_X100;
    g_pt.ctl.duty_x10000              = RUN_DUTY_X10000;
    g_pt.ctl.theta_pct_x100           = RUN_THROTTLE_PCT_X100;
    osMutexRelease(g_pt_mtx);
}
#endif

/* Detect supervisor-owned faults: stale ticks + range checks + CAN bus-off.
 * Per-task faults (RECT_STALE/OFFLINE, FC_STALE, ECU_STALE, SUPERVISOR_HANG,
 * TC_FAULT, ENGINE_OVERTEMP) are raised in their owning tasks. */
static void detect_faults(uint32_t now, engine_state_t engine_state) {
    uint32_t fc_tick, acs_tick, rect_tick;
    uint16_t v_dc_cV;
    int16_t  igbt_temp_C;
    uint16_t ecu_rpm;
    osMutexAcquire(g_pt_mtx, osWaitForever);
    fc_tick     = g_pt.fc.tick;
    acs_tick    = g_pt.acs.tick;
    rect_tick   = g_pt.rect.tick;
    v_dc_cV     = g_pt.rect.state.V_dc_cV;
    igbt_temp_C = g_pt.rect.state.igbt_temp_C;
    ecu_rpm     = g_pt.ecu.rpm;
    osMutexRelease(g_pt_mtx);

    /* FC throttle staleness — distinct from NodeStatus stale. */
    if (fc_tick && (now - fc_tick) > FC_THROTTLE_STALE_MS) pt_set_fault(FAULT_FC_THROTTLE_STALE);
    else if (fc_tick)                                      pt_clear_fault(FAULT_FC_THROTTLE_STALE);

    /* Current sensor staleness. */
    if (acs_tick && (now - acs_tick) > CURRENT_SENSOR_STALE_MS) pt_set_fault(FAULT_CURRENT_SENSOR_STALE);
    else if (acs_tick)                                          pt_clear_fault(FAULT_CURRENT_SENSOR_STALE);

    /* Bus over/under volt and rect overtemp only checked when rect data fresh. */
    if (rect_tick && (now - rect_tick) < 500u) {
        if (v_dc_cV > BUS_OVERVOLT_CV)  pt_set_fault(FAULT_BUS_OVERVOLT);
        else                            pt_clear_fault(FAULT_BUS_OVERVOLT);
        if (v_dc_cV < BUS_UNDERVOLT_CV) pt_set_fault(FAULT_BUS_UNDERVOLT);
        else                            pt_clear_fault(FAULT_BUS_UNDERVOLT);
        if (igbt_temp_C > RECT_OVERTEMP_C) pt_set_fault(FAULT_RECT_OVERTEMP);
        else                                pt_clear_fault(FAULT_RECT_OVERTEMP);
    }

    /* Engine stall — sustained low RPM while in RUN. */
    static uint32_t s_low_rpm_first = 0u;
    if (engine_state == ENGINE_RUN && ecu_rpm < ENGINE_STALL_MIN_RPM) {
        if (s_low_rpm_first == 0u) s_low_rpm_first = now;
        if ((now - s_low_rpm_first) > ENGINE_STALL_MS) pt_set_fault(FAULT_ENGINE_STALL);
    } else {
        s_low_rpm_first = 0u;
        pt_clear_fault(FAULT_ENGINE_STALL);
    }

    /* CAN bus-off — ESR.BOFF polled; auto-recovery clears it on its own. */
    if (can_hw_is_busoff(CAN_BUS_DRONECAN)) pt_set_fault(FAULT_BUS1_BUSOFF);
    else                                     pt_clear_fault(FAULT_BUS1_BUSOFF);
    if (can_hw_is_busoff(CAN_BUS_ENGINE))   pt_set_fault(FAULT_BUS2_BUSOFF);
    else                                     pt_clear_fault(FAULT_BUS2_BUSOFF);
}

static void supervisor_task(void *arg) {
    (void)arg;
    uint32_t next = osKernelGetTickCount();

    for (;;) {
        const uint32_t now = osKernelGetTickCount();

        engine_state_t engine_state;
        engine_state_t engine_state_req;
        uint16_t       faults;
        bool           expt_active;

        osMutexAcquire(g_pt_mtx, osWaitForever);
        engine_state     = g_pt.engine.state;
        engine_state_req = g_pt.engine.req;
        faults           = g_pt.fault_bits;
        expt_active      = g_pt.expt.active;
        osMutexRelease(g_pt_mtx);

        /* Detect supervisor-owned faults, then re-read fault_bits and apply policy. */
        detect_faults(now, engine_state);
        osMutexAcquire(g_pt_mtx, osWaitForever);
        faults = g_pt.fault_bits;
        osMutexRelease(g_pt_mtx);
        const fault_severity_t sev = worst_severity(faults);

        /* Fault policy enforcement — must run before request handling so a
         * FORCE_OFF / FORCE_FAULT supersedes any pending CRANK/RUN request. */
        if (sev == SEV_FORCE_FAULT && engine_state != ENGINE_FAULT) {
            s_swap_phase = SWAP_IDLE;
            pt_set_engine_state(ENGINE_FAULT);
            engine_state = ENGINE_FAULT;
            osMutexAcquire(g_pt_mtx, osWaitForever);
            g_pt.engine.req = ENGINE_STATE_REQ_NONE;
            osMutexRelease(g_pt_mtx);
            engine_state_req = ENGINE_STATE_REQ_NONE;
        } else if (sev == SEV_FORCE_OFF
                   && engine_state != ENGINE_OFF
                   && engine_state != ENGINE_FAULT) {
            s_swap_phase = SWAP_IDLE;
            pt_set_engine_state(ENGINE_OFF);
            engine_state = ENGINE_OFF;
            osMutexAcquire(g_pt_mtx, osWaitForever);
            g_pt.engine.req = ENGINE_STATE_REQ_NONE;
            osMutexRelease(g_pt_mtx);
            engine_state_req = ENGINE_STATE_REQ_NONE;
        }

        /* External request: RUN goes via swap (CRANK only); others apply direct. */
        if (engine_state_req != ENGINE_STATE_REQ_NONE
            && engine_state_req != engine_state) {
            if (engine_state_req == ENGINE_RUN) {
                if (engine_state == ENGINE_CRANK && s_swap_phase == SWAP_IDLE) {
                    s_swap_phase = SWAP_BLEED;
                    s_swap_tick  = now;
                }
            } else {
                s_swap_phase = SWAP_IDLE;
                pt_set_engine_state(engine_state_req);
                engine_state = engine_state_req;
                osMutexAcquire(g_pt_mtx, osWaitForever);
                g_pt.engine.req = ENGINE_STATE_REQ_NONE;
                osMutexRelease(g_pt_mtx);
            }
        } else if (engine_state_req == engine_state) {
            osMutexAcquire(g_pt_mtx, osWaitForever);
            g_pt.engine.req = ENGINE_STATE_REQ_NONE;
            osMutexRelease(g_pt_mtx);
        }

        const bool degrade = (sev >= SEV_DEGRADE);
        const bool switch_now =
            HAL_GPIO_ReadPin(ENGINE_RUN_SWITCH_PORT, ENGINE_RUN_SWITCH_PIN)
            == GPIO_PIN_SET;

        /* ---- engine_state transition entries ---- */
        if (engine_state != s_prev_engine_state) {
            if (engine_state == ENGINE_RUN && s_prev_engine_state != ENGINE_RUN) {
                on_run_entry();
            }
            if (engine_state == ENGINE_CRANK) {
                /* Re-arm: require a fresh rising edge from THIS state. */
                s_switch_prev = switch_now;
            }
            if (engine_state != ENGINE_CRANK && engine_state != ENGINE_RUN) {
                /* Operator backed out mid-swap — drop it. */
                s_swap_phase = SWAP_IDLE;
            }
            s_prev_engine_state = engine_state;
        }

        /* Rising-edge detect AFTER re-arm, BEFORE updating prev. */
        const bool switch_rising = switch_now && !s_switch_prev;
        s_switch_prev = switch_now;

        /* GPIO switch fires the CRANK -> RUN transition (via swap sequence). */
        if (!expt_active &&
            engine_state    == ENGINE_CRANK &&
            s_swap_phase    == SWAP_IDLE    &&
            switch_rising) {
            s_swap_phase = SWAP_BLEED;
            s_swap_tick  = now;
        }

        /* ---- Dispatch ---- */
        if (expt_active) {
            /* expt_run() owns the wire (bench mode); supervisor steps aside. */
        } else if (degrade) {
            safe_idle();
        } else if (swap_step(now)) {
            /* swap drove the wire this tick. */
        } else {
            switch (engine_state) {
            case ENGINE_CRANK:
                crank_hold();
                break;
            case ENGINE_RUN:
#if RUN_USE_V1_CONTROLLER
                run_v1(now);
#else
                run_fixed_setpoint();
#endif
                break;
            case ENGINE_WARMUP:
            case ENGINE_COOLDOWN:
                /* Engine running, no rectifier load; operator controls throttle. */
                pt_set_setpoint(0, MODE_IDLE);
                break;
            case ENGINE_OFF:
            case ENGINE_FAULT:
            default:
                safe_idle();
                break;
            }
        }

        /* Contactor policy: battery opens only on ENGINE_FAULT (covers FORCE_FAULT
         * severity since policy already sets state to ENGINE_FAULT). Rectifier
         * additionally opens on any DEGRADE-or-worse severity. */
        const bool engine_fault = (engine_state == ENGINE_FAULT);
        const bool batt_close   = !engine_fault;
        const bool rect_close   = !engine_fault && !degrade;
        pt_set_contactor_cmds(batt_close, rect_close);

        /* Mirror engine_throttle_req -> PWM. In RUN, V1 wrote req; elsewhere
         * operator/Live Watch owns it. */
        uint16_t eng_req;
        osMutexAcquire(g_pt_mtx, osWaitForever);
        eng_req = g_pt.engine_throttle.req_pct_x100;
        osMutexRelease(g_pt_mtx);
        pt_set_engine_throttle_pct_x100(eng_req);

        osMutexAcquire(g_pt_mtx, osWaitForever);
        g_pt.supervisor_heartbeat++;
        osMutexRelease(g_pt_mtx);

        watchdog_refresh();

        next += SUP_PERIOD_MS;
        osDelayUntil(next);
    }
}
