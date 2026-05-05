#include "supervisor_task.h"
#include "control_law.h"
#include "powertrain_state.h"
#include "periph_wrappers.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#define SUP_PERIOD_MS       10        /* 100 Hz */

/* Bench bring-up: closed-loop on bms_i_bat_cA to hold battery current at 0,
 * letting the rectifier cover all load. Set to 0 to fall back to the
 * flight-mode law (throttle + flight_state + soc). */
#define USE_IBAT_CONTROL_LAW   1

static StaticTask_t  s_tcb;
static StackType_t   s_stack[256];    /* 1 KB */

static ctl_state_t   s_ctl_state;
static ctl_params_t  s_ctl_params;

static void supervisor_task(void *arg);

void supervisor_task_start(void) {
    control_law_init(&s_ctl_state);
    control_law_default_params(&s_ctl_params);

    static const osThreadAttr_t tattr = {
        .name       = "supervisor",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityHigh,             /* prio 5 */
    };
    osThreadNew(supervisor_task, NULL, &tattr);
}

static void supervisor_task(void *arg) {
    (void)arg;
    uint32_t next = osKernelGetTickCount();
    uint32_t last_bms_tick = 0;

    for (;;) {
        /* --- Snapshot inputs under the mutex ------------------------- */
        ctl_inputs_t in;
        uint16_t     faults;
        int16_t      bms_i_bat_cA;
        uint32_t     bms_tick;
        bool         expt_active;
        int16_t      expt_I_cmd_cA;

        osMutexAcquire(g_pt_mtx, osWaitForever);
        in.mode             = g_pt.fc_flight_state;
        in.throttle_dem_pct = g_pt.fc_throttle_dem_pct;
        in.soc_pct          = g_pt.bms_soc_pct;
        faults              = g_pt.fault_bits;
        bms_i_bat_cA        = g_pt.bms_i_bat_cA;
        bms_tick            = g_pt.bms_input_tick;
        expt_active         = g_pt.expt_active;
        expt_I_cmd_cA       = g_pt.I_rect_cmd_cA;
        osMutexRelease(g_pt_mtx);

        /* --- Mode-override on faults --------------------------------- */
        vesc_mode_t final_mode = in.mode;
        if (faults & (FAULT_RECT_OFFLINE | FAULT_BUS2_BUSOFF)) {
            final_mode = VESC_MODE_FAULT;
        }
#if USE_IBAT_CONTROL_LAW
        /* No BMS reading = no measurement = no closed-loop. */
        if (faults & FAULT_BMS_STALE) {
            final_mode = VESC_MODE_FAULT;
        }
#endif
        in.mode = final_mode;

        /* --- Run control law ----------------------------------------- *
         * When an experiment is active, it owns the setpoint. We slave the
         * I_bat integrator to the experiment's command so resuming the
         * closed-loop after the experiment ends starts from a consistent
         * state, and we skip the publish step so we don't overwrite the
         * experiment's mode. */
        int16_t I_cmd;
#if USE_IBAT_CONTROL_LAW
        if (expt_active) {
            s_ctl_state.I_rect_cmd_cA = expt_I_cmd_cA;
        } else if (final_mode == VESC_MODE_FAULT) {
            s_ctl_state.I_rect_cmd_cA = 0;          /* reset integrator */
        } else if (bms_tick != last_bms_tick) {
            last_bms_tick = bms_tick;
            (void)control_law_step_ibat(&s_ctl_state, bms_i_bat_cA, &s_ctl_params);
        }
        I_cmd = s_ctl_state.I_rect_cmd_cA;
#else
        I_cmd = control_law_step(&s_ctl_state, &in, &s_ctl_params);
#endif

        /* --- Publish setpoint to rectifier_task (unless experiment owns it) - */
        if (!expt_active) {
            pt_set_setpoint(I_cmd, final_mode);
        }

        /* --- Contactor policy (V1: always close both; fault-driven
         *     opens deferred until BMS / battery fault detection). --- */
        pt_set_contactor_cmds(true, true);

        /* --- Liveness heartbeat -------------------------------------- */
        osMutexAcquire(g_pt_mtx, osWaitForever);
        g_pt.supervisor_heartbeat++;
        osMutexRelease(g_pt_mtx);

        /* --- Watchdog kick — IWDG ~64 ms timeout (LSI 32 kHz, prescaler 4,
         *     reload 512). At 10 ms tick we kick ~6× per timeout window. */
        watchdog_refresh();

        next += SUP_PERIOD_MS;
        osDelayUntil(next);
    }
}
