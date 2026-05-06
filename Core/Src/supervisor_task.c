#include "supervisor_task.h"
#include "control_law.h"
#include "powertrain_state.h"
#include "periph_wrappers.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#define SUP_PERIOD_MS       10        /* 100 Hz */

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

    for (;;) {
        /* --- Snapshot inputs under the mutex ------------------------- */
        ctl_inputs_t in;
        uint16_t     faults;

        osMutexAcquire(g_pt_mtx, osWaitForever);
        in.mode             = g_pt.fc_flight_state;
        in.throttle_dem_pct = g_pt.fc_throttle_dem_pct;
        in.soc_pct          = g_pt.bms_soc_pct;
        faults              = g_pt.fault_bits;
        osMutexRelease(g_pt_mtx);

        /* --- Mode-override on faults --------------------------------- */
        vesc_mode_t final_mode = in.mode;
        if (faults & (FAULT_RECT_OFFLINE | FAULT_BUS2_BUSOFF)) {
            final_mode = VESC_MODE_FAULT;
        }
        in.mode = final_mode;

        /* --- Run pure control law ------------------------------------ */
        int16_t I_cmd = control_law_step(&s_ctl_state, &in, &s_ctl_params);

        /* --- Publish setpoint to rectifier_task ---------------------- */
        pt_set_setpoint(I_cmd, final_mode);

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
