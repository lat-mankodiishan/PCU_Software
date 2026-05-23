#include "experiment_task.h"
#include "experiment.h"
#include "experiment_profiles.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

/* Available: &starter_gen_profile, &engine_throttle_sweep_profile. */
#define EXPT_BOOT_PROFILE   (&engine_throttle_sweep_profile)

static StaticTask_t s_tcb;
static StackType_t  s_stack[256];        /* 1 KB */

static void expt_task(void *arg);

void expt_task_start(void) {
    static const osThreadAttr_t tattr = {
        .name       = "expt",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityNormal,  /* prio 3 */
    };
    osThreadNew(expt_task, NULL, &tattr);
}

static void expt_task(void *arg) {
    (void)arg;

    /* Startup grace; let subscribers come up first. */
    osDelay(500);

    /* Defensive boot clear; guards live-watch values surviving non-reset. */
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.expt_start_req   = false;
    g_pt.expt_advance_req = false;
    g_pt.expt_abort_req   = false;
    osMutexRelease(g_pt_mtx);

    /* Edge-triggered start: false->true triggers a run. */
    bool prev_start = true;

    for (;;) {
        bool cur_start;
        osMutexAcquire(g_pt_mtx, osWaitForever);
        cur_start = g_pt.expt_start_req;
        osMutexRelease(g_pt_mtx);

        if (!prev_start && cur_start) {
            osMutexAcquire(g_pt_mtx, osWaitForever);
            g_pt.expt_start_req = false;
            osMutexRelease(g_pt_mtx);

            expt_run(EXPT_BOOT_PROFILE);

            prev_start = true;
        } else {
            prev_start = cur_start;
        }

        osDelay(100);
    }
}
