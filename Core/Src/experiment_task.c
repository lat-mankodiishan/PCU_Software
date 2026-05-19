#include "experiment_task.h"
#include "experiment.h"
#include "experiment_profiles.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

/* Boot profile. Only one defined right now:
 *   &starter_gen_profile — BLDC 5%% duty crank, FOC inverted, current
 *                          ladder 1..9 A in 2 A steps, spindown. */
#define EXPT_BOOT_PROFILE   (&starter_gen_profile)

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

    /* Brief startup grace so BMS / rectifier / sensor tasks come up first
     * — keeps us from publishing setpoints before subscribers are listening. */
    osDelay(500);

    /* Wait for explicit start — does NOT auto-run on boot. Set
     * g_pt.expt_start_req = true from the debugger when ready (or via any
     * other code path). The flag is cleared by us before expt_run, so a
     * second poke after the run finishes will replay the profile. */
    for (;;) {
        bool go;
        osMutexAcquire(g_pt_mtx, osWaitForever);
        go = g_pt.expt_start_req;
        if (go) g_pt.expt_start_req = false;
        osMutexRelease(g_pt_mtx);

        if (go) {
            expt_run(EXPT_BOOT_PROFILE);
        }
        osDelay(100);
    }
}
