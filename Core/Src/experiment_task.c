#include "experiment_task.h"
#include "experiment.h"
#include "experiment_profiles.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

/* Boot profile. Profiles available:
 *   &starter_gen_profile          — BLDC 5%% crank, FOC inverted duty ladder 10/14/18%%.
 *   &engine_throttle_sweep_profile — BLDC 42%% crank, FOC inverted 60%% for 10 s,
 *                                    75%% for 100 s. Operator varies engine throttle. */
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

    /* Brief startup grace so BMS / rectifier / sensor tasks come up first
     * — keeps us from publishing setpoints before subscribers are listening. */
    osDelay(500);

    /* Defensive boot clear — never auto-start after reset, regardless of
     * what RAM held. pt_init() already zeros g_pt; this is belt-and-
     * suspenders for the case where a live-watch value persists across a
     * "stop debug" that didn't actually reset the chip. */
    osMutexAcquire(g_pt_mtx, osWaitForever);
    g_pt.expt_start_req   = false;
    g_pt.expt_advance_req = false;
    g_pt.expt_abort_req   = false;
    osMutexRelease(g_pt_mtx);

    /* Edge-triggered start: only a false → true transition triggers a run.
     * Initialize prev_start = true so the first poll must observe false
     * before any subsequent true counts as an edge. After a run, we force
     * prev_start back to true so the operator must explicitly toggle the
     * flag low and high again to replay. */
    bool prev_start = true;

    for (;;) {
        bool cur_start;
        osMutexAcquire(g_pt_mtx, osWaitForever);
        cur_start = g_pt.expt_start_req;
        osMutexRelease(g_pt_mtx);

        if (!prev_start && cur_start) {
            /* false → true edge: consume the flag and run. */
            osMutexAcquire(g_pt_mtx, osWaitForever);
            g_pt.expt_start_req = false;
            osMutexRelease(g_pt_mtx);

            expt_run(EXPT_BOOT_PROFILE);

            /* Re-arm only after another false→true cycle. */
            prev_start = true;
        } else {
            prev_start = cur_start;
        }

        osDelay(100);
    }
}
