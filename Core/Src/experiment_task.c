#include "experiment_task.h"
#include "experiment.h"
#include "experiment_profiles.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

/* Pick which profile runs at boot. Re-flash to change. Options:
 *   &phase1_motor_only_profile  — omega ladder 0..8k eRPM
 *   &crank_to_foc_profile       — BLDC@30%duty -> FOC@3k eRPM via 0x104
 *   &starter_gen_profile        — manual-advance starter/gen bring-up
 *   &dyno_sweep_profile         — auto eRPM ladder for characterization */
#define EXPT_BOOT_PROFILE   (&crank_to_foc_profile)

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

    expt_run(EXPT_BOOT_PROFILE);

    /* Profile complete or aborted. Hold idle indefinitely; expt_run already
     * dropped the setpoint to 0 / VESC_MODE_IDLE on exit. */
    for (;;) osDelay(1000);
}
