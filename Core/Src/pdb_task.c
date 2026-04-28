#include "pdb_task.h"
#include "periph_wrappers.h"
#include "powertrain_state.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"

#define PDB_PERIOD_MS         20      /* 50 Hz */
#define PDB_SUP_HANG_MS      100      /* declare supervisor hung after 100 ms */

static StaticTask_t s_tcb;
static StackType_t  s_stack[256];     /* 1 KB */

static void pdb_task(void *arg);

void pdb_task_start(void) {
    static const osThreadAttr_t tattr = {
        .name       = "pdb",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityAboveNormal,    /* prio 4 */
    };
    osThreadNew(pdb_task, NULL, &tattr);
}

static void pdb_task(void *arg) {
    (void)arg;

    uint32_t next               = osKernelGetTickCount();
    uint32_t last_heartbeat     = 0;
    uint32_t last_progress_tick = osKernelGetTickCount();
    bool     supervisor_alive   = false;     /* don't drive until proven */

    /* GPIOs are at their reset state from MX_GPIO_Init (low = open).
     * PDB does its own precharge. We start writing only once the
     * supervisor's heartbeat shows progress. */

    for (;;) {
        uint32_t hb;
        bool     bat_cmd;
        bool     rect_cmd;

        osMutexAcquire(g_pt_mtx, osWaitForever);
        hb       = g_pt.supervisor_heartbeat;
        bat_cmd  = g_pt.contactor_battery_cmd;
        rect_cmd = g_pt.contactor_rectifier_cmd;
        osMutexRelease(g_pt_mtx);

        uint32_t now = osKernelGetTickCount();

        if (hb != last_heartbeat) {
            last_heartbeat     = hb;
            last_progress_tick = now;
            supervisor_alive   = true;
            pt_clear_fault(FAULT_SUPERVISOR_HANG);
        } else if (supervisor_alive &&
                   (now - last_progress_tick) > PDB_SUP_HANG_MS) {
            /* Supervisor stopped incrementing — hold-last-state.
             * NEVER open contactors here; mid-flight open is a crash. */
            supervisor_alive = false;
            pt_set_fault(FAULT_SUPERVISOR_HANG);
        }

        if (supervisor_alive) {
            pdb_hw_write(bat_cmd, rect_cmd);
        }
        /* else: do nothing — GPIO output level is retained by hardware. */

        next += PDB_PERIOD_MS;
        osDelayUntil(next);
    }
}
