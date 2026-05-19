#include "log_task.h"
#include "powertrain_state.h"
#include "sensor_task.h"
#include "fatfs.h"
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include <stdio.h>
#include <string.h>

#define LOG_PERIOD_MS    100         /* 10 Hz */
#define LOG_SYNC_EVERY    10         /* f_sync once per second */

static StaticTask_t s_tcb;
static StackType_t  s_stack[512];    /* 2 KB */

static FATFS    s_fs;
static FIL      s_file;
static char     s_line[512];
static char     s_filename[16];
static bool     s_mounted = false;
static uint32_t s_writes_since_sync = 0;

static void log_task(void *arg);

void log_task_start(void) {
    static const osThreadAttr_t tattr = {
        .name       = "log",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityLow,       /* prio 2 */
    };
    osThreadNew(log_task, NULL, &tattr);
}

/* Find next available LOGNNNN.CSV; fall back to LOG.CSV if all used. */
static void pick_filename(void) {
    for (int i = 0; i < 10000; ++i) {
        snprintf(s_filename, sizeof(s_filename), "LOG%04d.CSV", i);
        FILINFO fno;
        if (f_stat(s_filename, &fno) == FR_NO_FILE) return;
    }
    strcpy(s_filename, "LOG.CSV");
}

static void log_task(void *arg) {
    (void)arg;

    /* Mount + open + header. On any failure, fall through to a quiet idle
     * loop — log_task is non-essential, must never block other tasks. */
    if (f_mount(&s_fs, "", 1) != FR_OK) {
        for (;;) osDelay(1000);
    }
    s_mounted = true;
    pick_filename();
    if (f_open(&s_file, s_filename, FA_WRITE | FA_CREATE_ALWAYS) != FR_OK) {
        for (;;) osDelay(1000);
    }
    f_puts("ms,"
           "mode,ctrl_mode,I_cmd_cA,omega_cmd,duty_cmd,"
           "V_dc_cV,I_dc_cA,gen_rpm,igbt_C,rect_fault,rect_seq,rect_tick,"
           "fc_state,fc_thr_pct,fc_tick,"
           "bms_soc,bms_v_cV,bms_i_cA,bms_C,bms_tick,"
           "ecu_rpm,ecu_fuel_dg_s,ecu_cht_C,ecu_tick,"
           "sup_hb,ct_bat,ct_rect,"
           "faults,"
           "expt_active,expt_phase,expt_label,"
           "tc1_cdeg,tc2_cdeg,tc3_cdeg,adc0,adc1,adc2,adc3,"
           "dyno_I_mA,dyno_V_mV,dyno_P_dW,dyno_E_Wh,dyno_tick,"
           "ld_I_mA,ld_V_mV,ld_P_dW,ld_E_Wh,ld_tick,"
           "thr_en,thr_duty,thr_err_mA,thr_src_filt_mA,thr_tick\n", &s_file);
    f_sync(&s_file);

    uint32_t next = osKernelGetTickCount();
    for (;;) {
        powertrain_state_t pt;
        osMutexAcquire(g_pt_mtx, osWaitForever);
        pt = g_pt;
        osMutexRelease(g_pt_mtx);

        sensor_data_t sd;
        sensor_data_get(&sd);

        /* tc temps written as int32 cdeg (×100 °C) to avoid pulling
         * printf-float into the link. Decode by /100 in post-proc. */
        const char *expt_label = pt.expt_label ? pt.expt_label : "";

        int n = snprintf(s_line, sizeof(s_line),
            "%lu,"
            "%u,%u,%d,%ld,%d,"
            "%u,%d,%u,%d,%u,%u,%lu,"
            "%u,%u,%lu,"
            "%u,%u,%d,%d,%lu,"
            "%u,%u,%d,%lu,"
            "%lu,%u,%u,"
            "0x%04X,"
            "%u,%u,%s,"
            "%ld,%ld,%ld,%ld,%ld,%ld,%ld,"
            "%ld,%ld,%lu,%lu,%lu,"
            "%ld,%ld,%lu,%lu,%lu,"
            "%u,%u,%ld,%ld,%lu\n",
            (unsigned long)osKernelGetTickCount(),
            (unsigned)pt.mode, (unsigned)pt.rect_ctrl_mode,
            pt.I_rect_cmd_cA, (long)pt.omega_e_cmd_erpm, pt.duty_cmd_x10000,
            pt.rect_state.V_dc_cV, pt.rect_state.I_dc_cA,
            pt.rect_state.gen_rpm, pt.rect_state.igbt_temp_C,
            pt.rect_state.fault_bits, pt.rect_state.seq,
            (unsigned long)pt.rect_state_tick,
            (unsigned)pt.fc_flight_state, pt.fc_throttle_dem_pct,
            (unsigned long)pt.fc_input_tick,
            pt.bms_soc_pct, pt.bms_v_bat_cV, pt.bms_i_bat_cA,
            pt.bms_max_cell_C, (unsigned long)pt.bms_input_tick,
            pt.ecu_rpm, pt.ecu_fuel_rate_dg_s, pt.ecu_cht_C,
            (unsigned long)pt.ecu_input_tick,
            (unsigned long)pt.supervisor_heartbeat,
            (unsigned)pt.contactor_battery_cmd,
            (unsigned)pt.contactor_rectifier_cmd,
            pt.fault_bits,
            (unsigned)pt.expt_active, (unsigned)pt.expt_phase_idx, expt_label,
            (long)(sd.tc[0].tc_temp * 100.0f),
            (long)(sd.tc[1].tc_temp * 100.0f),
            (long)(sd.tc[2].tc_temp * 100.0f),
            (long)sd.adc.ch[0].raw,
            (long)sd.adc.ch[1].raw,
            (long)sd.adc.ch[2].raw,
            (long)sd.adc.ch[3].raw,
            (long)pt.dyno_current_mA,
            (long)pt.dyno_vbus_mV,
            (unsigned long)pt.dyno_power_dW,
            /* energy is uint64 but %llu pulls in newlib (not -nano);
             * truncate to 32 bits — 4 GWh is far past any bench run. */
            (unsigned long)pt.dyno_energy_Wh,
            (unsigned long)pt.dyno_input_tick,
            (long)pt.dyno_load_current_mA,
            (long)pt.dyno_load_vbus_mV,
            (unsigned long)pt.dyno_load_power_dW,
            (unsigned long)pt.dyno_load_energy_Wh,   /* truncated to u32, see note */
            (unsigned long)pt.dyno_load_input_tick,
            (unsigned)pt.throttle_ctrl_enabled,
            (unsigned)pt.throttle_pwm_duty,
            (long)pt.throttle_current_err_mA,
            (long)pt.throttle_src_filt_mA,
            (unsigned long)pt.throttle_ctrl_tick);

        UINT bw;
        if (n > 0 && f_write(&s_file, s_line, (UINT)n, &bw) == FR_OK) {
            if (++s_writes_since_sync >= LOG_SYNC_EVERY) {
                f_sync(&s_file);
                s_writes_since_sync = 0;
            }
        }

        next += LOG_PERIOD_MS;
        osDelayUntil(next);
    }
}
