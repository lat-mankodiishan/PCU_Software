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

/* Owned by fc_link_task; latched on first Fix2 frame with a real GPS time. */
extern volatile uint64_t g_utc_base_usec;
extern volatile uint32_t g_utc_base_tick;

#define LOG_GPS_WAIT_MS  3000u   /* Spin in pick_filename up to this long for first fix. */

static FATFS    s_fs;
static FIL      s_file;
static char     s_line[512];
static char     s_filename[32];  /* roomy for LOG_YYYYMMDD_HHMMSS.CSV */
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

/* Hand-rolled Unix epoch → calendar (no newlib gmtime_r dependency).
 * Accurate for years 1970..2099. */
static void epoch_to_ymdhms(uint64_t epoch_sec,
                            uint16_t *Y, uint8_t *M, uint8_t *D,
                            uint8_t *h, uint8_t *m, uint8_t *s) {
    uint32_t day_secs = (uint32_t)(epoch_sec % 86400u);
    uint64_t days     = epoch_sec / 86400u;
    *h = (uint8_t)(day_secs / 3600u);
    *m = (uint8_t)((day_secs / 60u) % 60u);
    *s = (uint8_t)(day_secs % 60u);

    uint16_t year = 1970;
    for (;;) {
        bool leap = ((year % 4u) == 0u && (year % 100u) != 0u) || ((year % 400u) == 0u);
        uint32_t year_days = leap ? 366u : 365u;
        if (days < year_days) break;
        days -= year_days;
        year++;
    }
    *Y = year;

    static const uint8_t mday[12] = { 31,28,31,30,31,30,31,31,30,31,30,31 };
    bool leap = ((year % 4u) == 0u && (year % 100u) != 0u) || ((year % 400u) == 0u);
    uint8_t month = 0;
    while (1) {
        uint8_t dim = (month == 1 && leap) ? 29u : mday[month];
        if (days < dim) break;
        days -= dim;
        month++;
    }
    *M = (uint8_t)(month + 1u);
    *D = (uint8_t)(days  + 1u);
}

/* LOG_YYYYMMDD_HHMMSS.CSV when GPS time is available (waits up to LOG_GPS_WAIT_MS);
 * falls back to LOGNNNN.CSV (next free index) otherwise. */
static void pick_filename(void) {
    uint32_t t_start = osKernelGetTickCount();
    while (g_utc_base_usec == 0u
           && (osKernelGetTickCount() - t_start) < LOG_GPS_WAIT_MS) {
        osDelay(100);
    }

    if (g_utc_base_usec != 0u) {
        uint64_t now_usec = g_utc_base_usec
            + ((uint64_t)(osKernelGetTickCount() - g_utc_base_tick) * 1000ull);
        uint64_t epoch_sec = now_usec / 1000000ull;
        uint16_t Y; uint8_t M, D, h, m, s;
        epoch_to_ymdhms(epoch_sec, &Y, &M, &D, &h, &m, &s);
        snprintf(s_filename, sizeof(s_filename),
                 "LOG_%04u%02u%02u_%02u%02u%02u.CSV",
                 Y, M, D, h, m, s);
        return;
    }

    /* GPS never showed up — fall back to monotonic index. */
    for (int i = 0; i < 10000; ++i) {
        snprintf(s_filename, sizeof(s_filename), "LOG%04d.CSV", i);
        FILINFO fno;
        if (f_stat(s_filename, &fno) == FR_NO_FILE) return;
    }
    strcpy(s_filename, "LOG.CSV");
}

static void log_task(void *arg) {
    (void)arg;

    /* On any FS failure, idle quietly; log is non-essential. */
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
           "duty_act,Iq_cA,Id_cA,vrun,vmt_echo,ext_tick,"
           "fc_state,fc_thr_pct,fc_tick,"
    	   "ecu_rpm,ecu_tps_x10,ecu_estat,ecu_tick,"
           "eng_state,eng_state_tick,"
           "ctl_i_bat_filt,ctl_i_bat_ref,ctl_i_rect_dem,ctl_p_rect_W,ctl_duty,ctl_theta,"
           "sup_hb,ct_bat,ct_rect,"
           "faults,"
           "expt_active,expt_phase,expt_label,"
           "tc1_cdeg,tc2_cdeg,tc3_cdeg,adc0,adc1,adc2,adc3\n", &s_file);
    f_sync(&s_file);

    uint32_t next = osKernelGetTickCount();
    for (;;) {
        powertrain_state_t pt;
        osMutexAcquire(g_pt_mtx, osWaitForever);
        pt = g_pt;
        osMutexRelease(g_pt_mtx);

        sensor_data_t sd;
        sensor_data_get(&sd);

        /* TC temps as int32 cdeg (x100 C) to avoid printf-float linkage. */
        const char *expt_label = pt.expt.label ? pt.expt.label : "";

        int n = snprintf(s_line, sizeof(s_line),
            "%lu,"
            "%u,%u,%d,%ld,%d,"
            "%u,%d,%u,%d,%u,%u,%lu,"
            "%d,%d,%d,%u,%u,%lu,"
            "%u,%u,%lu,"
            "%u,%d,%u,%lu,"
            "%u,%lu,"
            "%d,%d,%d,%lu,%u,%u,"
            "%lu,%u,%u,"
            "0x%04X,"
            "%u,%u,%s,"
            "%ld,%ld,%ld,%ld,%ld,%ld,%ld\n",
            (unsigned long)osKernelGetTickCount(),
            (unsigned)pt.rect_cmd.mode, (unsigned)pt.rect_cmd.ctrl_mode,
            pt.rect_cmd.I_cmd_cA, (long)pt.rect_cmd.omega_e_cmd_erpm, pt.rect_cmd.duty_cmd_x10000,
            pt.rect.state.V_dc_cV, pt.rect.state.I_dc_cA,
            pt.rect.state.gen_rpm, pt.rect.state.igbt_temp_C,
            pt.rect.state.fault_bits, pt.rect.state.seq,
            (unsigned long)pt.rect.tick,
            pt.rect.state_ext.duty_x10000, pt.rect.state_ext.Iq_cA, pt.rect.state_ext.Id_cA,
            (unsigned)pt.rect.state_ext.run_state, (unsigned)pt.rect.state_ext.motor_type_echo,
            (unsigned long)pt.rect.ext_tick,
            (unsigned)pt.fc.flight_state, pt.fc.throttle_dem_pct,
            (unsigned long)pt.fc.tick,
            pt.ecu.rpm,
			pt.ecu.tps_pct_x10,
            (unsigned)pt.ecu.engine_status,
            (unsigned long)pt.ecu.tick,
            (unsigned)pt.engine.state,
            (unsigned long)pt.engine.state_tick,
            pt.ctl.i_bat_filt_cA, pt.ctl.i_bat_ref_eff_cA,
            pt.ctl.i_rect_demand_cA,
            (unsigned long)pt.ctl.p_rect_W,
            pt.ctl.duty_x10000, pt.ctl.theta_pct_x100,
            (unsigned long)pt.supervisor_heartbeat,
            (unsigned)pt.contactor_cmd.battery,
            (unsigned)pt.contactor_cmd.rectifier,
            pt.fault_bits,
            (unsigned)pt.expt.active, (unsigned)pt.expt.phase_idx, expt_label,
            (long)(sd.tc[0].tc_temp * 100.0f),
            (long)(sd.tc[1].tc_temp * 100.0f),
            (long)(sd.tc[2].tc_temp * 100.0f),
            (long)sd.adc.ch[0].raw,
            (long)sd.adc.ch[1].raw,
            (long)sd.adc.ch[2].raw,
            (long)sd.adc.ch[3].raw);

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
