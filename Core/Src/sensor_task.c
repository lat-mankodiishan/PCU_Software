#include "sensor_task.h"
#include "powertrain_state.h"
#include "main.h"               /* ADC_DRDY_Pin */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>

#define SENSOR_PERIOD_MS   100       /* 10 Hz */

static StaticTask_t      s_tcb;
static StackType_t       s_stack[384];        /* 1.5 KB */

static StaticSemaphore_t s_mtx_cb;
static osMutexId_t       s_mtx;
static sensor_data_t     s_data;

/* --- ACS772ECB-300B current sensor calibration ---------------------------
 * Three sensors on AIN0/AIN1/AIN2 vs AINCOM. AINCOM must be bodged to GND
 * on the PCB (datasheet pin can't be routed to AVSS via INPMUX, must be
 * a physical connection). With AINN=GND, single-ended swing is 0.5..4.5 V
 * — exceeds the PGA's ±2.5 V range at gain 1 + internal VREF, so each
 * channel runs with PGA bypassed and the reference comes from AVDD/AVSS
 * (REFMUX = 0x24 in ADC_Init). Differential range becomes ±5 V.
 *
 * Math (per channel):
 *   V_out_V  = raw * (VREF / 2^31)            with VREF = AVDD ≈ 5 V
 *   I_A      = (V_out_V - V_quiescent) / sensitivity
 *   V_quiescent  = Vcc/2 = 2.5 V (ratiometric to ACS772 supply, 5 V)
 *   sensitivity  = 6.66 mV/A = 0.00666 V/A
 *
 *   I_mA = raw * SCALE_MA + OFFSET_MA
 *     SCALE_MA  = (5.0 / 2^31) / 0.00666 * 1000  ≈ 3.4956e-4 mA/LSB
 *     OFFSET_MA = -2.5 / 0.00666 * 1000          ≈ -375 375 mA
 *
 * To calibrate per-sensor offset drift on the bench: run with no current,
 * read raw values from Live Watch (g_pt.current_sensor_mA[i] near 0 mA),
 * and tune .offset per channel until each channel reads zero. Sensitivity
 * (.scale) is fixed by the part — do NOT adjust .scale unless you've
 * verified a non-nominal sensitivity. */
#define ACS772_SCALE_MA   (5.0f / 2147483648.0f / 0.00666f * 1000.0f)   /* mA per ADC LSB */
#define ACS772_OFFSET_MA  (-2.5f / 0.00666f * 1000.0f)                  /* mA at raw = 0  */

static const ADC_ChannelConfig_t s_ch_cfg[SENSOR_NUM_ADC_CH] = {
    /* I_1 — ACS772 #1 on AIN0 */
    { .muxp = ADC_AIN0, .muxn = ADC_AINCOM, .gain = ADC_GAIN_1, .pga_bypass = true,
      .scale = ACS772_SCALE_MA, .offset = ACS772_OFFSET_MA },
    /* I_2 — ACS772 #2 on AIN1 */
    { .muxp = ADC_AIN1, .muxn = ADC_AINCOM, .gain = ADC_GAIN_1, .pga_bypass = true,
      .scale = ACS772_SCALE_MA, .offset = ACS772_OFFSET_MA },
    /* I_3 — ACS772 #3 on AIN2 */
    { .muxp = ADC_AIN2, .muxn = ADC_AINCOM, .gain = ADC_GAIN_1, .pga_bypass = true,
      .scale = ACS772_SCALE_MA, .offset = ACS772_OFFSET_MA },
};

static void sensor_task(void *arg);

void sensor_task_start(void) {
    static const osMutexAttr_t mattr = {
        .name      = "sensor_mtx",
        .attr_bits = osMutexPrioInherit,
        .cb_mem    = &s_mtx_cb,
        .cb_size   = sizeof(s_mtx_cb),
    };
    s_mtx = osMutexNew(&mattr);
    memset(&s_data, 0, sizeof(s_data));

    /* ADC_Init is deferred into the task body — it spins on HAL_Delay and
     * pre-kernel calls have proven flaky on this board (HAL_GetTick stalls
     * before osKernelStart, even though TIM6 is nominally initialised).
     * Post-kernel HAL_Delay is reliable. */

    static const osThreadAttr_t tattr = {
        .name       = "sensor",
        .cb_mem     = &s_tcb,
        .cb_size    = sizeof(s_tcb),
        .stack_mem  = s_stack,
        .stack_size = sizeof(s_stack),
        .priority   = osPriorityNormal,        /* prio 3 */
    };
    osThreadNew(sensor_task, NULL, &tattr);
}

void sensor_data_get(sensor_data_t *out) {
    osMutexAcquire(s_mtx, osWaitForever);
    *out = s_data;
    osMutexRelease(s_mtx);
}

static void sensor_task(void *arg) {
    (void)arg;

    /* Post-kernel ADC bring-up: HAL_Delay inside ADC_Reset waits on TIM6
     * ticks, which only fire reliably once the scheduler is running.
     * Consumers reading sensor_data_get during this ~110 ms see the
     * zero-initialised s_data — fine for bring-up. */
    ADC_Init(s_ch_cfg, SENSOR_NUM_ADC_CH, ADC_SPS_100, ADC_FILT_SINC4);

    uint32_t next = osKernelGetTickCount();

    for (;;) {
        /* --- ADC: drive the per-channel scan when DRDY is set ----------- */
        if (ADC_DataReady()) {
            ADC_ReadAndStore();
            ADC_ScanResult_t scan;
            if (ADC_GetScanResult(&scan)) {
                osMutexAcquire(s_mtx, osWaitForever);
                s_data.adc = scan;
                osMutexRelease(s_mtx);

                /* Mirror current sensor channels into g_pt for consumers
                 * (log_task, supervisor, expt). ACS772 raw → mA conversion
                 * already done in ADC_ConverttoEngUnit via the per-channel
                 * scale/offset; cast float → int32 for the g_pt fields. */
                osMutexAcquire(g_pt_mtx, osWaitForever);
                for (uint8_t i = 0; i < SENSOR_NUM_ADC_CH && i < 3; ++i) {
                    g_pt.current_sensor_mA[i] = (int32_t)scan.ch[i].value;
                }
                g_pt.current_sensor_tick = osKernelGetTickCount();
                osMutexRelease(g_pt_mtx);
            }
        }

        /* --- 3 thermocouples on hspi2 ----------------------------------- */
        for (uint8_t i = 0; i < SENSOR_NUM_TC; ++i) {
            uint32_t raw;
            MAX_ReadSPI(i, &raw);
            TC_Reading_t r;
            bool ok = MAX_GetData(raw, &r);
            osMutexAcquire(s_mtx, osWaitForever);
            s_data.tc[i]       = r;
            s_data.tc_valid[i] = ok;
            osMutexRelease(s_mtx);
        }

        osMutexAcquire(s_mtx, osWaitForever);
        s_data.tick = osKernelGetTickCount();
        osMutexRelease(s_mtx);

        next += SENSOR_PERIOD_MS;
        osDelayUntil(next);
    }
}

/* HAL EXTI callback — routed to ADS1262's DRDY handler.
 * Lives here because sensor_task is the sole consumer. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ADC_DRDY_Pin) {
        ADC_DRDY_IRQ();
    }
}
