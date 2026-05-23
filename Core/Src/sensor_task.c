#include "sensor_task.h"
#include "powertrain_state.h"
#include "main.h"
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

/* ACS772ECB-300B calibration: AINCOM bodged to GND, PGA bypass, AVDD ref. */
#define ACS772_SCALE_MA   (5.0f / 2147483648.0f / 0.00666f * 1000.0f) /* mA/LSB */
#define ACS772_OFFSET_MA  (-2.5f / 0.00666f * 1000.0f)                /* mA at raw=0 */

static const ADC_ChannelConfig_t s_ch_cfg[SENSOR_NUM_ADC_CH] = {
    { .muxp = ADC_AIN0, .muxn = ADC_AINCOM, .gain = ADC_GAIN_1, .pga_bypass = true,
      .scale = ACS772_SCALE_MA, .offset = ACS772_OFFSET_MA },
    { .muxp = ADC_AIN1, .muxn = ADC_AINCOM, .gain = ADC_GAIN_1, .pga_bypass = true,
      .scale = ACS772_SCALE_MA, .offset = ACS772_OFFSET_MA },
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

    /* ADC_Init deferred to task body; pre-kernel HAL_Delay hangs on this board. */

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

    /* Post-kernel ADC bring-up; HAL_Delay needs scheduler running. */
    ADC_Init(s_ch_cfg, SENSOR_NUM_ADC_CH, ADC_SPS_100, ADC_FILT_SINC4);

    uint32_t next = osKernelGetTickCount();

    for (;;) {
        /* ---- ADC per-channel scan on DRDY ---- */
        if (ADC_DataReady()) {
            ADC_ReadAndStore();
            ADC_ScanResult_t scan;
            if (ADC_GetScanResult(&scan)) {
                osMutexAcquire(s_mtx, osWaitForever);
                s_data.adc = scan;
                osMutexRelease(s_mtx);

                /* Mirror current sensor channels into g_pt. */
                osMutexAcquire(g_pt_mtx, osWaitForever);
                for (uint8_t i = 0; i < SENSOR_NUM_ADC_CH && i < 3; ++i) {
                    g_pt.current_sensor_mA[i] = (int32_t)scan.ch[i].value;
                }
                g_pt.current_sensor_tick = osKernelGetTickCount();
                osMutexRelease(g_pt_mtx);
            }
        }

        /* ---- 3 thermocouples on hspi2 ---- */
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

/* HAL EXTI -> ADS1262 DRDY. */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if (GPIO_Pin == ADC_DRDY_Pin) {
        ADC_DRDY_IRQ();
    }
}
