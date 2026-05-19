#include "sensor_task.h"
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

/* Default channel config — generic AIN0..AIN3 vs AINCOM, unity gain.
 * Override `scale` / `offset` per channel once the front-end signal
 * conditioning is known (V_bus divider, current-shunt amp, etc.). */
static const ADC_ChannelConfig_t s_ch_cfg[SENSOR_NUM_ADC_CH] = {
    { .muxp = ADC_AIN0, .muxn = ADC_AINCOM, .gain = ADC_GAIN_1, .scale = 1.0f, .offset = 0.0f },
    { .muxp = ADC_AIN1, .muxn = ADC_AINCOM, .gain = ADC_GAIN_1, .scale = 1.0f, .offset = 0.0f },
    { .muxp = ADC_AIN2, .muxn = ADC_AINCOM, .gain = ADC_GAIN_1, .scale = 1.0f, .offset = 0.0f },
    { .muxp = ADC_AIN3, .muxn = ADC_AINCOM, .gain = ADC_GAIN_1, .scale = 1.0f, .offset = 0.0f },
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
     * pre-kernel calls have proven flaky depending on NVIC state. Post-
     * kernel HAL_Delay is reliable because TIM6's tick IRQ is unmasked. */

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

    /* Run ADC bring-up here, post-kernel: ADC_Reset sleeps via HAL_Delay
     * and that needs TIM6's IRQ active, which is reliable once the
     * scheduler is running. Other tasks reading sensor_data_get during
     * this ~110 ms see the zero-initialized s_data — fine for bring-up. */
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
            }
        }

        /* --- 3 thermocouples on hspi2 ----------------------------------- *
         * MAX_GetData distinguishes three outcomes:
         *   ok=true              → clean read, publish.
         *   ok=false, fault=true → genuine open/short fault, publish so
         *                          consumers see the fault state.
         *   ok=false, fault=false→ corrupt SPI frame (reserved bits set),
         *                          hold the last good reading.
         * tc_valid mirrors ok so consumers can see the latest sample's
         * trustworthiness. */
        for (uint8_t i = 0; i < SENSOR_NUM_TC; ++i) {
            uint32_t raw;
            MAX_ReadSPI(i, &raw);
            TC_Reading_t r;
            bool ok = MAX_GetData(raw, &r);
            osMutexAcquire(s_mtx, osWaitForever);
            if (ok || r.fault) {
                s_data.tc[i] = r;
            }
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
