#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include "ads1262.h"
#include "max31855.h"
#include <stdbool.h>
#include <stdint.h>

#define SENSOR_NUM_TC      3
#define SENSOR_NUM_ADC_CH  4

typedef struct {
    ADC_ScanResult_t adc;
    TC_Reading_t     tc[SENSOR_NUM_TC];
    bool             tc_valid[SENSOR_NUM_TC];
    uint32_t         tick;          /* osKernelGetTickCount() of last update */
} sensor_data_t;

/* Create the sensor task. Call after pt_init(), before osKernelStart(). */
void sensor_task_start(void);

/* Mutex-protected snapshot of the latest readings. */
void sensor_data_get(sensor_data_t *out);

#endif /* SENSOR_TASK_H */
