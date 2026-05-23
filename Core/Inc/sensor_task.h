#ifndef SENSOR_TASK_H
#define SENSOR_TASK_H

#include "ads1262.h"
#include "max31855.h"
#include <stdbool.h>
#include <stdint.h>

#define SENSOR_NUM_TC      3
#define SENSOR_NUM_ADC_CH  3       /* AIN0..AIN2 = ACS772 I_1..I_3 */

typedef struct {
    ADC_ScanResult_t adc;
    TC_Reading_t     tc[SENSOR_NUM_TC];
    bool             tc_valid[SENSOR_NUM_TC];
    uint32_t         tick;
} sensor_data_t;

void sensor_task_start(void);
void sensor_data_get(sensor_data_t *out);

#endif /* SENSOR_TASK_H */
