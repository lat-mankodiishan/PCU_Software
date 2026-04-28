/*
 * max31855.h
 *
 *  Created on: Apr 6, 2026
 *      Author: ishan
 */

#ifndef INC_MAX31855_H_
#define INC_MAX31855_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct{
	float tc_temp;
	float cj_temp;
	bool fault;
	bool short_vcc;
	bool short_gnd;
	bool open_ckt;
} TC_Reading_t;

void MAX_SetCS(uint8_t ch, uint8_t state);
void MAX_ReadSPI(uint8_t ch, uint32_t *data);
bool MAX_GetData(uint32_t data, TC_Reading_t *reading);


#endif /* INC_MAX31855_H_ */
