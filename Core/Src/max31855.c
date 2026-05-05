/*
 * max31855.c
 *
 *  Created on: Apr 6, 2026
 *      Author: ishan
 */


#include "max31855.h"
#include "spi.h"

extern SPI_HandleTypeDef hspi2;
static const struct{
	GPIO_TypeDef *port;
	uint16_t pin;
} tc_cs[] = {
		{CS_TC1_GPIO_Port,CS_TC1_Pin},{CS_TC2_GPIO_Port,CS_TC2_Pin},{CS_TC3_GPIO_Port, CS_TC3_Pin}
};

void MAX_SetCS(uint8_t ch, uint8_t state){
	if(ch > 2) return;
	HAL_GPIO_WritePin(tc_cs[ch].port,tc_cs[ch].pin,state ? GPIO_PIN_SET:GPIO_PIN_RESET);
}

void MAX_ReadSPI(uint8_t ch, uint32_t *data){
	if(ch > 2) { *data = 0; return; }
	uint8_t rx[4] = {0};
	uint8_t tx[4] = {0};
	MAX_SetCS(ch, 0);
	HAL_SPI_TransmitReceive(&hspi2, tx, rx, 4, 100);
	MAX_SetCS(ch, 1);

	*data = ((uint32_t)rx[0]<<24) | ((uint32_t)rx[1]<<16) | ((uint32_t)rx[2]<<8) | ((uint32_t)rx[3]);
}

bool MAX_GetData(uint32_t data, TC_Reading_t *reading){
	/* Bits 17 and 3 are reserved and always 0 in a valid MAX31855 frame.
	 * Either being set means the SPI read was corrupted (bus glitch, MISO
	 * float, contention with another chip). Signal corruption with
	 * ok=false AND fault=false so the caller can tell it apart from a
	 * genuine open/short fault (which sets fault=true). */
	if (((data >> 17) & 0x01u) || ((data >> 3) & 0x01u)) {
		reading->fault     = false;
		reading->short_vcc = false;
		reading->short_gnd = false;
		reading->open_ckt  = false;
		reading->tc_temp   = 0.0f;
		reading->cj_temp   = 0.0f;
		return false;
	}

	reading->fault = (data >> 16) & 0x01;
	reading->short_vcc = (data >> 2) & 0x01;
	reading -> short_gnd = (data >> 1) & 0x01;
	reading -> open_ckt = data & 0x01;

	int16_t tc_raw = (int16_t)(data>>18);
	if (tc_raw & 0x2000){
		tc_raw |= 0xC000;
	}
	reading->tc_temp = tc_raw *0.25f;

	int16_t cj_raw = (int16_t)((data>>4)&0x0FFF) ;
	if (cj_raw & 0x0800){
		cj_raw |= 0xF000;
	}
	reading->cj_temp = cj_raw * 0.0625f;
	return !reading->fault;
}
