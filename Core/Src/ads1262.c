/*
 * ads1262.c
 *
 *  Created on: Apr 6, 2026
 *      Author: ishan
 */

#include "ads1262.h"
#include <stdbool.h>
#include "spi.h"
#include <string.h>
#include <stdio.h>

extern SPI_HandleTypeDef hspi3;
static const ADC_ChannelConfig_t *channels;
static volatile bool drdy_flag = false;
static ADC_SPS_t stored_sps;
static uint8_t num_channels;
static uint8_t current_ch = 0;
static ADC_Reading_t results[ADC_MAX_CH];
static bool scan_complete = false;


void ADC_SetCS(uint8_t state){
	HAL_GPIO_WritePin(CS_ADC_GPIO_Port, CS_ADC_Pin, state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void ADC_SetStart(uint8_t state){
	HAL_GPIO_WritePin(ADC_START_GPIO_Port, ADC_START_Pin, state ? GPIO_PIN_SET:GPIO_PIN_RESET);
}

uint8_t ADC_SPIByteTransfer(uint8_t tx){
	uint8_t rx =0;
	HAL_SPI_TransmitReceive(&hspi3, &tx, &rx, 1, 100);
	return rx;
}

void ADC_WReg(uint8_t reg, uint8_t data){
	ADC_SetCS(0);
	ADC_SPIByteTransfer(ADC_CMD_WREG | reg);
	ADC_SPIByteTransfer(0x00);
	ADC_SPIByteTransfer(data);
	ADC_SetCS(1);
}

void ADC_RReg(uint8_t reg, uint8_t *data, uint8_t num_reg){
    ADC_SetCS(0);
    ADC_SPIByteTransfer(ADC_CMD_RREG | reg);
    ADC_SPIByteTransfer(num_reg - 1);
    for (uint8_t i = 0; i < num_reg; i++){
        data[i] = ADC_SPIByteTransfer(0x00);
    }
    ADC_SetCS(1);
}

//hardware reset
void ADC_Reset(void){
    HAL_GPIO_WritePin(ADC_RST_GPIO_Port, ADC_RST_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(ADC_RST_GPIO_Port, ADC_RST_Pin, GPIO_PIN_SET);
    HAL_Delay(10);
}

void ADC_StartADC(void){
	ADC_SetCS(0);
	ADC_SPIByteTransfer(ADC_CMD_START);
	ADC_SetCS(1);
}

void ADC_StopADC(void){
	ADC_SetCS(0);
	ADC_SPIByteTransfer(ADC_CMD_STOP);
	ADC_SetCS(1);
}

bool ADC_VerifyID(void){
	uint8_t id =0;
	ADC_RReg(ADC_REG_ID, &id, 1);
	return ((id>>5)==0x00);
}

void ADC_DRDY_IRQ(void){
     drdy_flag = true;
 }

bool ADC_DataReady(void){
	if(drdy_flag){
		drdy_flag = false;
		return true;
	}
	return false;
}

bool ADC_ReadRaw(int32_t *raw, uint8_t *status){
      uint8_t tx[7] = {ADC_CMD_RDATA, 0, 0, 0, 0, 0, 0};
      uint8_t rx[7] = {0};

      ADC_SetCS(0);
      HAL_SPI_TransmitReceive(&hspi3, tx, rx, 7, 100);
      ADC_SetCS(1);

      *status = rx[1];
      *raw = ((int32_t)rx[2] << 24) | ((int32_t)rx[3] << 16) |
             ((int32_t)rx[4] << 8)  | ((int32_t)rx[5]);
      uint8_t sum = rx[1] + rx[2] + rx[3] + rx[4] + rx[5];
      return (sum == rx[6]);
  }

float ADC_ConverttoEngUnit (int32_t raw, uint8_t ch_index){
	return (raw*channels[ch_index].scale) + channels[ch_index].offset;
}

void ADC_SetChannel(uint8_t ch_index){
	uint8_t mux = (channels[ch_index].muxp<<4) | channels[ch_index].muxn;
	uint8_t mode2 = (channels[ch_index].gain<<4) | stored_sps;

	ADC_SetCS(0);
	ADC_SPIByteTransfer(ADC_CMD_WREG | ADC_REG_MODE2);
	ADC_SPIByteTransfer(0x01);
	ADC_SPIByteTransfer(mode2);
	ADC_SPIByteTransfer(mux);
	ADC_SetCS(1);
}

void ADC_SwitchToNextChannel(void){
	current_ch = (current_ch + 1)%num_channels;
	if (current_ch == 0){
		scan_complete = true;
	}
	ADC_SetChannel(current_ch);
}

void ADC_ReadAndStore(void){
    int32_t raw;
    uint8_t status;
    bool valid = ADC_ReadRaw(&raw, &status);
    results[current_ch].raw = raw;
    results[current_ch].crc_valid = valid;
    ADC_SwitchToNextChannel();
}

bool ADC_GetScanResult(ADC_ScanResult_t *out){
    if (!scan_complete) return false;
    scan_complete = false;
    for (uint8_t i = 0; i < num_channels; i++){
        out->ch[i].raw = results[i].raw;
        out->ch[i].value = ADC_ConverttoEngUnit(results[i].raw, i);
        out->ch[i].crc_valid = results[i].crc_valid;
    }
    out->num_ch_active = num_channels;
    out->scan_complete = true;
    return true;
}

void ADC_SelfOffset_Cal(void){
	ADC_StopADC();
	ADC_WReg(ADC_REG_INPMUX, 0xFF);
	HAL_Delay(1);

	ADC_StartADC();

	ADC_SetCS(0);
	ADC_SPIByteTransfer(ADC_CMD_SFOCAL);
	ADC_SetCS(1);

	while(!ADC_DataReady()){}

	ADC_SetChannel(0);
	current_ch = 0;
}

void ADC_SelfGain_Cal(void){
      ADC_StopADC();
      ADC_StartADC();

      ADC_SetCS(0);
      ADC_SPIByteTransfer(ADC_CMD_SYGCAL);
      ADC_SetCS(1);

      while(!ADC_DataReady()){}

      ADC_SetChannel(0);
      current_ch = 0;
  }

void ADC_Init(const ADC_ChannelConfig_t *ch, uint8_t num_ch, ADC_SPS_t sps, ADC_Filt_t filt){
	channels = ch;
	num_channels = num_ch;
	stored_sps = sps;
	current_ch = 0;
	scan_complete = false;

	ADC_Reset();
	ADC_VerifyID();
	ADC_WReg(ADC_REG_POWER, ADC_POWER_INTREF);
	HAL_Delay(100);
	ADC_WReg(ADC_REG_INTERFACE, ADC_IF_STATUS | ADC_IF_CRC_CHKSUM);
	ADC_WReg(ADC_REG_MODE0, ADC_DELAY_17US << 0);
	ADC_WReg(ADC_REG_MODE1, (filt << 5));
	ADC_SetChannel(0);
	ADC_WReg(ADC_REG_REFMUX, 0x00);
	ADC_StartADC();
}

void ADC_DumpRegisters(void)
{
    uint8_t regs[ADC_NUM_REG];
    ADC_RReg(ADC_REG_ID, regs, ADC_NUM_REG);

    for (uint8_t i = 0; i < ADC_NUM_REG; i++) {
        printf("REG[0x%02X] = 0x%02X\r\n", i, regs[i]);
    }
}
