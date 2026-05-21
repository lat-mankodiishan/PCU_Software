/*
 * ADS1262.h
 *
 *  Created on: Apr 6, 2026
 *      Author: ishan
 */

#ifndef INC_ADS1262_H_
#define INC_ADS1262_H_

#include <stdint.h>
#include <stdbool.h>

//ADC SPI Commands
#define ADC_CMD_NOP 0x00
#define ADC_CMD_RESET 0x06
#define ADC_CMD_START 0x08
#define ADC_CMD_STOP 0x0A
#define ADC_CMD_RDATA 0x12 //Read ADC data
#define ADC_CMD_SYOCAL 0x16
#define ADC_CMD_SYGCAL 0x17
#define ADC_CMD_SFOCAL 0x19
#define ADC_CMD_RREG 0x20
#define ADC_CMD_WREG 0x40

//ADC Register Adresses
#define ADC_REG_ID 0x00
#define ADC_REG_POWER 0x01
#define ADC_REG_INTERFACE 0x02
#define ADC_REG_MODE0 0x03
#define ADC_REG_MODE1 0x04
#define ADC_REG_MODE2 0x05
#define ADC_REG_INPMUX 0x06
#define ADC_REG_OFCAL0 0x07
#define ADC_REG_OFCAL1 0x08
#define ADC_REG_OFCAL2 0x09
#define ADC_REG_FSCAL0 0x0A
#define ADC_REG_FSCAL1 0x0B
#define ADC_REG_FSCAL2 0x0C
#define ADC_REG_IDACMUX 0x0D
#define ADC_REG_IDACMAG 0x0E
#define ADC_REG_REFMUX 0x0F
#define ADC_REG_TDACP 0x10
#define ADC_REG_TDACN 0X11
#define ADC_REG_GPIOCON 0X12
#define ADC_REG_GPIODIR 0x13
#define ADC_REG_GPIODAT 0X14

#define ADC_DATA_BUF_SIZE 6
#define ADC_NUM_REG 0x15

// Status byte Masks
#define ADC2_NEW 0x80
#define ADC1_NEW 0x40
#define EXTCLK 0x20
#define REF_ALM 0x10
#define PGAL_ALM 0x08
#define PGAH_ALM 0x04
#define PGAD_ALM 0x02
#define RST_ALM 0x01

// POWER register
#define ADC_POWER_RESET     (1 << 4)
#define ADC_POWER_INTREF    (1 << 0)

// INTERFACE register
#define ADC_IF_STATUS       (1 << 2)
#define ADC_IF_CRC_OFF      0x00
#define ADC_IF_CRC_CHKSUM   0x01
#define ADC_IF_CRC_CRC      0x02

// MODE2 register
#define ADC_MODE2_BYPASS    (1 << 7)
/* TO BE ADDED REGISTER DEFAULT VALUE AND REGISTER PROPERTY BITMASKS*/
#define ADC_MAX_CH 10
//STRUCTS
typedef enum {
	ADC_AIN0 = 0x00,
	ADC_AIN1 = 0x01,
	ADC_AIN2 = 0x02,
	ADC_AIN3 = 0x03,
	ADC_AIN4 = 0x04,
	ADC_AIN5 = 0x05,
	ADC_AIN6 = 0x06,
	ADC_AIN7 = 0x07,
	ADC_AIN8 = 0x08,
	ADC_AIN9 = 0x09,
	ADC_AINCOM = 0x0A,
	ADC_TempSMP = 0x0B,
	ADC_AnaPSMN = 0x0C,
	ADC_DigPSMP = 0x0D,
	ADC_INP_TDACP = 0x0E,
	ADC_Open = 0x0F
} ADC_Input_t;

typedef enum {
	ADC_DELAY_NONE   = 0x00,
	ADC_DELAY_8_7US  = 0x01,
	ADC_DELAY_17US   = 0x02,
	ADC_DELAY_35US   = 0x03,
	ADC_DELAY_69US   = 0x04,
	ADC_DELAY_139US  = 0x05,
	ADC_DELAY_278US  = 0x06,
	ADC_DELAY_555US  = 0x07,
	ADC_DELAY_1_1MS  = 0x08,
	ADC_DELAY_2_2MS  = 0x09,
	ADC_DELAY_4_4MS  = 0x0A,
	ADC_DELAY_8_8MS  = 0x0B
} ADC_Delay_t; //Optional in case readings after mux switching is not stable

typedef enum {
	ADC_GAIN_1  = 0x00,
	ADC_GAIN_2  = 0x01,
	ADC_GAIN_4  = 0x02,
	ADC_GAIN_8  = 0x03,
	ADC_GAIN_16 = 0x04,
	ADC_GAIN_32 = 0x05
} ADC_Gain_t;

typedef enum {
	ADC_SPS_2_5   = 0x00,
	ADC_SPS_5     = 0x01,
	ADC_SPS_10    = 0x02,
	ADC_SPS_16_6  = 0x03,
	ADC_SPS_20    = 0x04,
	ADC_SPS_50    = 0x05,
	ADC_SPS_60    = 0x06,
	ADC_SPS_100   = 0x07,
	ADC_SPS_400   = 0x08,
	ADC_SPS_1200  = 0x09,
	ADC_SPS_2400  = 0x0A,
	ADC_SPS_4800  = 0x0B,
	ADC_SPS_7200  = 0x0C,
	ADC_SPS_14400 = 0x0D,
	ADC_SPS_19200 = 0x0E,
	ADC_SPS_38400 = 0x0F
} ADC_SPS_t;

typedef enum {
	ADC_FILT_SINC1 = 0x00,
	ADC_FILT_SINC2 = 0x01,
	ADC_FILT_SINC3 = 0x02,
	ADC_FILT_SINC4 = 0x03,
	ADC_FILT_FIR   = 0x04
} ADC_Filt_t;

typedef struct {
	ADC_Input_t     muxp;
	ADC_Input_t     muxn;
	ADC_Gain_t      gain;        /* ignored when pga_bypass == true                  */
	bool            pga_bypass;  /* true → MODE2.BYPASS=1, signal skips PGA          */
	float           scale;       /* raw → engineering: value = raw * scale + offset  */
	float           offset;
} ADC_ChannelConfig_t;

typedef struct {
	int32_t raw;
	float   value;
	bool    crc_valid;
} ADC_Reading_t;

typedef struct {
	ADC_Reading_t ch[ADC_MAX_CH];
	uint8_t num_ch_active;
	bool scan_complete;
} ADC_ScanResult_t;

void ADC_Reset(void);
bool ADC_VerifyID(void);
void ADC_SetCS(uint8_t state);
void ADC_WReg(uint8_t reg, uint8_t data);
void ADC_RReg(uint8_t reg, uint8_t *data, uint8_t num_reg);
void ADC_Init(const ADC_ChannelConfig_t *ch, uint8_t num_ch, ADC_SPS_t sps, ADC_Filt_t filt);
void ADC_SetStart(uint8_t state);
void ADC_StartADC(void);
void ADC_StopADC(void);
void ADC_DRDY_IRQ(void);
bool ADC_DataReady(void);
bool ADC_ReadRaw(int32_t *raw, uint8_t *status);
float ADC_ConverttoEngUnit (int32_t raw, uint8_t ch_index);
void ADC_SetChannel(uint8_t ch_index);
void ADC_SwitchToNextChannel(void);
void ADC_ReadAndStore(void);
bool ADC_GetScanResult(ADC_ScanResult_t *out);
void ADC_SelfOffset_Cal(void);
void ADC_SelfGain_Cal(void);
void ADC_DumpRegisters(void);
uint8_t ADC_SPIByteTransfer(uint8_t tx);

#endif /* INC_ADS1262_H_ */
