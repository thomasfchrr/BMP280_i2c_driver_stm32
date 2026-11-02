/*
 * bmp280.h
 *
 *  Created on: 1 nov. 2026
 *      Author: Thomas
 */

#ifndef BMP280_BMP280_H_
#define BMP280_BMP280_H_

#include <stdint.h>
#include "stm32f4xx_hal.h"

// #define BMP280_I2C_ADDR		(0x76<<1)
#define BMP280_I2C_ADDR (0x77 << 1)
#define BMP280_CHIP_ID  0x58

// Registers (from datasheet table 18)

#define BMP280_REG_CALIB_START 0x88
#define BMP280_REG_ID          0xD0
#define BMP280_REG_RESET       0xE0
#define BMP280_REG_STATUS      0xF3
#define BMP280_REG_CTRL_MEAS   0xF4
#define BMP280_REG_CONFIG      0xF5
#define BMP280_REG_PRESS_MSB   0xF7
// #define BMP280_REG_PRESS_LSB	0xF8
// #define BMP280_REG_PRESS_XLSB	0xF9
#define BMP280_REG_TEMP_MSB 0xFA
// #define BMP280_REG_TEMP_LSB	0xFB
// #define BMP280_REG_TEMP_XLSB	0xFC

// Register control

#define BMP280_RESET_VALUE 0xB6

#define BMP280_OSRS_SKIP 0x00
#define BMP280_OSRS_X1   0x01
#define BMP280_OSRS_X2   0x02
#define BMP280_OSRS_X4   0x03
#define BMP280_OSRS_X8   0x04
#define BMP280_OSRS_X16  0x05

#define BMP280_MODE_SLEEP  0x00
#define BMP280_MODE_FORCED 0x01
#define BMP280_MODE_NORMAL 0x03

#define BMP280_STANDBY_0_5_MS  0x00
#define BMP280_STANDBY_62_5_MS 0x01
#define BMP280_STANDBY_125_MS  0x02
#define BMP280_STANDBY_250_MS  0x03
#define BMP280_STANDBY_500_MS  0x04
#define BMP280_STANDBY_1000_MS 0x05
#define BMP280_STANDBY_2000_MS 0x06
#define BMP280_STANDBY_4000_MS 0x07

#define BMP280_FILTER_OFF 0x00
#define BMP280_FILTER_2   0x01
#define BMP280_FILTER_4   0x02
#define BMP280_FILTER_8   0x03
#define BMP280_FILTER_16  0x04

typedef struct
{
	I2C_HandleTypeDef *hi2c;
	uint8_t addr;
	// calibration coef datasheet table 17:
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	int32_t t_fine;
} BMP280_t;

uint8_t BMP280_Init(BMP280_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr);

uint8_t BMP280_ReadRegs(BMP280_t *dev, uint8_t reg, uint8_t *buf, uint16_t len);

uint8_t BMP280_ReadReg(BMP280_t *dev, uint8_t reg, uint8_t *data);

uint8_t BMP280_WriteReg(BMP280_t *dev, uint8_t reg, uint8_t value);

uint8_t BMP280_ReadCalib(BMP280_t *dev);

// Status register 0xF3 (table 19)
#define BMP280_STATUS_MEASURING 0x08
#define BMP280_STATUS_IM_UPDATE 0x01

// Raw value output when a measurement is skipped (osrs = 0)
#define BMP280_RAW_SKIPPED 0x80000

uint8_t BMP280_Reset(BMP280_t *dev); // soft reset (0xB6), waits start-up time
uint8_t BMP280_ReadID(BMP280_t *dev, uint8_t *id);
uint8_t BMP280_ReadStatus(BMP280_t *dev, uint8_t *status);

// ctrl_meas (0xF4): osrs_t[7:5] osrs_p[4:2] mode[1:0]
uint8_t BMP280_SetCtrlMeas(BMP280_t *dev, uint8_t osrs_t, uint8_t osrs_p, uint8_t mode);
// config (0xF5): t_sb[7:5] filter[4:2] spi3w_en[0] (0 in I2C). Write in sleep mode only (datasheet 4.3.5)
uint8_t BMP280_SetConfig(BMP280_t *dev, uint8_t t_sb, uint8_t filter);
uint8_t BMP280_SetMode(BMP280_t *dev, uint8_t mode); // read-modify-write of mode[1:0]
uint8_t BMP280_GetMode(BMP280_t *dev, uint8_t *mode);

// Waits until measuring/im_update bits are cleared. Returns 0 ok, 1 I2C error, 2 timeout
uint8_t BMP280_WaitMeasurement(BMP280_t *dev, uint32_t timeout_ms);

// Waits for the end of a forced conversion (mode back to sleep). Returns 0 ok, 1 I2C error, 2 timeout
uint8_t BMP280_WaitForced(BMP280_t *dev, uint32_t timeout_ms);

// Raw 20 bit values (0xF7..0xFC), burst read of the 6 bytes
uint8_t BMP280_ReadRaw(BMP280_t *dev, int32_t *adc_T, int32_t *adc_P);

// Compensation formulas (datasheet 3.11.3 and appendix 8). T functions update dev->t_fine
int32_t BMP280_CompensateT_int32(BMP280_t *dev, int32_t adc_T);  // 0.01 degC
uint32_t BMP280_CompensateP_int64(BMP280_t *dev, int32_t adc_P); // Pa, Q24.8
uint32_t BMP280_CompensateP_int32(BMP280_t *dev, int32_t adc_P); // Pa (32 bit alternative)
double BMP280_CompensateT_double(BMP280_t *dev, int32_t adc_T);  // degC
double BMP280_CompensateP_double(BMP280_t *dev, int32_t adc_P);  // Pa

// Reads raw data in the current mode and returns degC / Pa
uint8_t BMP280_ReadTempPress(BMP280_t *dev, float *temp_c, float *press_pa);
// One forced measurement: trigger, wait, read. Device must be in sleep (or forced) mode
uint8_t BMP280_MeasureForced(BMP280_t *dev, uint8_t osrs_t, uint8_t osrs_p, float *temp_c, float *press_pa);

#endif /* BMP280_BMP280_H_ */
