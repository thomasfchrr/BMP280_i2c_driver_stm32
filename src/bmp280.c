/*
 * bmp280.c
 *
 *  Created on: 1 nov. 2026
 *      Author: Thomas
 */

#include "bmp280.h"

uint8_t BMP280_ReadRegs(BMP280_t *dev, uint8_t reg, uint8_t *buf, uint16_t len)
{
	return (uint8_t)HAL_I2C_Mem_Read(dev->hi2c, dev->addr, reg, I2C_MEMADD_SIZE_8BIT, buf, len, 100);
}

uint8_t BMP280_ReadReg(BMP280_t *dev, uint8_t reg, uint8_t *data)
{
	return BMP280_ReadRegs(dev, reg, data, 1);
}

uint8_t BMP280_WriteReg(BMP280_t *dev, uint8_t reg, uint8_t value)
{
	return (uint8_t)HAL_I2C_Mem_Write(dev->hi2c, dev->addr, reg, I2C_MEMADD_SIZE_8BIT, &value, 1, 100);
}

uint8_t BMP280_ReadCalib(BMP280_t *dev)
{
	uint8_t b[24];

	if (BMP280_ReadRegs(dev, BMP280_REG_CALIB_START, b, 24) != 0)
	{
		return 1;
	}

	dev->dig_T1 = (uint16_t)((b[1] << 8) | b[0]);
	dev->dig_T2 = (int16_t)((b[3] << 8) | b[2]);
	dev->dig_T3 = (int16_t)((b[5] << 8) | b[4]);
	dev->dig_P1 = (uint16_t)((b[7] << 8) | b[6]);
	dev->dig_P2 = (int16_t)((b[9] << 8) | b[8]);
	dev->dig_P3 = (int16_t)((b[11] << 8) | b[10]);
	dev->dig_P4 = (int16_t)((b[13] << 8) | b[12]);
	dev->dig_P5 = (int16_t)((b[15] << 8) | b[14]);
	dev->dig_P6 = (int16_t)((b[17] << 8) | b[16]);
	dev->dig_P7 = (int16_t)((b[19] << 8) | b[18]);
	dev->dig_P8 = (int16_t)((b[21] << 8) | b[20]);
	dev->dig_P9 = (int16_t)((b[23] << 8) | b[22]);

	return 0;
}

uint8_t BMP280_Init(BMP280_t *dev, I2C_HandleTypeDef *hi2c, uint8_t addr)
{
	uint8_t id = 0;

	dev->hi2c = hi2c;
	dev->addr = addr;

	if (BMP280_ReadReg(dev, BMP280_REG_ID, &id) != 0)
	{
		return 1; // i2c communication error
	}

	if (id != BMP280_CHIP_ID)
	{
		return 2; // wrong chip id
	}

	if (BMP280_ReadCalib(dev) != 0)
	{
		return 3; // calibration failed
	}

	return 0;
}

uint8_t BMP280_ReadID(BMP280_t *dev, uint8_t *id)
{
	return BMP280_ReadReg(dev, BMP280_REG_ID, id);
}

uint8_t BMP280_ReadStatus(BMP280_t *dev, uint8_t *status)
{
	return BMP280_ReadReg(dev, BMP280_REG_STATUS, status);
}

uint8_t BMP280_WaitMeasurement(BMP280_t *dev, uint32_t timeout_ms)
{
	uint8_t st;
	uint32_t t0 = HAL_GetTick();

	do
	{
		if (BMP280_ReadStatus(dev, &st) != 0)
		{
			return 1;
		}
		if ((st & (BMP280_STATUS_MEASURING | BMP280_STATUS_IM_UPDATE)) == 0)
		{
			return 0;
		}
	} while ((HAL_GetTick() - t0) <= timeout_ms);

	return 2;
}

uint8_t BMP280_Reset(BMP280_t *dev)
{
	if (BMP280_WriteReg(dev, BMP280_REG_RESET, BMP280_RESET_VALUE) != 0)
	{
		return 1;
	}
	HAL_Delay(3);                           // tstartup = 2 ms max
	return BMP280_WaitMeasurement(dev, 10); // im_update clears once the NVM copy is done
}

uint8_t BMP280_SetCtrlMeas(BMP280_t *dev, uint8_t osrs_t, uint8_t osrs_p, uint8_t mode)
{
	return BMP280_WriteReg(dev, BMP280_REG_CTRL_MEAS,
	                       (uint8_t)(((osrs_t & 7) << 5) | ((osrs_p & 7) << 2) | (mode & 3)));
}

uint8_t BMP280_SetConfig(BMP280_t *dev, uint8_t t_sb, uint8_t filter)
{
	return BMP280_WriteReg(dev, BMP280_REG_CONFIG, (uint8_t)(((t_sb & 7) << 5) | ((filter & 7) << 2)));
}

uint8_t BMP280_GetMode(BMP280_t *dev, uint8_t *mode)
{
	uint8_t v;

	if (BMP280_ReadReg(dev, BMP280_REG_CTRL_MEAS, &v) != 0)
	{
		return 1;
	}
	*mode = v & 3;
	return 0;
}

uint8_t BMP280_SetMode(BMP280_t *dev, uint8_t mode)
{
	uint8_t v;

	if (BMP280_ReadReg(dev, BMP280_REG_CTRL_MEAS, &v) != 0)
	{
		return 1;
	}
	return BMP280_WriteReg(dev, BMP280_REG_CTRL_MEAS, (uint8_t)((v & ~3) | (mode & 3)));
}

uint8_t BMP280_ReadRaw(BMP280_t *dev, int32_t *adc_T, int32_t *adc_P)
{
	uint8_t b[6];

	if (BMP280_ReadRegs(dev, BMP280_REG_PRESS_MSB, b, 6) != 0)
	{
		return 1;
	}
	*adc_P = ((int32_t)b[0] << 12) | ((int32_t)b[1] << 4) | (b[2] >> 4);
	*adc_T = ((int32_t)b[3] << 12) | ((int32_t)b[4] << 4) | (b[5] >> 4);
	return 0;
}

// --- Compensation: datasheet 3.11.3 (rev 1.1 code) and appendix 8 ---

int32_t BMP280_CompensateT_int32(BMP280_t *dev, int32_t adc_T)
{
	int32_t var1, var2;

	var1 = ((((adc_T >> 3) - ((int32_t)dev->dig_T1 << 1))) * ((int32_t)dev->dig_T2)) >> 11;
	var2 = (((((adc_T >> 4) - ((int32_t)dev->dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->dig_T1))) >> 12) *
	        ((int32_t)dev->dig_T3)) >>
	       14;
	dev->t_fine = var1 + var2;
	return (dev->t_fine * 5 + 128) >> 8;
}

uint32_t BMP280_CompensateP_int64(BMP280_t *dev, int32_t adc_P)
{
	int64_t var1, var2, p;

	var1 = ((int64_t)dev->t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dev->dig_P6;
	var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
	var2 = var2 + (((int64_t)dev->dig_P4) << 35);
	var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int64_t)dev->dig_P2) << 12);
	var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->dig_P1) >> 33;
	if (var1 == 0)
	{
		return 0; // avoid division by zero
	}
	p = 1048576 - adc_P;
	p = (((p << 31) - var2) * 3125) / var1;
	var1 = (((int64_t)dev->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
	var2 = (((int64_t)dev->dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dev->dig_P7) << 4);
	return (uint32_t)p;
}

uint32_t BMP280_CompensateP_int32(BMP280_t *dev, int32_t adc_P)
{
	int32_t var1, var2;
	uint32_t p;

	var1 = (dev->t_fine >> 1) - (int32_t)64000;
	var2 = (((var1 >> 2) * (var1 >> 2)) >> 11) * ((int32_t)dev->dig_P6);
	var2 = var2 + ((var1 * ((int32_t)dev->dig_P5)) << 1);
	var2 = (var2 >> 2) + (((int32_t)dev->dig_P4) << 16);
	var1 = (((dev->dig_P3 * (((var1 >> 2) * (var1 >> 2)) >> 13)) >> 3) + ((((int32_t)dev->dig_P2) * var1) >> 1)) >> 18;
	var1 = ((((32768 + var1)) * ((int32_t)dev->dig_P1)) >> 15);
	if (var1 == 0)
	{
		return 0;
	}
	p = (((uint32_t)(((int32_t)1048576) - adc_P) - (var2 >> 12))) * 3125;
	if (p < 0x80000000)
	{
		p = (p << 1) / ((uint32_t)var1);
	}
	else
	{
		p = (p / (uint32_t)var1) * 2;
	}
	var1 = (((int32_t)dev->dig_P9) * ((int32_t)(((p >> 3) * (p >> 3)) >> 13))) >> 12;
	var2 = (((int32_t)(p >> 2)) * ((int32_t)dev->dig_P8)) >> 13;
	p = (uint32_t)((int32_t)p + ((var1 + var2 + dev->dig_P7) >> 4));
	return p;
}

double BMP280_CompensateT_double(BMP280_t *dev, int32_t adc_T)
{
	double var1, var2;

	var1 = (((double)adc_T) / 16384.0 - ((double)dev->dig_T1) / 1024.0) * ((double)dev->dig_T2);
	var2 = ((((double)adc_T) / 131072.0 - ((double)dev->dig_T1) / 8192.0) *
	        (((double)adc_T) / 131072.0 - ((double)dev->dig_T1) / 8192.0)) *
	       ((double)dev->dig_T3);
	dev->t_fine = (int32_t)(var1 + var2);
	return (var1 + var2) / 5120.0;
}

double BMP280_CompensateP_double(BMP280_t *dev, int32_t adc_P)
{
	double var1, var2, p;

	var1 = ((double)dev->t_fine / 2.0) - 64000.0;
	var2 = var1 * var1 * ((double)dev->dig_P6) / 32768.0;
	var2 = var2 + var1 * ((double)dev->dig_P5) * 2.0;
	var2 = (var2 / 4.0) + (((double)dev->dig_P4) * 65536.0);
	var1 = (((double)dev->dig_P3) * var1 * var1 / 524288.0 + ((double)dev->dig_P2) * var1) / 524288.0;
	var1 = (1.0 + var1 / 32768.0) * ((double)dev->dig_P1);
	if (var1 == 0.0)
	{
		return 0;
	}
	p = 1048576.0 - (double)adc_P;
	p = (p - (var2 / 4096.0)) * 6250.0 / var1;
	var1 = ((double)dev->dig_P9) * p * p / 2147483648.0;
	var2 = p * ((double)dev->dig_P8) / 32768.0;
	return p + (var1 + var2 + ((double)dev->dig_P7)) / 16.0;
}

uint8_t BMP280_ReadTempPress(BMP280_t *dev, float *temp_c, float *press_pa)
{
	int32_t ut, up;

	if (BMP280_ReadRaw(dev, &ut, &up) != 0)
	{
		return 1;
	}
	// T first: it sets t_fine used by P
	*temp_c = BMP280_CompensateT_int32(dev, ut) / 100.0f;
	*press_pa = BMP280_CompensateP_int64(dev, up) / 256.0f;
	return 0;
}

uint8_t BMP280_MeasureForced(BMP280_t *dev, uint8_t osrs_t, uint8_t osrs_p, float *temp_c, float *press_pa)
{
	if (BMP280_SetCtrlMeas(dev, osrs_t, osrs_p, BMP280_MODE_FORCED) != 0)
	{
		return 1;
	}
	if (BMP280_WaitForced(dev, 100) != 0) // max 43.2 ms at x16/x2 (table 13)
	{
		return 2;
	}
	return BMP280_ReadTempPress(dev, temp_c, press_pa);
}

// Forced mode ends by itself: mode[1:0] returns to sleep (3.6.2). Unlike the measuring bit, this
// cannot be missed right after the trigger, when the conversion has not started yet.
uint8_t BMP280_WaitForced(BMP280_t *dev, uint32_t timeout_ms)
{
	uint8_t m;
	uint32_t t0 = HAL_GetTick();

	do
	{
		if (BMP280_GetMode(dev, &m) != 0)
		{
			return 1;
		}
		if (m == BMP280_MODE_SLEEP)
		{
			return BMP280_WaitMeasurement(dev, timeout_ms);
		}
	} while ((HAL_GetTick() - t0) <= timeout_ms);

	return 2;
}
