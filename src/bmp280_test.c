/*
 * bmp280_test.c
 *
 * Datasheet references: BST-BMP280-DS001 rev 1.26
 */

#include "bmp280_test.h"
#include <stdio.h>
#include <math.h>
#include <stdlib.h>

static uint32_t g_fail; // failed checks of the running test

#define CHECK(cond, ...)                                                                                               \
	do                                                                                                                 \
	{                                                                                                                  \
		if (!(cond))                                                                                                   \
		{                                                                                                              \
			g_fail++;                                                                                                  \
			printf("    FAIL L%d: ", __LINE__);                                                                        \
			printf(__VA_ARGS__);                                                                                       \
			printf("\r\n");                                                                                            \
		}                                                                                                              \
	} while (0)

static void report(const char *name, uint32_t fails)
{
	printf("[%s] %s\r\n", fails ? "FAIL" : "PASS", name);
}

static uint8_t cfg(BMP280_t *dev, uint8_t ot, uint8_t op, uint8_t mode, uint8_t tsb, uint8_t filt)
{
	// config must be written while sleeping (4.3.5)
	return BMP280_SetCtrlMeas(dev, 0, 0, BMP280_MODE_SLEEP) | BMP280_SetConfig(dev, tsb, filt) |
	       BMP280_SetCtrlMeas(dev, ot, op, mode);
}

/* ------------------------------------------------------------------ */

uint32_t BMP280_Test_Identity(BMP280_t *dev)
{
	uint8_t id = 0;
	BMP280_t bad = *dev;

	g_fail = 0;
	CHECK(BMP280_ReadID(dev, &id) == 0, "I2C read id");
	CHECK(id == BMP280_CHIP_ID, "id=0x%02X expected 0x58 (reg 0xD0)", id);

	// unused address on the bus: must report an error and not hang
	bad.addr = (uint8_t)(0x50 << 1);
	CHECK(BMP280_ReadID(&bad, &id) != 0, "wrong address must NACK");
	CHECK(BMP280_Init(&bad, dev->hi2c, bad.addr) == 1, "Init on wrong address must return 1");
	HAL_I2C_DeInit(dev->hi2c);
	HAL_I2C_Init(dev->hi2c); // clear any bus error state
	report("Identity", g_fail);
	return g_fail;
}

uint32_t BMP280_Test_Reset(BMP280_t *dev)
{
	uint8_t v = 0xFF;

	g_fail = 0;
	// dirty the registers, then reset: POR values are ctrl_meas=0x00, config=0x00 (table 18)
	CHECK(cfg(dev, BMP280_OSRS_X2, BMP280_OSRS_X4, BMP280_MODE_SLEEP, BMP280_STANDBY_1000_MS, BMP280_FILTER_8) == 0,
	      "setup");
	CHECK(BMP280_Reset(dev) == 0, "reset");
	CHECK(BMP280_ReadReg(dev, BMP280_REG_CTRL_MEAS, &v) == 0 && v == 0x00, "ctrl_meas after reset=0x%02X", v);
	CHECK(BMP280_ReadReg(dev, BMP280_REG_CONFIG, &v) == 0 && v == 0x00, "config after reset=0x%02X", v);
	CHECK(BMP280_ReadReg(dev, BMP280_REG_RESET, &v) == 0 && v == 0x00, "reset reg reads 0x%02X, expected 0x00 (4.3.2)",
	      v);
	CHECK(BMP280_ReadStatus(dev, &v) == 0 && (v & (BMP280_STATUS_MEASURING | BMP280_STATUS_IM_UPDATE)) == 0,
	      "status=0x%02X", v);

	// writing a value other than 0xB6 has no effect (4.3.2)
	BMP280_SetCtrlMeas(dev, BMP280_OSRS_X1, BMP280_OSRS_X1, BMP280_MODE_SLEEP);
	BMP280_WriteReg(dev, BMP280_REG_RESET, 0x00);
	CHECK(BMP280_ReadReg(dev, BMP280_REG_CTRL_MEAS, &v) == 0 && v == 0x24,
	      "reset with 0x00 must be ignored, ctrl_meas=0x%02X", v);

	// registers 0x88.. keep the same values after reset (NVM)
	CHECK(BMP280_ReadCalib(dev) == 0 && dev->dig_T1 != 0, "calib after reset");
	BMP280_Reset(dev);
	report("Reset", g_fail);
	return g_fail;
}

uint32_t BMP280_Test_Calibration(BMP280_t *dev)
{
	uint8_t b[26], one;
	BMP280_t c = *dev;

	g_fail = 0;
	CHECK(BMP280_ReadCalib(&c) == 0, "calib read");
	CHECK(BMP280_ReadRegs(dev, BMP280_REG_CALIB_START, b, 26) == 0, "burst 0x88..0xA1");

	// every coefficient is stored LSB first (table 17)
	CHECK(c.dig_T1 == (uint16_t)(b[0] | (b[1] << 8)), "T1");
	CHECK(c.dig_T2 == (int16_t)(b[2] | (b[3] << 8)), "T2");
	CHECK(c.dig_T3 == (int16_t)(b[4] | (b[5] << 8)), "T3");
	CHECK(c.dig_P1 == (uint16_t)(b[6] | (b[7] << 8)), "P1");
	CHECK(c.dig_P2 == (int16_t)(b[8] | (b[9] << 8)), "P2");
	CHECK(c.dig_P3 == (int16_t)(b[10] | (b[11] << 8)), "P3");
	CHECK(c.dig_P4 == (int16_t)(b[12] | (b[13] << 8)), "P4");
	CHECK(c.dig_P5 == (int16_t)(b[14] | (b[15] << 8)), "P5");
	CHECK(c.dig_P6 == (int16_t)(b[16] | (b[17] << 8)), "P6");
	CHECK(c.dig_P7 == (int16_t)(b[18] | (b[19] << 8)), "P7");
	CHECK(c.dig_P8 == (int16_t)(b[20] | (b[21] << 8)), "P8");
	CHECK(c.dig_P9 == (int16_t)(b[22] | (b[23] << 8)), "P9");

	// single byte reads give the same bytes as the burst
	for (uint8_t i = 0; i < 24; i++)
	{
		CHECK(BMP280_ReadReg(dev, (uint8_t)(BMP280_REG_CALIB_START + i), &one) == 0 && one == b[i], "byte 0x%02X",
		      0x88 + i);
	}
	// the coefficients are never all 0x00 or 0xFF on a programmed device
	CHECK(c.dig_T1 != 0 && c.dig_T1 != 0xFFFF && c.dig_P1 != 0 && c.dig_P1 != 0xFFFF, "T1=%u P1=%u", c.dig_T1,
	      c.dig_P1);
	report("Calibration", g_fail);
	return g_fail;
}

uint32_t BMP280_Test_Registers(BMP280_t *dev)
{
	uint8_t v;

	g_fail = 0;
	BMP280_Reset(dev);

	// ctrl_meas: every osrs_t x osrs_p, sleep mode so the register is stable (tables 20-22)
	for (uint8_t t = 0; t <= 5; t++)
	{
		for (uint8_t p = 0; p <= 5; p++)
		{
			uint8_t exp = (uint8_t)((t << 5) | (p << 2));
			CHECK(BMP280_SetCtrlMeas(dev, t, p, BMP280_MODE_SLEEP) == 0, "write");
			CHECK(BMP280_ReadReg(dev, BMP280_REG_CTRL_MEAS, &v) == 0 && v == exp,
			      "ctrl_meas t=%u p=%u read 0x%02X exp 0x%02X", t, p, v, exp);
		}
	}

	// config: every t_sb x filter (tables 6, 11, 23)
	for (uint8_t s = 0; s <= 7; s++)
	{
		for (uint8_t f = 0; f <= 4; f++)
		{
			uint8_t exp = (uint8_t)((s << 5) | (f << 2));
			CHECK(BMP280_SetConfig(dev, s, f) == 0, "write");
			CHECK(BMP280_ReadReg(dev, BMP280_REG_CONFIG, &v) == 0 && v == exp,
			      "config s=%u f=%u read 0x%02X exp 0x%02X", s, f, v, exp);
		}
	}

	// SetMode only touches mode[1:0]; mode 10 is an alias of forced (table 10)
	BMP280_SetConfig(dev, 0, 0);
	BMP280_SetCtrlMeas(dev, BMP280_OSRS_X2, BMP280_OSRS_X16, BMP280_MODE_SLEEP);
	CHECK(BMP280_SetMode(dev, BMP280_MODE_NORMAL) == 0, "set normal");
	CHECK(BMP280_ReadReg(dev, BMP280_REG_CTRL_MEAS, &v) == 0 && v == ((2 << 5) | (5 << 2) | 3), "SetMode normal 0x%02X",
	      v);
	CHECK(BMP280_GetMode(dev, &v) == 0 && v == BMP280_MODE_NORMAL, "GetMode=%u", v);
	BMP280_SetMode(dev, BMP280_MODE_SLEEP);
	HAL_Delay(100);
	CHECK(BMP280_GetMode(dev, &v) == 0 && v == BMP280_MODE_SLEEP, "back to sleep=%u", v);

	BMP280_Reset(dev);
	report("Registers", g_fail);
	return g_fail;
}

uint32_t BMP280_Test_Compensation(BMP280_t *dev)
{
	// Known answer: typical Bosch coefficients, T = 25.08 degC, P = 100653.27 Pa
	BMP280_t k = {0};
	int32_t t32;
	uint32_t p64, p32;
	double td, pd;

	(void)dev;
	g_fail = 0;
	k.dig_T1 = 27504;
	k.dig_T2 = 26435;
	k.dig_T3 = -1000;
	k.dig_P1 = 36477;
	k.dig_P2 = -10685;
	k.dig_P3 = 3024;
	k.dig_P4 = 2855;
	k.dig_P5 = 140;
	k.dig_P6 = -7;
	k.dig_P7 = 15500;
	k.dig_P8 = -14600;
	k.dig_P9 = 6000;

	t32 = BMP280_CompensateT_int32(&k, 519888);
	CHECK(t32 == 2508, "T int32=%ld expected 2508", (long)t32);
	p64 = BMP280_CompensateP_int64(&k, 415148);
	CHECK(labs((long)p64 - 25767236L) <= 8, "P int64=%lu expected ~25767236 (100653.27 Pa)", (unsigned long)p64);
	p32 = BMP280_CompensateP_int32(&k, 415148);
	CHECK(labs((long)p32 - 100653) <= 5, // 32 bit variant is less accurate (appendix 8.2)
	      "P int32=%lu expected 100653", (unsigned long)p32);
	td = BMP280_CompensateT_double(&k, 519888);
	pd = BMP280_CompensateP_double(&k, 415148);
	CHECK(fabs(td - 25.08) < 0.01, "T double=%ld", (long)(td * 100));
	CHECK(fabs(pd - 100653.27) < 1.0, "P double=%ld", (long)pd);

	// sweep: the 3 implementations must agree over the whole range of the sensor
	for (int32_t adc_t = 400000; adc_t <= 600000; adc_t += 20000)
	{
		int32_t ti = BMP280_CompensateT_int32(&k, adc_t);
		double tdd = BMP280_CompensateT_double(&k, adc_t);
		CHECK(fabs(ti / 100.0 - tdd) < 0.02, "T sweep adc=%ld int=%ld dbl=%ld", (long)adc_t, (long)ti,
		      (long)(tdd * 100));
		for (int32_t adc_p = 300000; adc_p <= 560000; adc_p += 20000)
		{
			BMP280_CompensateT_int32(&k, adc_t);
			uint32_t a = BMP280_CompensateP_int64(&k, adc_p);
			uint32_t b = BMP280_CompensateP_int32(&k, adc_p);
			BMP280_CompensateT_double(&k, adc_t);
			double c = BMP280_CompensateP_double(&k, adc_p);
			CHECK(fabs(a / 256.0 - c) < 1.0, "P64 vs dbl adc_t=%ld adc_p=%ld %lu %ld", (long)adc_t, (long)adc_p,
			      (unsigned long)a, (long)(c * 256));
			CHECK(fabs((double)b - c) < 8.0, "P32 vs dbl adc_t=%ld adc_p=%ld %lu %ld", (long)adc_t, (long)adc_p,
			      (unsigned long)b, (long)c);
		}
	}
	report("Compensation", g_fail);
	return g_fail;
}

uint32_t BMP280_Test_Forced(BMP280_t *dev)
{
	// osrs_t, osrs_p, max measurement time [ms] from table 13
	static const uint8_t cases[][3] = {{1, 1, 7}, {1, 2, 9}, {1, 3, 14}, {1, 4, 23}, {2, 5, 44}};
	uint8_t v, st;
	float t, p;

	g_fail = 0;
	BMP280_Reset(dev);
	BMP280_SetConfig(dev, 0, BMP280_FILTER_OFF);

	for (uint32_t i = 0; i < sizeof(cases) / sizeof(cases[0]); i++)
	{
		uint32_t t0, dt;
		CHECK(BMP280_SetCtrlMeas(dev, cases[i][0], cases[i][1], BMP280_MODE_FORCED) == 0, "trigger");
		t0 = HAL_GetTick();
		CHECK(BMP280_WaitForced(dev, 100) == 0, "wait");
		dt = HAL_GetTick() - t0;
		// the I2C transactions add ~1 ms of polling granularity
		CHECK(dt <= (uint32_t)cases[i][2] + 2, "x%u/x%u took %lu ms, max %u (table 13)", cases[i][0], cases[i][1],
		      (unsigned long)dt, cases[i][2]);
		// forced mode returns to sleep by itself (3.6.2)
		BMP280_ReadReg(dev, BMP280_REG_CTRL_MEAS, &v);
		CHECK((v & 3) == BMP280_MODE_SLEEP, "mode after forced=%u, osrs bits=0x%02X", v & 3, v & 0xFC);
		CHECK((v & 0xFC) == (uint8_t)((cases[i][0] << 5) | (cases[i][1] << 2)), "osrs kept 0x%02X", v);
	}

	// measuring bit is high during a conversion (table 19): x16/x2 lasts ~37 ms
	BMP280_SetCtrlMeas(dev, 2, 5, BMP280_MODE_FORCED);
	HAL_Delay(5);
	CHECK(BMP280_ReadStatus(dev, &st) == 0 && (st & BMP280_STATUS_MEASURING), "measuring bit not set, status=0x%02X",
	      st);
	BMP280_WaitForced(dev, 100);

	// high-level helper
	CHECK(BMP280_MeasureForced(dev, BMP280_OSRS_X2, BMP280_OSRS_X16, &t, &p) == 0, "MeasureForced");
	printf("    forced x16/x2: T=%ld cC P=%ld Pa\r\n", (long)(t * 100), (long)p);

	// a second forced shot gives a new result: re-triggering works (mode must be re-armed each time)
	CHECK(BMP280_MeasureForced(dev, BMP280_OSRS_X1, BMP280_OSRS_X1, &t, &p) == 0, "second MeasureForced");
	report("Forced", g_fail);
	return g_fail;
}

uint32_t BMP280_Test_Skipped(BMP280_t *dev)
{
	int32_t ut, up;

	g_fail = 0;
	BMP280_Reset(dev);

	// pressure skipped -> up = 0x80000 (table 21)
	BMP280_SetCtrlMeas(dev, BMP280_OSRS_X1, BMP280_OSRS_SKIP, BMP280_MODE_FORCED);
	BMP280_WaitForced(dev, 100);
	CHECK(BMP280_ReadRaw(dev, &ut, &up) == 0, "raw");
	CHECK(up == BMP280_RAW_SKIPPED, "up=0x%05lX expected 0x80000", (unsigned long)up);
	CHECK(ut != BMP280_RAW_SKIPPED, "ut must be a real value");

	// temperature skipped -> ut = 0x80000 (table 22)
	BMP280_SetCtrlMeas(dev, BMP280_OSRS_SKIP, BMP280_OSRS_X1, BMP280_MODE_FORCED);
	BMP280_WaitForced(dev, 100);
	CHECK(BMP280_ReadRaw(dev, &ut, &up) == 0, "raw");
	CHECK(ut == BMP280_RAW_SKIPPED, "ut=0x%05lX expected 0x80000", (unsigned long)ut);
	CHECK(up != BMP280_RAW_SKIPPED, "up must be a real value");

	// both skipped
	BMP280_SetCtrlMeas(dev, BMP280_OSRS_SKIP, BMP280_OSRS_SKIP, BMP280_MODE_FORCED);
	BMP280_WaitForced(dev, 100);
	CHECK(BMP280_ReadRaw(dev, &ut, &up) == 0 && ut == BMP280_RAW_SKIPPED && up == BMP280_RAW_SKIPPED,
	      "both skipped ut=0x%05lX up=0x%05lX", (unsigned long)ut, (unsigned long)up);
	report("Skipped", g_fail);
	return g_fail;
}

uint32_t BMP280_Test_Normal(BMP280_t *dev)
{
	// x1/x1 + t_sb 62.5 ms -> ODR 14.71 Hz (table 14)
	uint8_t st, prev = 0, mode;
	uint32_t edges = 0, t0;

	g_fail = 0;
	BMP280_Reset(dev);
	CHECK(cfg(dev, BMP280_OSRS_X1, BMP280_OSRS_X1, BMP280_MODE_NORMAL, BMP280_STANDBY_62_5_MS, BMP280_FILTER_OFF) == 0,
	      "setup");
	CHECK(BMP280_GetMode(dev, &mode) == 0 && mode == BMP280_MODE_NORMAL, "normal mode stays set, mode=%u", mode);

	// count conversions through the rising edges of the measuring bit
	t0 = HAL_GetTick();
	while (HAL_GetTick() - t0 < 2000)
	{
		if (BMP280_ReadStatus(dev, &st) != 0)
		{
			CHECK(0, "status read");
			break;
		}
		st &= BMP280_STATUS_MEASURING;
		if (st && !prev)
			edges++;
		prev = st;
	}
	// 2 s * 14.71 Hz = 29.4; polling may miss a few 5.5 ms windows, so accept a band
	CHECK(edges >= 24 && edges <= 32, "%lu conversions in 2 s, expected ~29 (14.71 Hz)", (unsigned long)edges);
	printf("    normal x1/x1 tsb=62.5ms: %lu conversions / 2 s\r\n", (unsigned long)edges);

	// normal mode is not left on its own
	BMP280_GetMode(dev, &mode);
	CHECK(mode == BMP280_MODE_NORMAL, "still normal=%u", mode);

	// data available without further write (3.6.3)
	float t, p;
	CHECK(BMP280_ReadTempPress(dev, &t, &p) == 0, "read");

	// back to sleep stops conversions
	BMP280_SetMode(dev, BMP280_MODE_SLEEP);
	HAL_Delay(50);
	edges = 0;
	t0 = HAL_GetTick();
	while (HAL_GetTick() - t0 < 300)
	{
		BMP280_ReadStatus(dev, &st);
		if (st & BMP280_STATUS_MEASURING)
			edges++;
	}
	CHECK(edges == 0, "sleep mode still measuring (%lu)", (unsigned long)edges);
	report("Normal", g_fail);
	return g_fail;
}

static float stddev_press(BMP280_t *dev, uint8_t filter, uint32_t *settle)
{
	float v[20], t, mean = 0, var = 0;

	BMP280_Reset(dev);
	cfg(dev, BMP280_OSRS_X1, BMP280_OSRS_X4, BMP280_MODE_NORMAL, BMP280_STANDBY_62_5_MS, filter);
	HAL_Delay(100);
	for (int i = 0; i < 20; i++)
	{
		HAL_Delay(80); // > one cycle (62.5 + 11.5 ms)
		BMP280_ReadTempPress(dev, &t, &v[i]);
		mean += v[i] / 20;
	}
	for (int i = 0; i < 20; i++)
		var += (v[i] - mean) * (v[i] - mean) / 19;
	(void)settle;
	return sqrtf(var);
}

uint32_t BMP280_Test_Filter(BMP280_t *dev)
{
	g_fail = 0;
	float off = stddev_press(dev, BMP280_FILTER_OFF, 0);
	float f16 = stddev_press(dev, BMP280_FILTER_16, 0);

	printf("    pressure std-dev: filter off %ld mPa, filter 16 %ld mPa\r\n", (long)(off * 1000), (long)(f16 * 1000));
	// IIR can only lower the variation of a quasi-static signal
	CHECK(f16 <= off + 0.05f, "filter 16 noisier than filter off");
	CHECK(off < 5.0f, "unfiltered noise too high: %ld mPa", (long)(off * 1000));

	// writing filter resets the filter memory, the register keeps its value (3.3.3)
	uint8_t v;
	BMP280_Reset(dev);
	BMP280_SetConfig(dev, 0, BMP280_FILTER_4);
	CHECK(BMP280_ReadReg(dev, BMP280_REG_CONFIG, &v) == 0 && v == (BMP280_FILTER_4 << 2), "config=0x%02X", v);
	BMP280_Reset(dev);
	report("Filter", g_fail);
	return g_fail;
}

uint32_t BMP280_Test_Ranges(BMP280_t *dev)
{
	float t, p;

	g_fail = 0;
	BMP280_Reset(dev);
	BMP280_SetConfig(dev, 0, BMP280_FILTER_OFF);

	// operating range: -40..85 degC, 300..1100 hPa (datasheet table 2)
	CHECK(BMP280_MeasureForced(dev, BMP280_OSRS_X2, BMP280_OSRS_X16, &t, &p) == 0, "measure");
	printf("    T=%ld cC  P=%ld Pa\r\n", (long)(t * 100), (long)p);
	CHECK(t > -40.0f && t < 85.0f, "T out of range %ld cC", (long)(t * 100));
	CHECK(p > 30000.0f && p < 110000.0f, "P out of range %ld Pa", (long)p);

	// t_fine is the fine temperature: T = (t_fine*5+128)>>8 (3.11.3)
	CHECK(labs(((dev->t_fine * 5 + 128) >> 8) - (long)(t * 100.0f)) <= 1, "t_fine relation");

	// repeatability: 10 forced measurements must stay close (datasheet RMS noise 0.002 degC / 1.3 Pa)
	float tmin = 1000, tmax = -1000, pmin = 1e9f, pmax = 0;
	for (int i = 0; i < 10; i++)
	{
		CHECK(BMP280_MeasureForced(dev, BMP280_OSRS_X2, BMP280_OSRS_X16, &t, &p) == 0, "measure");
		if (t < tmin)
			tmin = t;
		if (t > tmax)
			tmax = t;
		if (p < pmin)
			pmin = p;
		if (p > pmax)
			pmax = p;
	}
	printf("    10 shots: dT=%ld mC dP=%ld cPa\r\n", (long)((tmax - tmin) * 1000), (long)((pmax - pmin) * 100));
	CHECK(tmax - tmin < 0.5f, "T spread %ld mC", (long)((tmax - tmin) * 1000));
	CHECK(pmax - pmin < 20.0f, "P spread %ld cPa", (long)((pmax - pmin) * 100));
	report("Ranges", g_fail);
	return g_fail;
}

uint32_t BMP280_Test_RunAll(BMP280_t *dev)
{
	uint32_t total = 0;

	printf("\r\n===== BMP280 driver test =====\r\n");
	total += BMP280_Test_Identity(dev);
	total += BMP280_Test_Reset(dev);
	total += BMP280_Test_Calibration(dev);
	total += BMP280_Test_Registers(dev);
	total += BMP280_Test_Compensation(dev);
	total += BMP280_Test_Forced(dev);
	total += BMP280_Test_Skipped(dev);
	total += BMP280_Test_Normal(dev);
	total += BMP280_Test_Filter(dev);
	total += BMP280_Test_Ranges(dev);
	BMP280_Reset(dev);
	printf("===== %s (%lu failed checks) =====\r\n", total ? "FAILED" : "ALL PASSED", (unsigned long)total);
	return total;
}
