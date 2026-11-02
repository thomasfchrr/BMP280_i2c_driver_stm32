/*
 * bmp280_test.h
 *
 * Full test suite of the BMP280 driver, checked against datasheet BST-BMP280-DS001.
 * Results are printed with printf (UART2 on this board).
 */

#ifndef BMP280_BMP280_TEST_H_
#define BMP280_BMP280_TEST_H_

#include "bmp280.h"

// Each test returns the number of failed checks (0 = pass)
uint32_t BMP280_Test_Identity(BMP280_t *dev);     // chip id 0xD0 = 0x58, wrong-address handling
uint32_t BMP280_Test_Reset(BMP280_t *dev);        // reset register 0xE0, POR values, status
uint32_t BMP280_Test_Calibration(BMP280_t *dev);  // table 17, burst read vs single reads
uint32_t BMP280_Test_Registers(BMP280_t *dev);    // ctrl_meas / config read-back (tables 20-23)
uint32_t BMP280_Test_Compensation(BMP280_t *dev); // known answer + int32/int64/double cross-check
uint32_t BMP280_Test_Forced(BMP280_t *dev);       // forced mode, return to sleep, timing (table 13)
uint32_t BMP280_Test_Skipped(BMP280_t *dev);      // osrs = 0 -> raw 0x80000
uint32_t BMP280_Test_Normal(BMP280_t *dev);       // normal mode, t_sb (table 11)
uint32_t BMP280_Test_Filter(BMP280_t *dev);       // IIR filter (table 6)
uint32_t BMP280_Test_Ranges(BMP280_t *dev);       // measured values within operating range

// Runs everything, prints a summary, returns total number of failed checks
uint32_t BMP280_Test_RunAll(BMP280_t *dev);

#endif /* BMP280_BMP280_TEST_H_ */
