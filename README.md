# BMP280 I2C driver for STM32 (HAL)

C driver for the Bosch Sensortec **BMP280** pressure and temperature sensor over **I2C**, built on the STM32 HAL, with a complete on-target test suite checked against the datasheet ([docs/bst-bmp280-ds001.pdf](docs/bst-bmp280-ds001.pdf), rev 1.26).

Developed and tested on a **NUCLEO-F411RE** (STM32F411RE, I2C1 at 100 kHz, USART2 at 115200 for logs).

## Files

| File | Role |
|------|------|
| `src/bmp280.h` / `src/bmp280.c` | The driver |
| `src/bmp280_test.h` / `src/bmp280_test.c` | Test suite (printf report) |
| `docs/bst-bmp280-ds001.pdf` | Bosch datasheet (not covered by the MIT license) |

## Features

- Register access: `ReadReg`, `ReadRegs` (burst), `WriteReg`
- Chip identification (`0xD0` = `0x58`), soft reset (`0xE0` = `0xB6`), status register (`measuring`, `im_update`)
- Calibration read (`dig_T1..dig_P9`, 0x88..0x9F, table 17)
- Configuration: oversampling (x1..x16 or skipped), power mode (sleep / forced / normal), standby time (0.5 ms..4 s), IIR filter (off..16)
- Raw 20 bit read of pressure and temperature in one 6 byte burst
- Compensation from the datasheet (section 3.11.3 and appendix 8):
  - temperature, 32 bit integer (0.01 degC)
  - pressure, 64 bit integer (Pa, Q24.8)
  - pressure, 32 bit integer (for cores without 64 bit math, less accurate)
  - temperature and pressure in double precision
- Helpers: `ReadTempPress` (float degC / Pa) and `MeasureForced` (trigger, wait, read)

Not implemented: SPI (3 or 4 wire), altitude computation.

## Usage

Copy the four files from `src/` into your project and add that folder to the include path. `bmp280.h` includes `stm32f4xx_hal.h`, change it for another STM32 family.

```c
#include "bmp280.h"

BMP280_t bmp;

if (BMP280_Init(&bmp, &hi2c1, BMP280_I2C_ADDR) != 0) { /* 1: I2C, 2: wrong id, 3: calibration */ }

BMP280_SetConfig(&bmp, BMP280_STANDBY_62_5_MS, BMP280_FILTER_4);   /* write while in sleep mode */

/* one shot (forced mode) */
float t, p;
BMP280_MeasureForced(&bmp, BMP280_OSRS_X2, BMP280_OSRS_X16, &t, &p);   /* degC, Pa */

/* continuous (normal mode) */
BMP280_SetCtrlMeas(&bmp, BMP280_OSRS_X2, BMP280_OSRS_X16, BMP280_MODE_NORMAL);
HAL_Delay(100);
BMP280_ReadTempPress(&bmp, &t, &p);
```

The I2C address is the 8 bit HAL form (7 bit address shifted left): `0x77 << 1` (SDO high) by default, `0x76 << 1` when SDO is low. Edit `BMP280_I2C_ADDR` in `bmp280.h`.

All functions return `0` on success and a non zero value on error (HAL status or driver code as documented in the header).

### Notes from the datasheet

- `config` (0xF5) writes can be ignored in normal mode: write it in sleep mode (`BMP280_SetConfig` before `SetCtrlMeas`).
- After a forced trigger the `measuring` bit is not yet set for a short time. `BMP280_WaitForced` waits for `mode` to return to sleep instead, which cannot be missed.
- Temperature must be compensated before pressure: it computes `t_fine`, stored in the `BMP280_t` structure.
- `printf` with `--specs=nano.specs` has no float support by default. The test suite prints fixed point values for that reason.

## Test suite

```c
#include "bmp280_test.h"

BMP280_Test_RunAll(&bmp);   /* prints a PASS / FAIL line per test, returns the number of failed checks */
```

`printf` must be redirected (for example `__io_putchar` to a UART).

| Test | What is checked | Datasheet |
|------|-----------------|-----------|
| Identity | chip id 0x58, NACK and error code on a wrong address | 4.3.1 |
| Reset | 0xB6 resets ctrl_meas and config to 0x00, other values ignored, reset register reads 0x00 | 4.3.2 |
| Calibration | 12 coefficients against raw bytes (LSB first), burst vs single reads | table 17 |
| Registers | read back of every osrs_t x osrs_p, every t_sb x filter, `SetMode` / `GetMode` | tables 6, 10, 11, 20-23 |
| Compensation | known answer plus int32 / int64 / double agreement over the full raw range | 3.11.3, 8 |
| Forced | back to sleep after a conversion, conversion time within the maximum of table 13, `measuring` bit | 3.6.2, 3.8.1 |
| Skipped | raw value `0x80000` when osrs is 0 for T, P or both | tables 21, 22 |
| Normal | about 29 conversions in 2 s at x1/x1 with t_sb = 62.5 ms (14.71 Hz), sleep stops them | 3.6.3, table 14 |
| Filter | IIR filter 16 reduces the pressure noise compared to filter off | 3.3.3 |
| Ranges | T and P in the operating range, repeatability over 10 shots | table 2 |

### Result on hardware (NUCLEO-F411RE, room conditions)

```
[PASS] Identity
[PASS] Reset
[PASS] Calibration
[PASS] Registers
[PASS] Compensation
[PASS] Forced
[PASS] Skipped
[PASS] Normal
    pressure std-dev: filter off 2517 mPa, filter 16 689 mPa
[PASS] Filter
    T=2130 cC  P=101455 Pa
[PASS] Ranges
===== ALL PASSED (0 failed checks) =====
```

### Limits of the tests

- The compensation reference answer (25.08 degC, about 100653 Pa for the typical Bosch coefficients, adc_T = 519888, adc_P = 415148) is not printed in the datasheet. It comes from the public Bosch examples. The three implementations are also checked against each other.
- The 32 bit pressure formula differs by about 4 Pa from the double one, the test tolerance is 8 Pa.
- Timing checks use the 1 ms HAL tick and add a 2 ms margin. Noise and filter checks use loose bounds: they depend on the environment.
- The SPI interface and the I2C address 0x76 are not tested.

## License

The code is released under the [MIT License](LICENSE). The datasheet is copyright Bosch Sensortec GmbH and is included for reference only.
