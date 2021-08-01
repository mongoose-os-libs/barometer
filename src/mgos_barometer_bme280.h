/*
 * Copyright 2018 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include "mgos.h"
#include "mgos_barometer_internal.h"

// DevID: 0x56/0x57 are samples of BMP280; 0x58 is mass production BMP280; 0x60 is BME280
#define BME280_REG_DEVID                           (0xD0) /* Chip ID Register */
#define BME280_REG_RESET                           (0xE0) /* Softreset Register */
#define BME280_REG_CTRL_HUM                        (0xF2) /* Humidity oversampling */
#define BME280_REG_STATUS                          (0xF3) /* Status Register */
#define BME280_REG_CTRL_MEAS                       (0xF4) /* Ctrl Measure Register */
#define BME280_REG_CONFIG                          (0xF5) /* Configuration Register */
#define BME280_REG_PRESSURE_MSB                    (0xF7) /* Pressure MSB Register */
#define BME280_REG_PRESSURE_LSB                    (0xF8) /* Pressure LSB Register */
#define BME280_REG_PRESSURE_XLSB                   (0xF9) /* Pressure XLSB Register */
#define BME280_REG_TEMPERATURE_MSB                 (0xFA) /* Temperature MSB Reg */
#define BME280_REG_TEMPERATURE_LSB                 (0xFB) /* Temperature LSB Reg */
#define BME280_REG_TEMPERATURE_XLSB                (0xFC) /* Temperature XLSB Reg */
#define BME280_REG_HUMIDITY_MSB                    (0xFD) /* Humidity MSB Reg (BME280 only)*/
#define BME280_REG_HUMIDITY_LSB                    (0xFE) /* Humidity LSB Reg (BME280 only)*/
#define BME280_MODE_SLEEP                          (0x00)
#define BME280_MODE_FORCED                         (0x01)
#define BME280_MODE_NORMAL                         (0x03)

#define BME280_REG_TEMPERATURE_CALIB_DIG_T1_LSB    (0x88)
#define BME280_REG_HUMIDITY_CALIB_ADDR             (0xE1)

#define BME280_OVERSAMP_SKIPPED                    (0x00)
#define BME280_OVERSAMP_1X                         (0x01)
#define BME280_OVERSAMP_2X                         (0x02)
#define BME280_OVERSAMP_4X                         (0x03)
#define BME280_OVERSAMP_8X                         (0x04)
#define BME280_OVERSAMP_16X                        (0x05)

#define BME280_STANDBY_500us                       (0x00)
#define BME280_STANDBY_62ms                        (0x01)
#define BME280_STANDBY_125ms                       (0x02)
#define BME280_STANDBY_250ms                       (0x03)
#define BME280_STANDBY_500ms                       (0x04)
#define BME280_STANDBY_1000ms                      (0x05)
// Note: Datasheet defines 110 == 10ms, 111 == 20ms, but BME280 and BMP280
// implement this differently.
#define BME280_FILTER_OFF                          (0x00)
#define BME280_FILTER_2X                           (0x01)
#define BME280_FILTER_4X                           (0x02)
#define BME280_FILTER_8X                           (0x03)
#define BME280_FILTER_16X                          (0x04)


struct mgos_barometer_bme280_calib_data {
  // Calibration data:
  uint16_t dig_T1;
  int16_t  dig_T2;
  int16_t  dig_T3;
  uint16_t dig_P1;
  int16_t  dig_P2;
  int16_t  dig_P3;
  int16_t  dig_P4;
  int16_t  dig_P5;
  int16_t  dig_P6;
  int16_t  dig_P7;
  int16_t  dig_P8;
  int16_t  dig_P9;

  // Additional calibration data for BME280
  uint8_t  dig_H1;
  int16_t  dig_H2;
  uint8_t  dig_H3;
  int16_t  dig_H4; // Note: this is 0xE4 / 0xE5[3:0]
  int16_t  dig_H5; // Note: this is 0xE5[7:4] / 0xE6
  int8_t   dig_H6;
};

struct mgos_barometer_bme280_data {
  struct mgos_barometer_bme280_calib_data calib;

  float                                   humidity;
};

bool mgos_barometer_bme280_detect(struct mgos_barometer *dev);
bool mgos_barometer_bme280_create(struct mgos_barometer *dev);
bool mgos_barometer_bme280_destroy(struct mgos_barometer *dev);
bool mgos_barometer_bme280_read(struct mgos_barometer *dev);
