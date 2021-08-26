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

#include "mgos_barometer_bme280.h"
#include "mgos_i2c.h"

#define BME280_CONCAT_BYTES(msb, lsb)            (((uint16_t)msb << 8) | (uint16_t)lsb)

// Datasheet:
// https://cdn-shop.adafruit.com/datasheets/BST-BME280_DS001-10.pdf
//


bool mgos_barometer_bme280_detect(struct mgos_barometer *dev) {
  int val;

  if (!dev) {
    return false;
  }

  if ((val = mgos_i2c_read_reg_b(dev->i2c, dev->i2caddr, BME280_REG_DEVID)) < 0) {
    return false;
  }

  LOG(LL_DEBUG, ("Detecting BMx280 %d, reg b is: %d", dev->i2caddr, val));


  if (val == 0x56 || val == 0x57) {
    LOG(LL_INFO, ("Preproduction version of BMP280 detected (0x%02x)", val));
    return true;
  }
  if (val == 0x58) { // Mass production BMP280
    return true;
  }

  if (val == 0x60) { // Mass production BME280
    dev->capabilities |= MGOS_BAROMETER_CAP_HYGROMETER;
    return true;
  }

  return true;
}

bool mgos_barometer_bme280_create(struct mgos_barometer *dev) {
  struct mgos_barometer_bme280_data *bme280_data;

  if (!dev) {
    return false;
  }

  bme280_data = calloc(1, sizeof(struct mgos_barometer_bme280_data));
  if (!bme280_data) {
    return false;
  }
  dev->user_data = bme280_data;

  // Reset device
  if (!mgos_i2c_write_reg_b(dev->i2c, dev->i2caddr, BME280_REG_RESET, 0xB6)) {
    return false;
  }
  mgos_usleep(10000);

  // Read calibration data
  if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, BME280_REG_TEMPERATURE_CALIB_DIG_T1_LSB, 24, (uint8_t *)bme280_data)) {
    free(dev->user_data);
    return false;
  }

  if( dev->capabilities && MGOS_BAROMETER_CAP_HYGROMETER ) {
    uint8_t reg_data[ 7 ];
    int16_t dig_h4_lsb;
    int16_t dig_h4_msb;
    int16_t dig_h5_lsb;
    int16_t dig_h5_msb;

    if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, BME280_REG_HUMIDITY_CALIB_ADDR, 7, reg_data)) {
      free(dev->user_data);
      return false;
    }

    bme280_data->calib.dig_H2 = (int16_t)BME280_CONCAT_BYTES(reg_data[1], reg_data[0]);
    bme280_data->calib.dig_H3 = reg_data[2];
    dig_h4_msb = (int16_t)(int8_t)reg_data[3] * 16;
    dig_h4_lsb = (int16_t)(reg_data[4] & 0x0F);
    bme280_data->calib.dig_H4 = dig_h4_msb | dig_h4_lsb;
    dig_h5_msb = (int16_t)(int8_t)reg_data[5] * 16;
    dig_h5_lsb = (int16_t)(reg_data[4] >> 4);
    bme280_data->calib.dig_H5 = dig_h5_msb | dig_h5_lsb;
    bme280_data->calib.dig_H6 = (int8_t)reg_data[6];
  }

  // SPI | 0.5ms period | 16X IIR filter
  if (!mgos_i2c_write_reg_b(dev->i2c, dev->i2caddr, BME280_REG_CONFIG, 0x00 | BME280_STANDBY_500us << 2 | BME280_FILTER_16X << 5)) {
    free(dev->user_data);
    return false;
  }
  mgos_usleep(10000);


  // Humidity OS
  if (!mgos_i2c_write_reg_b(dev->i2c, dev->i2caddr, BME280_REG_CTRL_HUM, BME280_OVERSAMP_2X)) {
    free(dev->user_data);
    return false;
  }
  mgos_usleep(5000);

  // Mode | Pressure OS | Temp OS
  if (!mgos_i2c_write_reg_b(dev->i2c, dev->i2caddr, BME280_REG_CTRL_MEAS, BME280_MODE_NORMAL | BME280_OVERSAMP_16X << 2 | BME280_OVERSAMP_2X << 5)) {
    free(dev->user_data);
    return false;
  }


  dev->capabilities |= MGOS_BAROMETER_CAP_BAROMETER;
  dev->capabilities |= MGOS_BAROMETER_CAP_THERMOMETER;

  return true;
}

bool mgos_barometer_bme280_destroy(struct mgos_barometer *dev) {
  if (!dev) {
    return false;
  }
  if (dev->user_data) {
    free(dev->user_data);
    dev->user_data = NULL;
  }
  return true;
}

bool mgos_barometer_bme280_read(struct mgos_barometer *dev) {
  struct mgos_barometer_bme280_data *bme280_data;

  if (!dev) {
    return false;
  }
  bme280_data = (struct mgos_barometer_bme280_data *)dev->user_data;
  if (!bme280_data) {
    return false;
  }

  // read data from sensor
  uint8_t data[8];
  if (!mgos_i2c_read_reg_n(dev->i2c, dev->i2caddr, BME280_REG_PRESSURE_MSB, 8, data)) {
    return false;
  }
  uint32_t Padc, Tadc, Hadc;
  Padc = (uint32_t)((((uint32_t)(data[0])) << 12) | (((uint32_t)(data[1])) << 4) | ((uint32_t)data[2] >> 4));
  Tadc = (uint32_t)((((uint32_t)(data[3])) << 12) | (((uint32_t)(data[4])) << 4) | ((uint32_t)data[5] >> 4));
  Hadc = (data[6]) << 8 | data[7];

  // Convert data (from datasheet, section 8.1)
  double  var1, var2, var3, var4, var5, var6, T, P, H;
  double H_min = 1.0;
  double H_max = 100.0;
  int32_t t_fine;

  // Compensation for temperature -- double precision
  var1 = (((double)Tadc) / 16384.0 - ((double)bme280_data->calib.dig_T1) / 1024.0) * ((double)bme280_data->calib.dig_T2);
  var2 = ((((double)Tadc) / 131072.0 - ((double)bme280_data->calib.dig_T1) / 8192.0) *
          (((double)Tadc) / 131072.0 - ((double)bme280_data->calib.dig_T1) / 8192.0)) * ((double)bme280_data->calib.dig_T3);
  t_fine           = (int32_t)(var1 + var2);
  T                = (var1 + var2) / 5120.0;
  dev->temperature = (float)T;

  // Compensation for pressure -- double precision
  var1 = ((double)t_fine / 2.0) - 64000.0;
  var2 = var1 * var1 * ((double)bme280_data->calib.dig_P6) / 32768.0;
  var2 = var2 + var1 * ((double)bme280_data->calib.dig_P5) * 2.0;
  var2 = (var2 / 4.0) + (((double)bme280_data->calib.dig_P4) * 65536.0);
  var1 = (((double)bme280_data->calib.dig_P3) * var1 * var1 / 524288.0 + ((double)bme280_data->calib.dig_P2) * var1) / 524288.0;
  var1 = (1.0 + var1 / 32768.0) * ((double)bme280_data->calib.dig_P1);
  if (var1 == 0.0) {
    P = 0.0;
  } else {
    P    = 1048576.0 - (double)Padc;
    P    = (P - (var2 / 4096.0)) * 6250.0 / var1;
    var1 = ((double)bme280_data->calib.dig_P9) * P * P / 2147483648.0;
    var2 = P * ((double)bme280_data->calib.dig_P8) / 32768.0;
    P    = P + (var1 + var2 + ((double)bme280_data->calib.dig_P7)) / 16.0;
  }
  dev->pressure = (float)P;

  H = 0.0;
  if( dev->capabilities && MGOS_BAROMETER_CAP_HYGROMETER ) {
    // Compensation for humidity
    var1 = ((double)t_fine) - 76800.0;
    var2 = (((double)bme280_data->calib.dig_H4) * 64.0 + (((double) bme280_data->calib.dig_H5) / 16384.0) * var1);
    var3 = Hadc - var2;
    var4 = ((double)bme280_data->calib.dig_H2) / 65536.0;
    var5 = (1.0 + (((double) bme280_data->calib.dig_H3) / 67108864.0) * var1);
    var6 = 1.0 + (((double) bme280_data->calib.dig_H6) / 67108864.0) * var1 * var5;
    var6 = var3 * var4 * (var5 * var6);
    H = var6 * (1.0 - ((double) bme280_data->calib.dig_H1) * var6 / 524288.0);

    if (H > H_max)
        H = H_max;
    else if (H < H_min)
        H = H_min;
  }

  dev->humidity = (float)H;
  
  LOG(LL_DEBUG, ("P=%.2f T=%.2f H=%.2f", dev->pressure, dev->temperature, dev->humidity));

  return true;
}
