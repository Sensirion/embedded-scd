/*
 * Copyright (c) 2018, Sensirion AG
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * * Neither the name of Sensirion AG nor the names of its
 *   contributors may be used to endorse or promote products derived from
 *   this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include "scd30.h"
#include "scd_git_version.h"
#include "sensirion_arch_config.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"

#ifdef SCD30_ADDRESS
static const uint8_t SCD30_I2C_ADDRESS = SCD30_ADDRESS;
#else
static const uint8_t SCD30_I2C_ADDRESS = 0x61;
#endif

#define SCD30_CMD_START_PERIODIC_MEASUREMENT 0x0010
#define SCD30_CMD_STOP_PERIODIC_MEASUREMENT 0x0104
#define SCD30_CMD_READ_MEASUREMENT 0x0300
#define SCD30_CMD_SET_MEASUREMENT_INTERVAL 0x4600
#define SCD30_CMD_GET_DATA_READY 0x0202
#define SCD30_CMD_SET_TEMPERATURE_OFFSET 0x5403
#define SCD30_CMD_SET_ALTITUDE 0x5102
#define SCD30_CMD_SET_FORCED_RECALIBRATION 0x5204
#define SCD30_CMD_AUTO_SELF_CALIBRATION 0x5306
#define SCD30_CMD_READ_SERIAL 0xD033
#define SCD30_SERIAL_NUM_WORDS 16
#define SCD30_WRITE_DELAY_US 20000

#define SCD30_MAX_BUFFER_WORDS 24
#define SCD30_CMD_SINGLE_WORD_BUF_LEN                                          \
    (SENSIRION_COMMAND_SIZE + SENSIRION_WORD_SIZE + CRC8_LEN)

int16_t scd30_start_periodic_measurement(uint16_t ambient_pressure_mbar) {
    if (ambient_pressure_mbar &&
        (ambient_pressure_mbar < 700 || ambient_pressure_mbar > 1400)) {
        /* out of allowable range */
        return STATUS_FAIL;
    }

    return sensirion_i2c_write_cmd_with_args(
        SCD30_I2C_ADDRESS, SCD30_CMD_START_PERIODIC_MEASUREMENT,
        &ambient_pressure_mbar, SENSIRION_NUM_WORDS(ambient_pressure_mbar));
}

int16_t scd30_stop_periodic_measurement() {
    return sensirion_i2c_write_cmd(SCD30_I2C_ADDRESS,
                                   SCD30_CMD_STOP_PERIODIC_MEASUREMENT);
}

int16_t scd30_read_measurement(float *co2_ppm, float *temperature,
                               float *humidity) {
    int16_t ret;
    uint8_t data[3][4];

    ret =
        sensirion_i2c_write_cmd(SCD30_I2C_ADDRESS, SCD30_CMD_READ_MEASUREMENT);
    if (ret != STATUS_OK)
        return ret;

    ret = sensirion_i2c_read_words_as_bytes(SCD30_I2C_ADDRESS, &data[0][0],
                                            SENSIRION_NUM_WORDS(data));
    if (ret != STATUS_OK)
        return ret;

    *co2_ppm = sensirion_bytes_to_float(data[0]);
    *temperature = sensirion_bytes_to_float(data[1]);
    *humidity = sensirion_bytes_to_float(data[2]);

    return STATUS_OK;
}

int16_t scd30_set_measurement_interval(uint16_t interval_sec) {
    int16_t ret;

    if (interval_sec < 2 || interval_sec > 1800) {
        /* out of allowable range */
        return STATUS_FAIL;
    }

    ret = sensirion_i2c_write_cmd_with_args(
        SCD30_I2C_ADDRESS, SCD30_CMD_SET_MEASUREMENT_INTERVAL, &interval_sec,
        SENSIRION_NUM_WORDS(interval_sec));
    sensirion_sleep_usec(SCD30_WRITE_DELAY_US);

    return ret;
}

int16_t scd30_get_data_ready(uint16_t *data_ready) {
    return sensirion_i2c_read_cmd(SCD30_I2C_ADDRESS, SCD30_CMD_GET_DATA_READY,
                                  data_ready, SENSIRION_NUM_WORDS(*data_ready));
}

int16_t scd30_set_temperature_offset(uint16_t temperature_offset) {
    int16_t ret;

    ret = sensirion_i2c_write_cmd_with_args(
        SCD30_I2C_ADDRESS, SCD30_CMD_SET_TEMPERATURE_OFFSET,
        &temperature_offset, SENSIRION_NUM_WORDS(temperature_offset));
    sensirion_sleep_usec(SCD30_WRITE_DELAY_US);

    return ret;
}

int16_t scd30_set_altitude(uint16_t altitude) {
    int16_t ret;

    ret = sensirion_i2c_write_cmd_with_args(SCD30_I2C_ADDRESS,
                                            SCD30_CMD_SET_ALTITUDE, &altitude,
                                            SENSIRION_NUM_WORDS(altitude));
    sensirion_sleep_usec(SCD30_WRITE_DELAY_US);

    return ret;
}

int16_t scd30_get_automatic_self_calibration(uint8_t *asc_enabled) {
    uint16_t word;
    int16_t ret;

    ret = sensirion_i2c_read_cmd(SCD30_I2C_ADDRESS,
                                 SCD30_CMD_AUTO_SELF_CALIBRATION, &word,
                                 SENSIRION_NUM_WORDS(word));
    if (ret != STATUS_OK)
        return ret;

    *asc_enabled = (uint8_t)word;

    return STATUS_OK;
}

int16_t scd30_enable_automatic_self_calibration(uint8_t enable_asc) {
    int16_t ret;
    uint16_t asc = !!enable_asc;

    ret = sensirion_i2c_write_cmd_with_args(SCD30_I2C_ADDRESS,
                                            SCD30_CMD_AUTO_SELF_CALIBRATION,
                                            &asc, SENSIRION_NUM_WORDS(asc));
    sensirion_sleep_usec(SCD30_WRITE_DELAY_US);

    return ret;
}

int16_t scd30_set_forced_recalibration(uint16_t co2_ppm) {
    int16_t ret;

    ret = sensirion_i2c_write_cmd_with_args(
        SCD30_I2C_ADDRESS, SCD30_CMD_SET_FORCED_RECALIBRATION, &co2_ppm,
        SENSIRION_NUM_WORDS(co2_ppm));
    sensirion_sleep_usec(SCD30_WRITE_DELAY_US);

    return ret;
}

int16_t scd30_read_serial(char *serial) {
    int16_t ret;

    ret = sensirion_i2c_write_cmd(SCD30_I2C_ADDRESS, SCD30_CMD_READ_SERIAL);
    if (ret)
        return ret;

    sensirion_sleep_usec(SCD30_WRITE_DELAY_US);
    ret = sensirion_i2c_read_words_as_bytes(
        SCD30_I2C_ADDRESS, (uint8_t *)serial, SCD30_SERIAL_NUM_WORDS);
    serial[2 * SCD30_SERIAL_NUM_WORDS] = '\0';
    return ret;
}

const char *scd30_get_driver_version() {
    return SCD_DRV_VERSION_STR;
}

uint8_t scd30_get_configured_address() {
    return SCD30_I2C_ADDRESS;
}

int16_t scd30_probe() {
    uint16_t data_ready;

    /* try to read data-ready state */
    return scd30_get_data_ready(&data_ready);
}
