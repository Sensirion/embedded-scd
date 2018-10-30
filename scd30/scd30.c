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

#include "sensirion_arch_config.h"
#include "sensirion_i2c.h"
#include "sensirion_common.h"
#include "scd_git_version.h"
#include "scd30.h"


#ifdef SCD_ADDRESS
static const u8 SCD_I2C_ADDRESS = SCD_ADDRESS;
#else
static const u8 SCD_I2C_ADDRESS = 0x61;
#endif

#define SCD_CMD_START_PERIODIC_MEASUREMENT  0x0010
#define SCD_CMD_STOP_PERIODIC_MEASUREMENT   0x0104
#define SCD_CMD_READ_MEASUREMENT            0x0300
#define SCD_CMD_SET_MEASUREMENT_INTERVAL    0x4600
#define SCD_CMD_GET_DATA_READY              0x0202
#define SCD_CMD_SET_TEMPERATURE_OFFSET      0x5403
#define SCD_CMD_SET_ALTITUDE                0x5102
#define SCD_CMD_SET_FORCED_RECALIBRATION    0x5204
#define SCD_CMD_AUTO_SELF_CALIBRATION       0x5306

#define SCD_MAX_BUFFER_WORDS 24
#define SCD_CMD_SINGLE_WORD_BUF_LEN (SENSIRION_COMMAND_SIZE + \
                                     SENSIRION_WORD_SIZE + CRC8_LEN)


s16 scd_start_periodic_measurement(u16 ambient_pressure_mbar) {
    if (ambient_pressure_mbar && (ambient_pressure_mbar < 700 ||
                                  ambient_pressure_mbar > 1400)) {
        /* out of allowable range */
        return STATUS_FAIL;
    }

    return sensirion_i2c_write_cmd_with_args(SCD_I2C_ADDRESS,
                                             SCD_CMD_START_PERIODIC_MEASUREMENT,
                                             &ambient_pressure_mbar,
                                             SENSIRION_NUM_WORDS(ambient_pressure_mbar));
}


s16 scd_stop_periodic_measurement() {
    return sensirion_i2c_write_cmd(SCD_I2C_ADDRESS,
                                   SCD_CMD_STOP_PERIODIC_MEASUREMENT);
}


s16 scd_read_measurement(f32 *co2_ppm, f32 *temperature, f32 *humidity) {
    s16 ret;
    u32 word_buf[3]; /* 32bit or 2 words for each co2, temperature, humidity */
    union {
        u32 raw;
        f32 float32;
    } tmp;

    ret = sensirion_i2c_read_cmd(SCD_I2C_ADDRESS, SCD_CMD_READ_MEASUREMENT,
                                 (u16 *)word_buf,
                                 SENSIRION_NUM_WORDS(word_buf));
    if (ret != STATUS_OK)
        return ret;

    tmp.raw = be32_to_cpu(word_buf[0]);
    *co2_ppm = tmp.float32;

    tmp.raw = be32_to_cpu(word_buf[1]);
    *temperature = tmp.float32;

    tmp.raw = be32_to_cpu(word_buf[2]);
    *humidity = tmp.float32;

    return STATUS_OK;
}


s16 scd_set_measurement_interval(u16 interval_sec) {
    if (interval_sec < 2 || interval_sec > 1800) {
        /* out of allowable range */
        return STATUS_FAIL;
    }

    return sensirion_i2c_write_cmd_with_args(SCD_I2C_ADDRESS,
                                             SCD_CMD_SET_MEASUREMENT_INTERVAL,
                                             &interval_sec,
                                             SENSIRION_NUM_WORDS(interval_sec));
}


s16 scd_get_data_ready(u16 *data_ready) {
    return sensirion_i2c_read_cmd(SCD_I2C_ADDRESS, SCD_CMD_GET_DATA_READY,
                                  data_ready,
                                  SENSIRION_NUM_WORDS(*data_ready));
}


s16 scd_set_temperature_offset(u16 temperature_offset) {
    return sensirion_i2c_write_cmd_with_args(SCD_I2C_ADDRESS,
                                             SCD_CMD_SET_TEMPERATURE_OFFSET,
                                             &temperature_offset,
                                             SENSIRION_NUM_WORDS(temperature_offset));
}


s16 scd_set_altitude(u16 altitude) {
    return sensirion_i2c_write_cmd_with_args(SCD_I2C_ADDRESS,
                                             SCD_CMD_SET_ALTITUDE, &altitude,
                                             SENSIRION_NUM_WORDS(altitude));
}


s16 scd_get_automatic_self_calibration(u8 *asc_enabled) {
    u16 word;
    s16 ret;

    ret = sensirion_i2c_read_cmd(SCD_I2C_ADDRESS, SCD_CMD_AUTO_SELF_CALIBRATION,
                                 &word, SENSIRION_NUM_WORDS(word));
    if (ret == 0)
        *asc_enabled = (u8)word;

    return ret;
}


s16 scd_enable_automatic_self_calibration(u8 enable_asc) {
    u16 asc = !!enable_asc;

    return sensirion_i2c_write_cmd_with_args(SCD_I2C_ADDRESS,
                                             SCD_CMD_AUTO_SELF_CALIBRATION,
                                             &asc, SENSIRION_NUM_WORDS(asc));
}


s16 scd_set_forced_recalibration(u16 co2_ppm) {
    return sensirion_i2c_write_cmd_with_args(SCD_I2C_ADDRESS,
                                             SCD_CMD_SET_FORCED_RECALIBRATION,
                                             &co2_ppm,
                                             SENSIRION_NUM_WORDS(co2_ppm));
}


const char *scd_get_driver_version() {
    return SCD_DRV_VERSION_STR;
}


u8 scd_get_configured_address() {
    return SCD_I2C_ADDRESS;
}


s16 scd_probe() {
    u16 data_ready;

    /* Initialize I2C */
    sensirion_i2c_init();

    /* try to read data-ready state */
    return scd_get_data_ready(&data_ready);
}
