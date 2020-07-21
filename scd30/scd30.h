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

#ifndef SCD30_H
#define SCD30_H
#include "sensirion_arch_config.h"
#include "sensirion_common.h"
#include "sensirion_i2c.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * scd30_probe() - check if the SCD sensor is available and initialize it
 *
 * @return  0 on success, an error code otherwise.
 */
int16_t scd30_probe(void);

/**
 * scd30_get_driver_version() - Returns the driver version
 *
 * @return  Driver version string
 */
const char* scd30_get_driver_version(void);

/**
 * scd30_get_configured_address() - Returns the configured I2C address
 *
 * @return      uint8_t I2C address
 */
uint8_t scd30_get_configured_address(void);

/**
 * scd30_start_periodic_measurement() - Start continuous measurement to measure
 * CO2 concentration, relative humidity and temperature or updates the ambient
 * pressure if the periodic measurement is already running.
 *
 * Measurement data which is not read from the sensor is continuously
 * overwritten. The CO2 measurement value can be compensated for ambient
 * pressure by setting the pressure value in mBar. Setting the ambient pressure
 * overwrites previous and future settings of altitude compensation. Setting the
 * pressure to zero deactivates the ambient pressure compensation.
 * The continuous measurement status is saved in non-volatile memory. The last
 * measurement mode is resumed after repowering.
 *
 * @param ambient_pressure_mbar Ambient pressure in millibars. 0 to deactivate
 *                              ambient pressure compensation (reverts to
 *                              altitude compensation, if set), 700-1200mBar
 *                              allowable range otherwise
 *
 * @return                      0 if the command was successful, an error code
 *                              otherwise
 */
int16_t scd30_start_periodic_measurement(uint16_t ambient_pressure_mbar);

/**
 * scd30_stop_periodic_measurement() - Stop the continuous measurement
 *
 * @return  0 if the command was successful, else an error code
 */
int16_t scd30_stop_periodic_measurement(void);

/**
 * scd30_read_measurement() - Read out an available measurement when new
 * measurement data is available.
 * Make sure that the measurement is completed by reading the data ready status
 * bit with scd30_get_data_ready().
 *
 * @param co2_ppm       CO2 concentration in ppm
 * @param temperature   the address for the result of the temperature
 *                      measurement
 * @param humidity      the address for the result of the relative humidity
 *                      measurement
 *
 * @return              0 if the command was successful, an error code otherwise
 */
int16_t scd30_read_measurement(float* co2_ppm, float* temperature,
                               float* humidity);

/**
 * scd30_set_measurement_interval() - Sets the measurement interval in
 * continuous measurement mode.
 *
 * The initial value on powerup is 2s. The chosen measurement interval is saved
 * in non-volatile memory and thus is not reset to its initial value after power
 * up.
 *
 * @param interval_sec  The measurement interval in seconds. The allowable range
 *                      is 2-1800s
 *
 * @return              0 if the command was successful, an error code otherwise
 */
int16_t scd30_set_measurement_interval(uint16_t interval_sec);

/**
 * scd30_get_data_ready() - Get data ready status
 *
 * Data ready command is used to determine if a measurement can be read from the
 * sensor's buffer. Whenever there is a measurement available from the internal
 * buffer this command returns 1 and 0 otherwise. As soon as the measurement has
 * been read by the return value changes to 0. It is recommended to use the data
 * ready status byte before readout of the measurement values with
 * scd30_read_measurement().
 *
 * @param data_ready    Pointer to memory of where to set the data ready bit.
 *                      The memory is set to 1 if a measurement is ready to be
 *                      fetched, 0 otherwise.
 *
 * @return              0 if the command was successful, an error code otherwise
 */
int16_t scd30_get_data_ready(uint16_t* data_ready);

/**
 * scd30_set_temperature_offset() - Set the temperature offset
 *
 * The on-board RH/T sensor is influenced by thermal self-heating of SCD30 and
 * other electrical components. Design-in alters the thermal properties of SCD30
 * such that temperature and humidity offsets may occur when operating the
 * sensor in end-customer devices. Compensation of those effects is achievable
 * by writing the temperature offset found in continuous operation of the device
 * into the sensor.
 * The temperature offset value is saved in non-volatile memory. The last set
 * value will be used after repowering.
 *
 * @param temperature_offset    Temperature offset, unit [degrees Celsius * 100]
 *                              i.e. one tick corresponds to 0.01 degrees C
 *
 * @return                      0 if the command was successful, an error code
 *                              otherwise
 */
int16_t scd30_set_temperature_offset(uint16_t temperature_offset);

/**
 * scd30_set_altitude() - Set the altitude above sea level
 *
 * Measurements of CO2 concentration are influenced by altitude. When a value is
 * set, the altitude-effect is compensated. The altitude setting is disregarded
 * when an ambient pressure is set on the sensor with
 * scd30_start_periodic_measurement.
 * The altitude is saved in non-volatile memory. The last set value will be used
 * after repowering.
 *
 * @param altitude  altitude in meters above sea level, 0 meters is the default
 *                  value and disables altitude compensation
 *
 * @return          0 if the command was successful, an error code otherwise
 */
int16_t scd30_set_altitude(uint16_t altitude);

/**
 * scd30_get_automatic_self_calibration() - Read if the sensor's automatic self
 * calibration is enabled or disabled
 *
 * See scd30_enable_automatic_self_calibration() for more details.
 *
 * @param asc_enabled   Pointer to memory of where to set the self calibration
 *                      state. 1 if ASC is enabled, 0 if ASC disabled. Remains
 *                      untouched if return is non-zero.
 *
 * @return              0 if the command was successful, an error code otherwise
 */
int16_t scd30_get_automatic_self_calibration(uint8_t* asc_enabled);

/**
 * scd30_enable_automatic_self_calibration() - Enable or disable the sensor's
 * automatic self calibration
 *
 * When activated for the first time a period of minimum 7 days is needed so
 * that the algorithm can find its initial parameter set for ASC.
 * The sensor has to be exposed to fresh air for at least 1 hour every day.
 * Refer to the datasheet for further conditions
 *
 * ASC status is saved in non-volatile memory. When the sensor is powered down
 * while ASC is activated SCD30 will continue with automatic self-calibration
 * after repowering without sending the command.
 *
 * @param enable_asc    enable ASC if non-zero, disable otherwise
 *
 * @return              0 if the command was successful, an error code otherwise
 */
int16_t scd30_enable_automatic_self_calibration(uint8_t enable_asc);

/**
 * scd30_set_forced_recalibration() - Forcibly recalibrate the sensor to a known
 * value.
 *
 * Forced recalibration (FRC) is used to compensate for sensor drifts when a
 * reference value of the CO2 concentration in close proximity to the SCD30 is
 * available.
 *
 * For best results the sensor has to be run in a stable environment in
 * continuous mode at a measurement rate of 2s for at least two minutes before
 * applying the calibration command and sending the reference value.
 * Setting a reference CO2 concentration will overwrite the settings from ASC
 * (see scd30_enable_automatic_self_calibration) and vice-versa. The reference
 * CO2 concentration has to be in the range 400..2000 ppm.
 *
 * FRC value is saved in non-volatile memory, the last set FRC value will be
 * used for field-calibration after repowering.
 *
 * @param co2_ppm   recalibrate to this specific co2 concentration
 *
 * @return          0 if the command was successful, an error code otherwise
 */
int16_t scd30_set_forced_recalibration(uint16_t co2_ppm);

/**
 * Read out the serial number
 *
 * @param serial    the address for the result of the serial number.
 *                  --------------------------------------
 *                  THE BUFFER MUST HOLD AT LEAST 33 BYTES
 *                  --------------------------------------
 *                  Usage example:
 *                  ```
 *                  char scd30_serial[33];
 *                  if (scd30_read_serial(scd30_serial) == 0) {
 *                      printf("SCD30 serial: %s\n", scd30_serial);
 *                  } else {
 *                      printf("Error reading SCD30 serial\n");
 *                  }
 *                  ```
 *                  Contains a zero-terminated string.
 * @return          0 if the command was successful, else an error code.
 */
int16_t scd30_read_serial(char* serial);

#ifdef __cplusplus
}
#endif

#endif /* SCD30_H */
