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

#include <stdio.h> /* printf */

/* TO USE CONSOLE OUTPUT (printf) YOU MAY NEED TO ADAPT THE
 * INCLUDE ABOVE OR DEFINE IT ACCORDING TO YOUR PLATFORM.
 * #define printf(...)
 */

int main(void) {
    float co2_ppm, temperature, relative_humidity;
    int16_t err;
    uint16_t interval_in_seconds = 2;

    /* Initialize I2C */
    sensirion_i2c_init();

    /* Busy loop for initialization, because the main loop does not work without
     * a sensor.
     */
    while (scd30_probe() != STATUS_OK) {
        printf("SCD30 sensor probing failed\n");
        sensirion_sleep_usec(1000000u);
    }
    printf("SCD30 sensor probing successful\n");

    scd30_set_measurement_interval(interval_in_seconds);
    sensirion_sleep_usec(20000u);
    scd30_start_periodic_measurement(0);
    sensirion_sleep_usec(interval_in_seconds * 1000000u);

    while (1) {
        /* Measure co2, temperature and relative humidity and store into
         * variables.
         */
        err =
            scd30_read_measurement(&co2_ppm, &temperature, &relative_humidity);
        if (err != STATUS_OK) {
            printf("error reading measurement\n");

        } else {
            printf("measured co2 concentration: %0.2f ppm, "
                   "measured temperature: %0.2f degreeCelsius, "
                   "measured humidity: %0.2f %%RH\n",
                   co2_ppm, temperature, relative_humidity);
        }

        sensirion_sleep_usec(interval_in_seconds * 1000000u);
    }

    scd30_stop_periodic_measurement();
    return 0;
}
