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
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "scd30.h"

/* TO USE CONSOLE OUTPUT (printf) AND WAIT (sleep/usleep) PLEASE ADAPT THEM TO
 * YOUR PLATFORM, OR REMOVE THE DEFINES BELOW TO USE AS-IS.
 */
#include <stdio.h> /* printf */
#include <unistd.h> /* sleep, usleep */
#define printf(...)
#define sleep(...)
#define usleep(...)


int main(void)
{
    f32 co2_ppm, temperature, relative_humidity;
    u16 data_ready;
    s16 ret;
    s16 interval_in_seconds = 2;

    /* Busy loop for initialization, because the main loop does not work without
     * a sensor.
     */
    while (scd_probe() != STATUS_OK) {
        printf("SCD30 sensor probing failed\n");
        sleep(1);
    }
    printf("SCD30 sensor probing successful\n");

    scd_set_measurement_interval(interval_in_seconds);
    usleep(20000);
    scd_start_periodic_measurement(0);
    sleep(interval_in_seconds);

    while (1) {
        /* Measure co2, temperature and relative humidity and store into
         * variables.
         */
        ret = scd_get_data_ready(&data_ready);
        if (ret == STATUS_OK) {
            if (data_ready) {
                ret = scd_read_measurement(&co2_ppm, &temperature,
                                           &relative_humidity);
                if (ret != STATUS_OK) {
                    printf("error reading measurement\n");

                } else {
                    printf("measured co2 concentration: %0.2f ppm, "
                           "measured temperature: %0.2f degreeCelsius, "
                           "measured humidity: %0.2f %%RH\n",
                           co2_ppm, temperature, relative_humidity);
                }
            } else {
                printf("measurement not ready\n");
                usleep(20000);
                continue;
            }

        } else {
            printf("error reading data ready flag\n");
        }

        sleep(interval_in_seconds);
    }

    scd_stop_periodic_measurement();
    return 0;
}
