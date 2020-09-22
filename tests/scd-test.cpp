#include "scd30.h"
#include "sensirion_test_setup.h"

#define SCD30_TEST_MIN_PRESSURE 700
#define SCD30_TEST_MAX_PRESSURE 1400
#define SCD30_TEST_MIN_MEASUREMENT_INTERVAL 2
#define SCD30_TEST_MAX_MEASUREMENT_INTERVAL 1800
#define SCD30_MIN_CO2 0
#define SCD30_MAX_CO2 40000
#define SCD30_MIN_TEMPERATURE 10
#define SCD30_MAX_TEMPERATURE 35
#define SCD30_MIN_HUMIDITY 20
#define SCD30_MAX_HUMIDITY 100

static void scd30_tests(uint16_t interval_s, uint16_t pressure_mbar) {
    int16_t ret;
    uint16_t data_ready;
    float co2_ppm;
    float temperature;
    float humidity;

    printf("\t\t\t\tTesting with pressure = %umbar\n", pressure_mbar);

    ret = scd30_start_periodic_measurement(pressure_mbar);
    if (pressure_mbar == 0 || (pressure_mbar >= SCD30_TEST_MIN_PRESSURE &&
                               pressure_mbar <= SCD30_TEST_MAX_PRESSURE)) {
        CHECK_ZERO_TEXT(ret,
                        "scd30_start_periodic_measurement should not fail");
    } else {
        CHECK_TRUE_TEXT(ret, "scd30_start_periodic_measurement should fail");
        return;
    }

    // Check if there is old data available
    ret = scd30_get_data_ready(&data_ready);
    CHECK_ZERO_TEXT(ret, "scd30_get_data_ready for old data");

    if (data_ready) {
        // Get old data and discard it
        ret = scd30_read_measurement(&co2_ppm, &temperature, &humidity);
        CHECK_ZERO_TEXT(ret, "scd30_read_measurement for old data");
    }

    // Sleep for interval_s seconds minus 100ms
    sensirion_sleep_usec((1e6 * interval_s) - 1e5);

    // Wait for data to be ready
    do {
        sensirion_sleep_usec(1e5);  // Sleep 100ms
        ret = scd30_get_data_ready(&data_ready);
        CHECK_ZERO_TEXT(ret, "scd30_get_data_ready while polling");
    } while (!data_ready);

    ret = scd30_read_measurement(&co2_ppm, &temperature, &humidity);
    CHECK_ZERO_TEXT(ret, "scd30_read_measurement");
    printf("\t\t\t\t\tCO2 = %fppm\n", co2_ppm);
    printf("\t\t\t\t\tTmp = %f degree C\n", temperature);
    printf("\t\t\t\t\t%%RH = %f%%\n", humidity);
    CHECK_TRUE_TEXT(co2_ppm >= SCD30_MIN_CO2 && co2_ppm <= SCD30_MAX_CO2,
                    "scd30_measurement CO2");
    CHECK_TRUE_TEXT(temperature >= SCD30_MIN_TEMPERATURE &&
                        temperature <= SCD30_MAX_TEMPERATURE,
                    "scd30_measurement temperature");
    CHECK_TRUE_TEXT(humidity >= SCD30_MIN_HUMIDITY &&
                        humidity <= SCD30_MAX_HUMIDITY,
                    "scd30_measurement humidity");

    ret = scd30_stop_periodic_measurement();
    CHECK_ZERO_TEXT(ret, "scd30_stop_periodic_measurement");
}

static void scd30_test_all_pressures(uint16_t interval_s) {
    scd30_tests(interval_s, 0);
    scd30_tests(interval_s, SCD30_TEST_MIN_PRESSURE - 1);
    scd30_tests(interval_s, SCD30_TEST_MIN_PRESSURE);
    scd30_tests(interval_s, SCD30_TEST_MAX_PRESSURE);
    scd30_tests(interval_s, SCD30_TEST_MAX_PRESSURE + 1);
}

static void scd30_test_set_measurement_interval(uint16_t interval_s) {
    printf("\t\t\tTesting measurement interval = %us\n", interval_s);
    int16_t ret = scd30_set_measurement_interval(interval_s);
    if (interval_s >= SCD30_TEST_MIN_MEASUREMENT_INTERVAL &&
        interval_s <= SCD30_TEST_MAX_MEASUREMENT_INTERVAL) {
        CHECK_ZERO_TEXT(ret, "scd30_set_measurement_interval should work");
        scd30_test_all_pressures(interval_s);
    } else {
        CHECK_TRUE_TEXT(ret, "scd30_set_measurement_interval should fail");
    }
}

static void scd30_test_all_measurement_intervals() {
    scd30_test_set_measurement_interval(SCD30_TEST_MIN_MEASUREMENT_INTERVAL -
                                        1);
    scd30_test_set_measurement_interval(SCD30_TEST_MIN_MEASUREMENT_INTERVAL);
    scd30_test_set_measurement_interval(2 *
                                        SCD30_TEST_MIN_MEASUREMENT_INTERVAL);
    scd30_test_set_measurement_interval(SCD30_TEST_MAX_MEASUREMENT_INTERVAL +
                                        1);
}

static void scd30_test_temperature_offset(uint16_t temp_offset) {
    printf("\t\tTesting temperature offset = %u degree C\n", temp_offset);
    int16_t ret = scd30_set_temperature_offset(temp_offset * 100);
    CHECK_ZERO_TEXT(ret, "scd30_set_temperature_offset");
    scd30_test_all_measurement_intervals();
}

static void scd30_test_all_temperature_offsets() {
    scd30_test_temperature_offset(0);   // No offset
    scd30_test_temperature_offset(50);  // 50 degrees C
}

static void scd30_test_altitude(uint16_t altitude) {
    int16_t ret;
    printf("\tTesting altitude = %um\n", altitude);
    ret = scd30_set_altitude(altitude);
    CHECK_ZERO_TEXT(ret, "scd30_set_altitude");
    scd30_test_all_temperature_offsets();
}

static void scd30_test_all_altitudes() {
    scd30_test_altitude(0);    // Altitude compensation disabled
    scd30_test_altitude(500);  // With altitude compensation on 500m
}

static void scd30_test_automatic_calibration(uint8_t set_asc) {
    uint8_t get_asc;
    int16_t ret;

    ret = scd30_enable_automatic_self_calibration(set_asc);
    CHECK_ZERO_TEXT(ret, "scd30_enable_automatic_self_calibration");

    ret = scd30_get_automatic_self_calibration(&get_asc);
    CHECK_ZERO_TEXT(ret, "scd30_get_automatic_self_calibration");

    CHECK_EQUAL_TEXT(set_asc, get_asc,
                     "automatic_self_calibration variables should match");
    scd30_test_all_altitudes();
}

static int16_t scd30_reset() {
    const uint16_t arg = 1;
    int16_t ret = sensirion_i2c_write_cmd_with_args(0x61, 0xD304, &arg,
                                                    SENSIRION_NUM_WORDS(arg));
    sensirion_sleep_usec(1e6);
    return ret;
}

static void scd30_test_setup() {
    int16_t ret;
    char serial[33];

    ret = scd30_reset();
    CHECK_ZERO_TEXT(ret, "scd30_reset");

    const char* version = scd30_get_driver_version();
    printf("scd30_get_driver_version: %s\n", version);

    uint8_t addr = scd30_get_configured_address();
    CHECK_EQUAL_TEXT(0x61, addr, "scd30_get_configured_address");

    ret = scd30_read_serial(serial);
    CHECK_ZERO_TEXT(ret, "scd30_get_serial_id");
    printf("SCD30 serial: %s\n", serial);
}

TEST_GROUP (SCDTestGroup) {
    void setup() {
        int16_t ret;
        sensirion_i2c_init();
        ret = sensirion_i2c_mux_set_single_channel(0x72, 3);
        CHECK_EQUAL_TEXT(0, ret, "sensirion_i2c_mux_select_TEXT(0x72, 3)");
        ret = scd30_probe();
        CHECK_ZERO_TEXT(ret, "scd30_probe in test setup");
        scd30_test_setup();
    }

    void teardown() {
        int16_t ret = scd30_reset();
        CHECK_EQUAL_TEXT(0, ret, "scd30_reset");
        sensirion_i2c_release();
    }
};

TEST (SCDTestGroup, SCD30Test_no_automatic) {
    printf("Starting SCD30 Test without automatic calibration\n");
    scd30_test_automatic_calibration(0);
}

TEST (SCDTestGroup, SCD30Test_with_automatic) {
    printf("Starting SCD30 Test with automatic calibration\n");
    scd30_test_automatic_calibration(1);
}

TEST (SCDTestGroup, SCD30Test_with_forced) {
    printf("Starting SCD30 Test with forced calibration\n");
    int16_t ret = scd30_set_forced_recalibration(400);
    CHECK_ZERO_TEXT(ret, "scd30_set_forced_recalibration");
    scd30_test_all_altitudes();
}
