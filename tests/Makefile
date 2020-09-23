include ../embedded-common/test-config/base_config.inc
scd_driver_dir := ${driver_dir}/embedded-scd
include ${scd_driver_dir}/scd30/default_config.inc

scd30_test_binaries := scd30-test-sw_i2c

.PHONY: all clean prepare test

all: clean prepare test

prepare:
	cd ${scd_driver_dir} && $(MAKE) prepare

scd30-test-sw_i2c: CONFIG_I2C_TYPE := sw_i2c
scd30-test-sw_i2c: scd-test.cpp ${scd30_sources} ${sw_i2c_sources} ${sensirion_test_sources}
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

clean:
	$(RM) ${scd30_test_binaries}

test: prepare ${scd30_test_binaries}
	set -ex; for test in ${scd30_test_binaries}; do echo $${test}; ./$${test}; echo; done;
