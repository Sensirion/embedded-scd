sensirion_common_dir := ../embedded-common
scd_common_dir := .
sw_i2c_dir := ${sensirion_common_dir}/sw_i2c
hw_i2c_dir := ${sensirion_common_dir}/hw_i2c

CFLAGS := -Os -Wall -Werror -I. -I${sensirion_common_dir}

sensirion_common_objects := sensirion_common.o
scd_common_objects := scd_git_version.o
scd30_binaries := scd30_example_usage_sw_i2c scd30_example_usage_hw_i2c
scd_binaries += ${scd30_binaries}

sw_objects := sensirion_sw_i2c.o sensirion_sw_i2c_implementation.o
hw_objects := sensirion_hw_i2c_implementation.o
all_objects := ${sensirion_common_objects} ${scd_common_objects} ${hw_objects} ${sw_objects} scd30.o

.PHONY: all

all: ${scd_binaries}

scd_git_version.o: ${scd_common_dir}/scd_git_version.c
	$(CC) $(CFLAGS) -c -o $@ $^
sensirion_common.o: ${sensirion_common_dir}/sensirion_common.c
	$(CC) $(CFLAGS) -c -o $@ $^
sensirion_sw_i2c_implementation.o: ${sw_i2c_dir}/sample-implementations/linux_user_space/sensirion_sw_i2c_implementation.c
	$(CC) -I${sw_i2c_dir} $(CFLAGS) -c -o $@ $^
sensirion_hw_i2c_implementation.o: ${hw_i2c_dir}/sensirion_hw_i2c_implementation.c
	$(CC) $(CFLAGS) -c -o $@ $^

sensirion_sw_i2c.o: ${sw_i2c_dir}/sensirion_sw_i2c.c
	$(CC) -I${sw_i2c_dir} $(CFLAGS) -c -o $@ $^

scd30.o: ${sensirion_common_dir}/sensirion_arch_config.h ${sensirion_common_dir}/sensirion_i2c.h ${scd_common_dir}/scd_git_version.c scd30.h scd30.c

scd30_example_usage_sw_i2c: ${sensirion_common_objects} ${scd_common_objects} ${sw_objects} scd30.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LOADLIBES) $(LDLIBS) scd30_example_usage.c

scd30_example_usage_hw_i2c: ${sensirion_common_objects} ${scd_common_objects} ${hw_objects} scd30.o
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LOADLIBES) $(LDLIBS) scd30_example_usage.c

clean:
	$(RM) ${all_objects} ${scd_binaries}
