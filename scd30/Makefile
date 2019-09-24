# See user_config.inc for build customization
-include user_config.inc
include default_config.inc

.PHONY: all clean

all: scd30_example_usage

scd30_example_usage: clean
	$(CC) $(CFLAGS) -o $@ ${scd30_sources} ${${CONFIG_I2C_TYPE}_sources} ${scd30_dir}/scd30_example_usage.c

clean:
	$(RM) scd30_example_usage
