scd_driver_dir ?= ..
sensirion_common_dir ?= ${scd_driver_dir}/embedded-common
scd_common_dir ?= ${scd_driver_dir}/scd-common
scd30_dir ?= ${scd_driver_dir}/scd30
CONFIG_I2C_TYPE ?= hw_i2c

sw_i2c_impl_src ?= ${sensirion_common_dir}/sw_i2c/sensirion_sw_i2c_implementation.c
hw_i2c_impl_src ?= ${sensirion_common_dir}/hw_i2c/sensirion_hw_i2c_implementation.c

CFLAGS ?= -Os -Wall -fstrict-aliasing -Wstrict-aliasing=1 -Wsign-conversion -fPIC
CFLAGS += -I${sensirion_common_dir} -I${scd_common_dir} -I${scd30_dir} \
          -I${sensirion_common_dir}/${CONFIG_I2C_TYPE}

sensirion_common_sources = ${sensirion_common_dir}/sensirion_arch_config.h \
                           ${sensirion_common_dir}/sensirion_i2c.h \
                           ${sensirion_common_dir}/sensirion_common.h \
                           ${sensirion_common_dir}/sensirion_common.c

scd_common_sources = ${scd_common_dir}/scd_git_version.h \
                     ${scd_common_dir}/scd_git_version.c

scd30_sources = ${sensirion_common_sources} ${scd_common_sources} \
                ${scd30_dir}/scd30.h ${scd30_dir}/scd30.c

hw_i2c_sources = ${hw_i2c_impl_src}
sw_i2c_sources = ${sensirion_common_dir}/sw_i2c/sensirion_sw_i2c_gpio.h \
                 ${sensirion_common_dir}/sw_i2c/sensirion_sw_i2c.c \
                 ${sw_i2c_impl_src}
