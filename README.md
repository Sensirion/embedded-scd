# embedded-scd [![GitHub license](https://img.shields.io/badge/license-BSD3-blue.svg)](https://raw.githubusercontent.com/Sensirion/embedded-scd/master/LICENSE)

Note: This repository is deprecated. Please use our new drivers:
* SCD30: https://github.com/Sensirion/embedded-i2c-scd30
* SCD4x: https://github.com/Sensirion/embedded-i2c-scd4x

This repository contains the embedded driver sources for Sensirion's SCD product
line.

## Clone this repository
```
 git clone --recursive https://github.com/Sensirion/embedded-scd.git
```

## Repository content
* embedded-common (submodule repository for the common embedded driver HAL)
* scd30 (SCD30 driver related)

## Collecting resources
```
make release
```
This will create the `release` folder with the necessary driver files in it,
including a Makefile. That way, you have just one folder with all the sources
ready to build your driver for your platform.

## Files to adjust (from embedded-common)
You only need to touch the following files:

* `sensirion_arch_config.h` (architecture specifics, you need to specify
the integer sizes)

and depending on your i2c implementation either of the following:

* `embedded-common/hw_i2c/sensirion_hw_i2c_implementation.c`
  functions for hardware i2c communication if your platform supports that
* `embedded-common/sw_i2c/sensirion_sw_i2c_implementation.c`
  functions for software i2c communication via GPIOs

## Building the driver
1. Adjust sensirion\_arch\_config.h if you don't have the `<stdint.h>` header
   file available
2. Implement necessary functions in one of the `*_implementation.c` files
   described above
3. make

---

Please check the [embedded-common](https://github.com/Sensirion/embedded-common)
repository for further information and sample implementations.

---
