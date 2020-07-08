# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [2.1.0] - 2020-07-08

 * [`fixed`]    Fix typo in header include-guard.
 * [`changed`]  Use `float` instead of the custom `float32_t` type
 * [`changed`]  Use configuration independent endianness conversions: no need to
                define SENSIRION_BIG_ENDIAN anymore.

## [2.0.2] - 2019-11-01

 * [`fixed`]    SCD30: Fix buffer overflow when reading measurements that was
                introduced in 2.0.1.

## [2.0.1] - 2019-10-17

 * [`fixed`]    Copy correct `AUTHORS`, `CHANGELOG.md`, `LICENSE`, and
                `README.md` files to target locations when running the `release`
                target of the driver's root Makefile.
 * [`removed`]  Remove the `AUTHORS` file from the driver and the
                `embedded-common` submodule, as it adds more noise than benefit.
                The contributors can be found in the git log.
 * [`changed`]  Avoid converting twice between endianness in
                `scd30_read_measurement`. (improves performance and portablity)
 * [`changed`]  Cleanup scd30 example

## [2.0.0] - 2019-10-01

 * [`changed`]  Functions are now dedicated per sensor (e.g. `scd30_probe()`
                instead of `scd_probe()`)
 * [`changed`]  Move the i2c init call out of `probe()` and into the example
 * [`added`]    Add `scd30_read_serial()` to read out the serial number
 * [`added`]    embedded-common: Add multi-bus support along with an example.
 * [`fixed`]    Fix compilation warnings when compiling the linux user space
                sample implementation with `CONFIG_I2C_TYPE` set to `sw_i2c`
 * [`changed`]  Split out `default_config.inc` from Makefile to configure paths
                and CFLAGS for the SCD30 driver
 * [`changed`]  Only one example with either `hw_i2c` or `sw_i2c` is build,
                depending on `CONFIG_I2C_TYPE`. Defaults to `hw_i2c`.
 * [`fixed`]    Fix clock stretching timeout that might occur when
                `CONFIG_I2C_TYPE` is set to `sw_i2c`

## [1.0.0] - 2019-05-14

 * `[fixed]` Add processing delay on i2c writes
 * `[fixed]` Potential issues regarding strict aliasing
 * `[changed]` Use stdint types (`uint16_t` instead of `u16`)

## [0.9.0] - 2018-10-08

 * `[fixed]` Measurement readout
 * `[added]` Self calibration
   * `scd_get_automatic_self_calibration`
   * `scd_set_forced_recalibration`
   * `scd_enable_automatic_self_calibration`
 * `[fixed]` Parameters on big-endian systems

## [0.1.0] - 2018-07-20

 * Initial support for SCD30

[Unreleased]: https://github.com/Sensirion/embedded-scd/compare/2.1.0...master
[2.1.0]: https://github.com/Sensirion/embedded-scd/compare/2.0.2...2.1.0
[2.0.2]: https://github.com/Sensirion/embedded-scd/compare/2.0.1...2.0.2
[2.0.1]: https://github.com/Sensirion/embedded-scd/compare/2.0.0...2.0.1
[2.0.0]: https://github.com/Sensirion/embedded-scd/compare/1.0.0...2.0.0
[1.0.0]: https://github.com/Sensirion/embedded-scd/compare/0.9.0...1.0.0
[0.9.0]: https://github.com/Sensirion/embedded-scd/compare/0.1.0...0.9.0
[0.1.0]: https://github.com/Sensirion/embedded-scd/releases/tag/0.1.0
