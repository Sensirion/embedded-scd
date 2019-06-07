# Changelog

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

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

[Unreleased]: https://github.com/Sensirion/embedded-scd/compare/1.0.0...master
[1.0.0]: https://github.com/Sensirion/embedded-scd/compare/0.9.0...1.0.0
[0.9.0]: https://github.com/Sensirion/embedded-scd/compare/0.1.0...0.9.0
[0.1.0]: https://github.com/Sensirion/embedded-scd/releases/tag/0.1.0
