# RiskyBird state estimation

This Zephyr application runs a RiskyBird-owned, RV64-safe Madgwick IMU
filter. Its default deterministic replay source makes toolchain and estimator
testing independent of attached sensors. The `bmi088` mode reads the FPGA I2C
controller, calibrates stationary gyro bias, and then runs at 200 Hz.

```console
rb build state --target arty100t
rb build state --target arty100t --mode bmi088
rb run state --target arty100t --mode bmi088
```

Output uses scaled integers so it does not require floating-point `printf`
support: quaternion values are multiplied by 1000 and Euler angles are in
millidegrees.
