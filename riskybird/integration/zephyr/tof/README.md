# RiskyBird Zephyr ToF contract

This component owns the board-independent interface between RiskyBird
workloads and the four VL53L5CX sensors behind the ADS7128 GPIO expander.

`rb_tof_array_init()` configures the four declared XSHUT channels, preserves
unrelated ADS7128 outputs, assigns unique 7-bit addresses cumulatively, and
checks the `0xF002` identity. Address-only builds use only Zephyr's public I2C
API. Ranging builds additionally adapt the pinned backend's VL53L5CX driver.

`rb_tof_frame` fixes the integration contract:

- four sensors;
- 4x4 or 8x8 row-major grids;
- millimeter signed distances;
- target count and status for every zone;
- final 7-bit sensor address;
- monotonically increasing host sequence;
- host monotonic completion timestamp in microseconds.

The timestamp is not yet the sensor exposure time. A later synchronization
revision must add an exposure-time estimate or interrupt-capture timestamp
before ToF frames are used for tightly timed control or estimator validation.

The Zephyr driver came from `origin/dev` at VL53L1X `19de8c845d0` and VL53L5CX
`84b2afb52d6`. The upstream ST ULD release and its license file were not
recorded at import — outstanding provenance work before any external release.
