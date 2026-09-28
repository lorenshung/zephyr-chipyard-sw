Four-VL53L5CX diagnostic
########################

The default ``address`` mode drives the four XSHUT signals through ADS7128
GPIO1--GPIO4, assigns addresses ``0x31``--``0x34``, and probes each sensor
identity. It uses only Zephyr's public I2C API.

.. code-block:: console

   ./rb build vl35l5cx_test --target esp32c6 --mode address
   ./rb run vl35l5cx_test --target esp32c6 --mode address \
     --port /dev/ttyACM0

The ``ranging`` mode uses the pinned backend's public VL53L5CX grid API and
streams machine-readable ``RB_TOF`` frames:

.. code-block:: console

   ./rb build vl35l5cx_test --target esp32c6 --mode ranging
   ./rb build vl35l5cx_test --target arty100t --mode ranging

Both modes call the RiskyBird-owned interface in
``integration/zephyr/tof``. The frame contract records sensor identity,
address, sequence, host completion timestamp, grid shape, target counts,
target status, and signed millimeter distance.

The current full-ranging image needs more than the KU040 configuration's
32 KiB addressable scratchpad. ``rb`` enforces a 192 KiB FPGA minimum for
this mode. This is a limitation of the current KU040 SoC configuration, not
of the custom board or its populated 2 GiB DDR4.

No attached-sensor capture has yet verified address assignment or ranging in
this repository. The current timestamp is also host completion time, not a
sensor exposure timestamp.
