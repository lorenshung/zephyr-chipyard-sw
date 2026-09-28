BMI088 diagnostic
#################

This workload streams accelerometer, gyroscope, and die-temperature samples
through Zephyr's BMI08x driver. Target overlays define the two sensor nodes and
the ``bmi088-accel`` and ``bmi088-gyro`` aliases.

.. code-block:: console

   ./rb build bmi088_test --target esp32c6
   ./rb run bmi088_test --target esp32c6 --port /dev/ttyACM0

The same source builds for ``arty100t`` and ``ku040``; only their devicetree
and memory contracts differ.
