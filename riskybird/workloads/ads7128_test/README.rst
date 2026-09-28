ADS7128 diagnostic
##################

This workload probes the ADS7128 at address ``0x17``, reads its configuration
registers, and exercises a GPIO output. Bus wiring is supplied by the selected
Zephyr target rather than this application.

Build for any registered target:

.. code-block:: console

   ./rb build ads7128_test --target esp32c6
   ./rb build ads7128_test --target arty100t
   ./rb build ads7128_test --target ku040

Run on the connected ESP drone:

.. code-block:: console

   ./rb run ads7128_test --target esp32c6 --port /dev/ttyACM0

A successful software build does not prove that the ADS7128 is present.
Capture the serial result to establish physical evidence.
