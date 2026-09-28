I2C scanner
###########

This target-independent diagnostic scans I2C addresses ``0x03`` through
``0x77`` and reports acknowledgements.

.. code-block:: console

   ./rb build i2c_scanner --target esp32c6
   ./rb run i2c_scanner --target esp32c6 --port /dev/ttyACM0
