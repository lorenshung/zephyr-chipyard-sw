PMW3901 optical-flow diagnostic and flow-calibration capture
############################################################

Two jobs in one image:

**Diagnostic (IDLE).** One line a second with ``deltaX``, ``deltaY``, ``SQUAL``
and ``shutter``, so the sensor can be aimed and the surface judged before
anything is measured. Counts seen while idle are still summed and shown, so
nothing is silently discarded.

**Calibration capture (CAPTURE).** Sums every motion count between a start and a
stop and prints one greppable ``FLOWCAL RUN`` line carrying the sums, the
surface quality and the shutter. ``tools/flow-cal.py`` parses exactly that line
and solves ``FLOW_RAD_PER_COUNT``.

The full bench procedure -- surface, lighting, geometry, run count, rejection
criteria and the validation batch -- is
``docs/flight-calibration-procedures.md``. Read that before capturing anything;
this file only says how to build and run.

Nothing here drives a motor, a PWM channel or any output but the console and the
flow sensor's chip select, so it is safe with the battery disconnected and safe
with propellers fitted.

Console keys
============

==============  ==============================================================
``r`` / SPACE   start a capture (resets the accumulator)
``s`` / ENTER   stop the capture and print the ``FLOWCAL RUN`` summary
``z``           zero the accumulator without leaving the current mode
``v``           toggle per-sample lines (for a short look; **not** for a run --
                ``printk`` busy-waits on the FPGA console)
``?``           reprint the key list and the current configuration
==============  ==============================================================

Build and run
=============

The application uses a target-supplied ``riskybird,pmw3901`` devicetree
contract for its SPI bus, software chip select, reset and LED control.

ESP32-C6:

.. code-block:: console

   ./rb build pmw3901_test --target esp32c6
   ./rb run pmw3901_test --target esp32c6 --port /dev/ttyACM0

FPGA (the drone shell, which is where the calibration is captured):

.. code-block:: console

   ./rb build pmw3901_test --target arty200t \
     --config RocketArty200TDroneFullDDRConfig
   ./rb run pmw3901_test --target arty200t \
     --config RocketArty200TDroneFullDDRConfig \
     --program-usb-location <bitstream-ftdi> --usb-location <debug-ftdi>

Tunables (``RB_EXTRA_CFLAGS=-D...``)
====================================

=========================  =======  ==================================================
``FLOW_CAL_PERIOD_MS``     10       read period; matches ``FLOW_PERIOD_MS`` in flow.c
``FLOW_CAL_MIN_SQUAL``     19       surface-quality floor; matches ``FLOW_MIN_SQUAL``
``FLOW_CAL_IDLE_MS``       1000     IDLE heartbeat period
=========================  =======  ==================================================

The read period does **not** change the answer -- the sensor accumulates
between reads, so the sum is rate-independent -- but keeping it equal to the
flight build's removes one difference between the number measured here and the
number used in flight.

Notes
=====

The chip-ID line is a gate, not decoration: it must read ``0x49`` **and**
``0xB6``. The driver discards one read before the pair, because on this bench
the first register read after the chip-select sequence returns ``0x92`` --
``0x49`` with its bits in the other order -- while the very next read is
correct.
