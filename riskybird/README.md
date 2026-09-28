# Unified RiskyBird firmware draft

Firmware moved byte-for-byte from RiskyBird main into workloads/, integration/zephyr/
and hardware/zephyr/. Pass RB_ROOT=<this directory> to existing firmware CMake
interfaces. Repository-level rb orchestration remains in the parent RiskyBird repo.

Two controller source variants remain explicit while behavior is reconciled:

* samples/rose_flight_controller preserves the flown FullDDR/ESP motor-link source
  at 44f79221. Its ROSE_CAMERA captures one still for post-landing CPU drain/JTAG.
* samples/rose_flight_controller_dma preserves the FPGA camera DMA, UART TX DMA,
  and accelerated controller experiments from upstream 1e74c5ea. rose_fc_dronet
  consumes this variant. It is a bench pipeline; the old HTTP ESP bridge is not
  a replacement for workloads/esp_bridge motor mode and its failsafe.

DMA UART requires the DmaUart hardware MMIO block. Standalone DMA controller
uses 50 MHz UART peripheral clock; FC+DroNet sets 35 MHz. Neither implies new
hardware validation. Experimental RTOS patches under rose_fc_dronet/patches are
not applied automatically. Existing co-residency and vector-state issues remain.

## Source bootstrap

Install west in an environment and run scripts/bootstrap-riskybird.sh from the
backend repository. It initializes the gitlinks and an isolated zephyr_ws west
workspace using zephyr_ws/manifest/west.yml. The manifest and gitlink identify
the same RTOS commit; XNNPACK, ExecuTorch and zephyr-rose also have immutable pins.
The RTOS draft requires publication to lorenshung/zephyr before remote bootstrap.
Run west blobs fetch hal_espressif from zephyr_ws for ESP WiFi builds when needed.

Use the canonical external Chipyard checkout provided by RiskyBird; this backend
has no Chipyard gitlink. Pass compiler/Spike paths explicitly for model generation.
No developer-specific checkout is a supported fallback. Build recipes may use an
existing SDK through ZEPHYR_SDK_INSTALL_DIR and an already active west environment.
