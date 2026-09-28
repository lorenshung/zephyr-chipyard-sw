# RiskyBird workloads

This directory owns board-independent Zephyr applications. Target-specific
pins, buses, addresses, and peripheral instances live under
`hardware/zephyr/targets/`; `tools/rb/boards.py` is the machine-readable
workload/target registry, and `./rb workloads` prints it, legacy notes
included. Where this page and `rb workloads` disagree, the registry is right.

Three registered workloads are built in place from a backend rather than from
this directory: `flight_controller` and `flight_estimator` from
`backends/zephyr-chipyard-sw/samples/rose_flight_controller`, and `dronet`
from `backends/modelblaster/harness`.

The sensor and actuator diagnostics were imported from
`backends/zephyr-chipyard-sw` remote ref `origin/riskybird-bringup` at commit
`2bcbd635bffdb9dba6596aaf326e0cc2be3d8ff6`. They are now top-level
applications and are selected with the same `rb build` and `rb run` interface
as everything else.

## What is available

Targets: **FPGA** is `arty100t`, `arty200t` and `ku040`; **ESP** is
`esp32c6`; **FireSim** is `firesim`, whose base devicetree has a console and
memory and no I2C, SPI, GPIO or PWM, so only peripheral-free workloads are
registered for it.

On `arty200t` every drone workload builds against
`RocketArty200TDroneFullDDRConfig` unless `--config` says otherwise; the board
default, `RocketArty200TOspiConfig`, has none of the drone periphery.

### The drone stack

| Workload | Modes | Targets | Function |
| --- | --- | --- | --- |
| `flight_controller` | `default`, `telem`, `flow`, `esp-motors`, the flight presets (`t16-hover-0.6m` ... `t27-hover-1.6m`), `imu-test`; legacy `motors`, `autoflight` | arty200t | RoSE estimator plus the PID cascade. `esp-motors` is the mode that flies; each preset is `esp-motors` plus the values one flight used (`tools/flight/presets/`) |
| `flight_estimator` | `default`, `flow` | arty200t | The same sources with the controller compiled out |
| `bootup_check` | `default`; legacy `motors` | arty200t | Every peripheral in boot order, ending in a board-ready verdict |
| `esp_bridge` | `default`, `motors` | ESP | WiFi modem for the flight controller; `--mode motors` also owns the four motor gates, with the 120 ms failsafe and the dashboard ESTOP |
| `motor_duty` | `default` | arty200t | Fixed duty on chosen motors through the ESP (`rb fly motors`); props on, restrained |
| `esp_uart` | `default` | arty200t | The FPGA end of the ESP link: TX proof and RX rate sweep |
| `camera_photo` | `default` | arty200t | One full HM01B0 frame into RAM for JTAG readout |
| `motor_safe` | `default` | ESP | Park the ESP side of the motor nets as pulled-down inputs |
| `dronet` | `default` | arty200t | DroNet through the ModelBlaster harness; start it with `rb modelblaster run` |

### Diagnostics

| Workload | Modes | Targets | Function |
| --- | --- | --- | --- |
| `hello_world` | `default` | FPGA, ESP, FireSim | Console bring-up probe |
| `ddr_stress` | `default` | FPGA, FireSim | Four patterns over a main-memory window; needs a DDR shell (96 MiB floor) |
| `i2c_scanner` | `default` | FPGA, ESP | I2C address scan |
| `ads7128_test` | `default` | FPGA, ESP | ADS7128 identity, register and GPIO diagnostic |
| `bmi088_test` | `default` | FPGA, ESP | BMI088 accelerometer, gyroscope and temperature stream |
| `pmw3901_test` | `default` | FPGA, ESP | PMW3901 SPI optical-flow stream |
| `vl35l5cx_test` | `address`, `ranging` | FPGA, ESP | The ToF array behind the ADS7128: `address` everywhere, `ranging` needs 192 KiB, so not on a scratchpad config |

### Legacy

These still build -- main has images and docs that name them -- but
`rb workloads` marks them and every build prints the note below as a warning.

| Workload | Targets | Superseded by |
| --- | --- | --- |
| `motor_1` ... `motor_4` | FPGA, ESP | `flight_controller --mode esp-motors` with `esp_bridge --mode motors`, and `motor_duty` for a bench spin. They drive the FPGA PWM pins: motor 4's pin F13 is dead, and they fight the ESP for the gates when it runs `esp_bridge --mode motors` |
| `esp_motors` | ESP | `esp_bridge --mode motors`, which drives the same gates with the same failsafe and also carries telemetry and the ESTOP |
| `sensor_check` | arty200t | `bootup_check` |
| `state` | FPGA, ESP, FireSim (`bmi088` not on FireSim) | `flight_controller` / `flight_estimator`. On a stationary board this Madgwick settles to roll -178.7 deg where the flight estimator reads 0.10 |
| `mpc` | FPGA, ESP, FireSim, rose-spike (`hil` not on FireSim) | `flight_controller`. `--mode rose` needs `backends/RoSE`, which is on main, not on the setup branch |
| `camera_bringup` | arty100t, arty200t | `bootup_check`'s camera stage and `camera_photo` |
| `flight_controller --mode motors`, `--mode autoflight`, `bootup_check --mode motors` | arty200t | The same FPGA-PWM problem as `motor_N`; the flying path is `esp-motors` + `esp_bridge --mode motors` |

Removed: `i2c_expander` and `tof_sensor` (VL53L0X; the drone carries VL53L1X
and VL53L5CX), and `sensor_bringup` (the ESP-hosted flight stack is gone).

“Software builds” means Zephyr configured and linked the image -- not a
hardware result.

The directory name `vl35l5cx_test` is retained for continuity with the source
branch; the sensor implemented by the workload is the VL53L5CX.

The pinned Zephyr tree contains the VL53L5CX grid API used by full ranging.
That backend addition is kept at the Zephyr driver boundary; RiskyBird
workloads consume the stable top-level contract under
`integration/zephyr/tof/`.

## Build and run

Install the ESP and ST HAL projects required by the additional targets once:

```console
rb zephyr setup-targets
```

Usually unnecessary. `rb env setup zephyr` -- step 2b of `docs/setup.md` -- runs
a west update that already fetches both HALs, and `rb doctor` reports them `ok`
afterwards. Run this only if that check says otherwise.

Discover the registered interface and build individual diagnostics:

```console
./rb targets
./rb workloads

./rb build i2c_scanner --target esp32c6
./rb build ads7128_test --target esp32c6
./rb build bmi088_test --target esp32c6
./rb build pmw3901_test --target esp32c6
./rb build vl35l5cx_test --target esp32c6 --mode address
```

The drone stack builds against the drone shell by default:

```console
./rb build bootup_check --target arty200t
./rb build flight_controller --target arty200t --mode esp-motors
./rb build esp_bridge --target esp32c6 --mode motors
```

With the ESP drone connected, `run` builds, flashes, and opens the console:

```console
./rb run bmi088_test --target esp32c6 --port /dev/ttyACM0
./rb run vl35l5cx_test --target esp32c6 --mode address \
  --port /dev/ttyACM0 --capture-duration 30
```

`rb capture` can record an already-running image. It stores the serial stream,
operator input, ELF hash, target/workload/mode, and repository revisions in a
new results directory:

```console
./rb capture bmi088_test --target esp32c6 --port /dev/ttyACM0 \
  --duration 30
```

FPGA I2C diagnostics use the OSPI configurations (the board default):

```console
./rb build bmi088_test --target arty100t
./rb build vl35l5cx_test --target ku040 --mode address
```

## The ToF array

The drone carrier is meant to carry three ToF sensors: two side VL53L5CX and
the down-facing VL53L1X, all with their XSHUT lines on the ADS7128 at `0x17`.
`vl35l5cx_test` declares four slots, on ADS7128 GPIO1, GPIO2, GPIO3 and GPIO6
(the channels a scan found gating anything); GPIO6 is the down VL53L1X. On
the current board only GPIO1's side sensor and the down sensor answer, so a
missing side sensor is a real fault, not a config to trim.

All the parts share the default address `0x29`. `address` mode:

1. configures the ADS7128 channels as push-pull XSHUT outputs;
2. holds every sensor in reset;
3. releases one additional sensor at a time, detecting whether it is a
   VL53L5CX or a VL53L1X;
4. changes its volatile address to `0x31`, `0x32`, `0x33`, or `0x34`; and
5. leaves earlier sensors awake while bringing up the next sensor.

Keeping earlier sensors awake is required because asserting XSHUT again would
discard their volatile address and return them to `0x29`. Address mode then
probes each device identity continuously without depending on a private
VL53L5CX Zephyr driver.

Full ranging is a separate `ranging` mode: same XSHUT and address sequence,
then the Zephyr VL53L5CX devices at their final addresses, emitting target
count, status, and signed millimeter distance per row-major zone. Only the
side slots have a VL53L5CX node in the target overlays; the down VL53L1X and
any slot that did not come up are skipped. The linked Arty image is ~137 KiB,
so `rb` requires 192 KiB addressable RAM on an FPGA target. Scratchpad-only
configs take `address` mode but not `ranging`.

## Motor safety

The motors are brushed, switched by low-side SI2302 FETs off the battery;
nothing turns without the LiPo connected. In the configuration that flies, the
ESP32-C6 owns all four gates (`esp_bridge --mode motors`) and the FPGA only
sends it duty frames (`flight_controller --mode esp-motors`, `motor_duty`).
FPGA ball F13 -- motor 4's gate -- is dead as an output, so no FPGA-PWM
workload can drive motor 4.

The legacy `motor_1` ... `motor_4` diagnostics force every PWM output to zero
at boot and require an explicit `s` command before applying the default 10%
diagnostic duty cycle; `x`, space, or `q` stops and disarms the outputs.
Remove propellers before using them or any other FPGA-PWM mode.

The carrier connects ESP32-C6 GPIO20/21/22/23 and the FPGA PWM outputs to the
same four MOSFET gates without arbitration. Before letting the FPGA drive the
gates, flash `motor_safe` to the ESP. Its GPIO hogs make those four pins
inputs with pull-downs as soon as the Zephyr GPIO driver is ready, and
`main()` reasserts that state:

```console
./rb flash motor_safe --target esp32c6 --port /dev/ttyACM0 --build
```

The SiFive PWM peripheral has four comparators, but comparator 0 establishes
the period; usable output channels are 1–3. The logical FPGA design therefore
uses channels 1–3 of `pwm0` for motors 1–3 and channel 1 of `pwm1` for motor
4.
