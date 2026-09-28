# RiskyBird MPC

The canonical single-drone, scalar Rocket integration of Accelerated-TinyMPC
with RiskyBird quadrotor problem data. It drops the backend sample's mandatory
RVV, four worker threads, CPU pinning, and CMake-comment source selection.

```console
rb build mpc --target arty100t
rb run mpc --target arty100t
rb build mpc --target arty100t --mode hil
rb hil replay mpc-hover --executor rose-spike --clock-mhz 1000
```

The HIL mode retains the existing binary framing:

- request: `DE AD BE EF`, drone count, then 12 little-endian floats per drone;
- response: `DE AD BE EF`, drone id 0, four little-endian control floats, and
  a little-endian `uint32_t` solve time in nanoseconds.

Only the first drone is solved. Extra request states are consumed so the stream
stays synchronized. HIL mode owns `uart0` and therefore disables text console
output.

RoSE mode replaces the UART framing with request `0x12` for the target-relative
12-state vector and action `0x20` for the four controls. The top-level HIL
command supervises Spike and PyBullet and writes a trace, summary, logs, and an
SVG plot. See [the RoSE MPC flow](../../docs/rose-mpc-flow.md).

`rb` links against each target's actual RAM and uses a three-step horizon on
KU040 against Arty100T's ten-step — a holdover from the 32 KiB scratchpad era
that the DDR4-backed KU040 configs make reviewable. The 12-state model,
dynamics, costs, constraints, and scalar solver are identical. The active
horizon is printed in startup telemetry as `horizon=`.

For binary HIL, configure the FPGA and load the image without opening the text
console, then connect the HIL client to the SiFive UART:

```console
rb run mpc --target arty100t --mode hil --no-console
```
