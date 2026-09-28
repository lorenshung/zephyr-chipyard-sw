Legacy combined sensor bring-up
###############################

This imported ESP-only application combines PMW3901, ADS7128 GPIO6, and one
readdressed VL53L1X. The pinned Zephyr tree now exposes
``vl53l1x_reinit()`` through a public driver header, and the application
configures and links for ESP32-C6.

Do not use this application as the basis for the combined autonomy sensor
pipeline. The separate diagnostics and target devicetree contracts in the
parent workload README are the maintained integration points. This
application also exercises a legacy single-ToF GPIO6 topology; the current
drone contract has four VL53L5CX XSHUT signals on ADS7128 GPIO1--GPIO4.

Successful linking is not a new physical-hardware result. PMW3901 traffic,
VL53L1X readdressing, and sensor output still require a captured ESP run.
