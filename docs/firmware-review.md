# Firmware review and architecture

## What changed

The original root-level `drone.c` and `controller.c` are C++ Arduino programs despite their `.c` extensions. They now live under separate PlatformIO source folders and compile as independent targets:

- `src/drone/main.cpp` is the receiver entry point. `FlightController.cpp` owns its startup and flight loop; PID math and motor mixing/output are in `PidController.*` and `MotorOutput.*`.
- `src/controller/main.cpp` is the remote entry point. `ControllerApp.cpp` owns its inputs, buttons, and ESP-NOW transmission.
- `include/common/command_packet.hpp` defines the shared, 28-byte packet layout.
- `include/drone/board_config.hpp` contains the receiver pinout and pulse limits.

The receiver now rejects malformed or out-of-range packets, does not perform power switching or block inside the ESP-NOW receive callback, and latches a disarm after link loss or excessive tilt until it receives an explicit disarm command. Motor, IMU, barometer, radio initialization, and startup calibration failures leave the motors stopped.

## Manual and code comparison

The Nabhyan manual supplied in the original workspace documents an ESP32-WROOM receiver. Its receiver assignments agree with the current receiver source:

| Receiver function | Manual and firmware |
|---|---|
| Motors FL/RL/RR/FR | GPIO12/13/14/15 |
| Power latch | GPIO4 |
| LEDs | GPIO18/19 |
| I²C SDA/SCL | GPIO21/22 |
| LSM6DS3TR-C / BMP280 | I²C; current source uses addresses `0x6A` / `0x76` |

The manual also documents GNSS on GPIO25/26, battery monitoring on GPIO27, and a magnetometer on the sensor bus. The current receiver firmware does not implement those features. Its altitude hold is a basic BMP280 loop; it has no GNSS navigation, compass heading hold, or battery failsafe.

The transmitter code does not match the manual in several important ways. The manual maps GPIO34 to yaw and GPIO35 to the left-stick forward/back input; the source maps GPIO34 to throttle and GPIO35 to yaw. GPIO36/39 are the right-stick axes in both. The manual describes S1 as speed mode, S3 as landing, S5 as return-to-home, S6 as emergency motor stop, S7 as LED control, and S8 as calibration. The source assigns S1/S3 to trim-step/reset, S5/S6 to roll and yaw trim, and S7/S8 to pitch trim. Verify whether the installed remote PCB or this custom control scheme is authoritative before flashing it.

## Main issues found in the original source

1. **Unsafe packet handling.** The receiver copied `sizeof(packet)` bytes without checking `len`, accepted any sender, and trusted arbitrary control values. It also changed motor and power GPIOs and spun forever inside the radio callback.
2. **Disarm was not latched.** A crash tilt or one-second radio timeout set `armed` false, but the next armed packet immediately set it true again.
3. **Sensor failures were ignored.** IMU and barometer setup results were discarded. Calibration divided by the planned sample count even when reads failed. During flight, failed IMU reads left the previous motor PWM in place.
4. **Control-loop timing was weakly bounded.** A delayed loop could produce a very small or irregular `dt`, affecting derivative and integral terms. The loop also requests a 5 ms cadence while the IMU is configured for 208 Hz and I²C reads happen on every cycle.
5. **Altitude mode behavior is underspecified.** It activates based on throttle stability, forces a 10 m target once the limit is reached, and adds a PID output directly to the base ESC pulse. The throttle-to-altitude behavior, saturation handling, and barometer error response need bench evaluation.
6. **Controller input and link handling are fragile.** Stick centers are sampled only at boot with no range or movement checks; button inputs have no debounce; send/peer initialization results and send status are ignored; the MAC address is hard-coded; there is no receiver telemetry or link-quality display.
7. **Control and hardware assumptions are embedded in monolithic files.** Packet definitions, sensor logic, PID code, mixer math, and GPIO setup were difficult to review independently. The original `.c` suffixes also prevented a normal PlatformIO C++ build.
8. **No motor-mixer or orientation verification was present.** The axis signs, motor numbering, propeller directions, PWM pulse range, and failsafe behavior had no automated or bench evidence.

## ESP-FC comparison

The reference project has a much broader, well-separated design: a thin `src/main.cpp`; a HAL layer; dedicated sensor, device, controller, mixer, input, telemetry, CLI/MSP, storage, and blackbox modules; and unit tests for control, sensors, input, and motor output. It supports multiple sensor/receiver/ESC protocols and configurable pin maps.

This project is intentionally narrower: two ESP32 boards, a custom ESP-NOW packet, one fixed receiver pinout, LSM6DS3TR-C plus BMP280, and four PWM ESC outputs. The new split adopts the reference's separation of entry points, control math, and motor output without importing its unrelated protocols or hardware abstraction. Useful next improvements are a dedicated sensor abstraction, a versioned and sequenced packet with sender validation, explicit failsafe states and telemetry, battery voltage monitoring, and unit tests for PID, mixing, packet validation, and input mapping.

## Build evidence and flashing status

Both PlatformIO environments compiled successfully with pioarduino platform `55.03.36`, Arduino ESP32 Core `3.3.6`, and the `upesy_wroom` target:

```text
pio run -e drone       SUCCESS
pio run -e controller  SUCCESS
```

The receiver build used about 46 KB RAM and 0.98 MB of its 1.31 MB application partition. The controller build used about 44 KB RAM and 0.93 MB of that partition. No unit or flight tests were run. No firmware was uploaded: the only detected serial ports were Bluetooth COM3/COM4, with no USB-connected ESP32.

The supplied working directory was an extracted source bundle without Git metadata. These changes are being staged against the remote repository's `main` branch on a new feature branch, as requested.
