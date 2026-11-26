# Firmware review and architecture

## What changed

The original root-level `drone.c` and `controller.c` are C++ Arduino programs despite their `.c` extensions. They now live under separate PlatformIO source folders and compile as independent targets:

- `src/drone/main.cpp` is the receiver entry point. `FlightController.cpp` owns its startup and flight loop; `SensorManager.*` wraps the IMU and barometer; PID math and motor mixing/output are in `PidController.*` and `MotorOutput.*`.
- `src/controller/main.cpp` is the remote entry point. `ControllerApp.cpp` owns its inputs, buttons, and ESP-NOW transmission.
- `include/common/command_packet.hpp` defines the shared, 28-byte packet layout.
- `include/drone/board_config.hpp` contains the receiver pinout and pulse limits.

The folder refactor preserves the original receiver and controller behavior. It moves the receiver's sensor reads, PID math, and motor mixing/output into focused files and gives each firmware its own entry point and PlatformIO environment. The callback behavior, control thresholds, calibration math, mode behavior, failsafe timing, and transmitter button logic remain the same as the original `src` programs. The structural extraction is not a safety fix; the issues below remain in the current logic.

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

## Difference from ESP-FC and which fits this drone

ESP-FC is a full flight-controller product with an abstract hardware layer and many selectable devices and protocols. The comparison below is based on this workspace's `esp-fc-master` reference, not on a claim that the two flight loops are interchangeable.

| Area | This drone's original logic | ESP-FC reference | Fit for this drone |
|---|---|---|---|
| Structure | Two Arduino programs: receiver and handheld controller. The receiver owns its flight task and global state. | Thin `src/main.cpp`; `Hal`, `Espfc`, `Control`, `Sensor`, `Device`, `Output`, `Input`, `Connect`, `Telemetry`, `Blackbox`, and utility modules. | ESP-FC's boundaries are a good architecture reference. Keeping the two programs and packet behavior preserves the custom setup. |
| Hardware abstraction | Direct Arduino, FreeRTOS, ESP-NOW, LEDC, Wire, and Adafruit calls. Fixed WROOM pin map. | `Hal` isolates GPIO, bus, timer, serial, task, and ESC drivers; pin resources and model settings are configurable. | The manual's fixed PCB map makes the custom receiver mapping the closer fit. HAL patterns would help future portability. |
| IMU | Adafruit LSM6DS3TR-C over I2C at `0x6A`, 208 Hz. | Driver set includes MPU and ICM devices plus LSM6DSO; it does not list an LSM6DS3TR-C driver. | The custom source already targets the documented sensor. ESP-FC needs a compatible sensor driver or hardware change. |
| Other sensors | BMP280 at `0x76` for relative altitude. The supplied manual also documents GNSS, compass, and battery monitoring, which this logic does not use. | Supports several barometers, magnetometers, and GPS protocols and has voltage sensing infrastructure. | ESP-FC offers a better base for those additional sensors, after confirming the exact PCB peripherals and writing any missing drivers. |
| Attitude estimate | Gyro rates plus accelerometer angles, simple complementary fusion, and per-axis Kalman filters. Yaw angle is not fused or used for heading hold. | Provides AHRS alternatives and configurable gyro filters. The loop/controller path is designed around its model and HAL. | The custom estimator is simpler and tied to this IMU; ESP-FC has more tuning options, but is not a drop-in estimator replacement. |
| Stabilization | Fixed angle-to-rate outer control (`4.0`) followed by fixed PID gains and feed-forward for roll, pitch, and yaw rate. | Separate configurable outer and inner loops, rate profiles, PID settings, filtering, and mixer modes. | ESP-FC offers more tuning and control modes. The current code's constants and behavior should remain the baseline until measured on this airframe. |
| Altitude | BMP280-based hold is implemented, activates from steady throttle, clamps the target at 10 m, and adds PID output directly to throttle pulse. | Altitude hold is listed as a TODO in this reference version. | The custom code is the only one of the two with altitude hold here, but its mode and output scaling need bench and flight validation. |
| Motor outputs | Four fixed PWM signals at 416 Hz and 1000–2000 us; custom Quad-X arithmetic with a separate yaw-bias mode. | Configurable mixer rules and ESC protocols including PWM, OneShot, MultiShot, and DShot. | The custom matrix matches the documented motor pin assignments more directly. ESP-FC supports more ESC types and mixer configuration. |
| Transmitter/input | Custom ESP32 remote sends a seven-field ESP-NOW packet every 20 ms to a hard-coded MAC; joystick centers are learned at startup; trims/mode buttons are bespoke. | Supports PPM, SBUS, IBUS, CRSF, and ESP-NOW receiver options, plus Betaflight-compatible configuration. Its ESP-NOW transmitter binds automatically after power-up. | The manual's documented remote control labels differ from this custom controller's assignments. Neither mapping should be assumed until checked against the actual remote PCB. |
| Failsafe | Tilt over 75 degrees disarms; radio timeout disarms after 1 s and requests power-latch cutoff after 60 s. A later armed packet can re-arm. Kill mode 99 cuts the latch from the callback. | Has receiver failsafe handling, modes, and configuration/reporting paths. | ESP-FC has a broader failsafe design. The existing custom thresholds and behavior should be preserved for now but are a priority for a separately reviewed safety change. |
| Debug/config | PID gains, pins, and behaviors are hard-coded; there is no telemetry or flight log in this source. | CLI/MSP, telemetry, Blackbox, stats, storage, and extensive unit-test areas. | ESP-FC is substantially stronger for tuning and diagnosis. Those are useful additions after the current hardware behavior is verified. |
| Timing | Dedicated FreeRTOS task with a 5 ms delay-until loop; sensor acquisition is over I2C. | Hardware-timer gyro loop and multi-task scheduling; README reports up to 4 kHz on ESP32 with SPI gyro. | ESP-FC has higher-rate infrastructure. Its SPI hardware assumptions do not match this receiver's current I2C sensor wiring. |

For this particular drone, the custom source is the more direct hardware fit today because it uses the documented receiver pins, LSM6DS3TR-C, BMP280, and matching custom remote protocol. ESP-FC is the stronger software architecture and has more control, input, ESC, logging, and configuration features. The practical direction is to retain the original drone behavior while borrowing ESP-FC's module boundaries and then improve the custom safety and diagnostics in separately reviewed steps. Copying ESP-FC's control loop wholesale would require porting its sensor, receiver, mixer, timing, and board layers first.

## Build evidence and flashing status

Both PlatformIO environments compiled successfully with pioarduino platform `55.03.36`, Arduino ESP32 Core `3.3.6`, and the `upesy_wroom` target. These builds use the original flight and controller behavior after the structural extraction:

```text
pio run -e drone       SUCCESS
pio run -e controller  SUCCESS
```

The receiver build used 46,264 bytes RAM and 976,315 bytes of its 1,310,720-byte application partition. The controller build used 44,352 bytes RAM and 930,411 bytes of that partition. No unit or flight tests were run. No firmware was uploaded: the only detected serial ports were Bluetooth COM3/COM4, with no USB-connected ESP32.

The supplied working directory was an extracted source bundle without Git metadata. The changes were pushed against the remote repository's `main` branch on `codex/esp32-firmware-modularization`.
