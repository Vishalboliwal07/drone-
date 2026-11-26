# ESP32 Drone Firmware

This project contains separate firmware for the drone receiver and its ESP-NOW remote controller. The receiver uses the NABHYAN WROOM board pinout documented in the included manual.

## Build

Install PlatformIO, connect the relevant ESP32 by USB, and run:

```sh
pio run -e drone
pio run -e controller
```

The default target is `drone`. The project uses the pioarduino stable ESP32 platform because the receiver's LEDC output uses the Arduino Core 3 API. The board is configured as `upesy_wroom`, a generic WROOM ESP32 target. If your hardware is the Lolin32 board, verify its flash configuration and pin wiring before changing the target.

To build and upload a target:

```sh
pio run -e drone -t upload --upload-port COMx
pio run -e controller -t upload --upload-port COMx
```

Replace `COMx` with the USB serial port for the board connected at that time. Each environment creates a separate firmware image.

## Layout

```text
include/common/command_packet.hpp  Shared ESP-NOW packet layout
include/drone/board_config.hpp     Receiver pin and motor settings
src/drone/                          Receiver entry point, flight loop, PID, motor output
src/controller/                     Remote controller entry point and input/radio logic
esp-fc-master/                      Reference flight-controller project
```

## Hardware

The receiver uses GPIO12/13/14/15 for motors FL/RL/RR/FR, GPIO4 for the power latch, GPIO18/19 for status LEDs, and I²C on GPIO21/22. The firmware expects an LSM6DS3TR-C at address `0x6A` and a BMP280 at `0x76`.

The transmitter source currently uses the pin assignments and button actions in the original project README. Those do not fully match the transmitter map in the NABHYAN manual supplied for this review. Read [the firmware review](docs/firmware-review.md) and confirm the PCB version before uploading controller firmware.

## Safety and known limitations

Build success confirms compilation only. Motor direction, receiver control response, radio-loss behavior, altitude hold, and the controller's physical input map still need bench validation with propellers removed. Do not use the built images for flight until those checks pass. The full issue list and ESP-FC comparison are in [the firmware review](docs/firmware-review.md).
