# Drone Firmware Project

This repository contains firmware code for a drone using **PlatformIO** and **ESP32**.

---

## Adding a New Library

1. Create a new folder inside the `lib` directory:

### lib/Utilities


2. Inside the library folder, add a `library.json` file:

```json
{
  "name": "Utilities",
  "version": "1.0.0",
  "description": "Drone firmware utilities",
  "export": {
    "include": [
      "src",
      "lib_droneFunctions.h",
      "lib_droneConstants.h",
      "lib_utilities.h",
      "lib_globalConstants.h",
      "lib_functions.h"
    ]
  },
  "frameworks": "*",
  "platforms": "*"
}
```

## Adding a New Source Target

Each source file requires its own environment in platformio.ini.

### Base Environment

```ini
[env]
platform = https://github.com/pioarduino/platform-espressif32/releases/download/stable/platform-espressif32.zip
board = upesy_wroom
framework = arduino
lib_compat_mode = strict
lib_ldf_mode = chain
monitor_speed = 115200

```

### Environments
```ini
[env:MAIN_controller]
lib_deps =
    Adafruit LSM6DS
    adafruit/Adafruit BMP280 Library@^2.6.8
    Utilities
build_src_filter = -<**/*.cpp> +<**/main-controller.cpp>


[env:MAIN_drone]
lib_deps =
    Adafruit LSM6DS
    adafruit/Adafruit BMP280 Library@^2.6.8
    Utilities
build_src_filter = -<**/*.cpp> +<**/main-drone.cpp>


[env:TESTING_motors]
lib_deps =
    Adafruit LSM6DS
    adafruit/Adafruit BMP280 Library@^2.6.8
    Peripherals
build_src_filter = -<**/*.cpp> +<**/test-motors.cpp>


[env:TESTING_macAddress]
lib_deps =
    Adafruit LSM6DS
    adafruit/Adafruit BMP280 Library@^2.6.8
build_src_filter = -<**/*.cpp> +<**/find-mac-address.cpp>


```

# Build and Upload (PlatformIO)

1. Click Build (✔ check mark)
2. Click Upload (➜ arrow)
3. Select the COM port from the bottom bar
4. Click Upload
5. Open Monitor to view serial output

