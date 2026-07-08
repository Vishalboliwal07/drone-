# drone

# 🚁 ESP32 Custom Flight Controller & Remote (ESP-NOW)

A fully custom, from-scratch Quadcopter Flight Controller and Transmitter built using two ESP32 microcontrollers. They communicate using the ultra-low-latency **ESP-NOW** protocol.

Unlike basic DIY drones, this project features professional-grade software mechanics including **PID stabilization with anti-windup, cubic exponential joystick curves, in-flight dynamic trimming, automatic joystick zeroing, and a hardware kill switch.**

## ✨ Key Features

* **ESP-NOW Protocol:** Peer-to-peer 2.4GHz communication bypassing standard Wi-Fi latency.
* **Auto-Calibration:** Joysticks automatically zero out drift upon remote startup.
* **Dynamic In-Flight Trimming:** Use dedicated buttons to adjust Roll and Pitch mid-flight to counter wind or hardware imbalances.
* **Adjustable Trim Steps:** Increase or decrease the "aggressiveness" of the trim buttons dynamically (5, 10, 20, 25, 50, 75, 100 increments).
* **Exponential Stick Curves:** "Expo" math applied to the joysticks for smooth hovering in the center and aggressive flips at the edges.
* **PID Anti-Windup:** Prevents the drone from trying to correct itself while idling on the ground (stops the "takeoff flip").
* **Crash Detection (Kill Switch):** Automatically cuts motor power if the drone tilts beyond 75 degrees.

---

## 🛠️ Hardware Requirements

### 1. The Drone (Receiver)

* **Microcontroller:** ESP32 (WROOM-32 or similar)
* **IMU (Gyro/Accel):** Adafruit LSM6DS3TR-C (I2C Address: `0x6A`)
* **Barometer (Altitude):** Adafruit BMP280 (I2C)
* **Motors & ESCs:** 4x Brushless Motors with standard ESCs (416Hz PWM)
* **Status LEDs:** 2x standard LEDs

### 2. The Transmitter (Remote)

* **Microcontroller:** ESP32
* **Joysticks:** 2x Analog Thumbsticks (Throttle, Yaw, Pitch, Roll)
* **Buttons:** 9x Momentary Push Buttons (Arm, Power, Trims, Steps)
* **Feedback:** 1x Active Buzzer, 1x LED

---

## 🔌 Pinout Mapping

### Drone (Receiver)

| Component | ESP32 Pin | Notes |
| --- | --- | --- |
| **Motor FL (Front Left)** | `GPIO 12` | PWM Output |
| **Motor RL (Rear Left)** | `GPIO 13` | PWM Output |
| **Motor RR (Rear Right)** | `GPIO 14` | PWM Output |
| **Motor FR (Front Right)** | `GPIO 15` | PWM Output |
| **Power Latch** | `GPIO 4` | Keeps battery circuit closed |
| **Status LED 1** | `GPIO 18` | Status indicator |
| **Status LED 2** | `GPIO 19` | Status indicator |
| **I2C SDA / SCL** | Default I2C | Connects to LSM6DS3TR & BMP280 |

### Transmitter (Remote)

| Component | ESP32 Pin | Notes |
| --- | --- | --- |
| **Throttle (Left Y)** | `GPIO 34` | Analog Input |
| **Yaw (Left X)** | `GPIO 35` | Analog Input |
| **Roll (Right X)** | `GPIO 36` | Analog Input |
| **Pitch (Right Y)** | `GPIO 39` | Analog Input |
| **Arm Button (S4)** | `GPIO 26` | Toggles Motors ON/OFF |
| **Power Button (S10)** | `GPIO 13` | Hold 2s to Sleep/Wake |
| **Right Trim (S5)** | `GPIO 16` | Rolls Left |
| **Left Trim (S6)** | `GPIO 18` | Rolls Right |
| **Back Trim (S7)** | `GPIO 17` | Pitches Forward |
| **Front Trim (S8)** | `GPIO 19` | Pitches Backward |
| **Increase Step (S1)** | `GPIO 12` | Increases Trim Aggressiveness |
| **Decrease Step (S2)** | `GPIO 25` | Decreases Trim Aggressiveness |
| **Reset Trims (S3)** | `GPIO 27` | Sets Roll/Pitch Trims to 0 |
| **Buzzer** | `GPIO 15` | Audio Feedback |
| **Status LED** | `GPIO 21` | Visual Feedback |

*(Note: All buttons use `INPUT_PULLUP` and should be wired to connect the GPIO pin to GND when pressed).*

---

## 📚 Dependencies

You will need to install the following libraries in the Arduino IDE Library Manager:

1. `Adafruit LSM6DS` (by Adafruit)
2. `Adafruit BMP280 Library` (by Adafruit)
3. `Adafruit Unified Sensor` (by Adafruit)
4. `SimpleKalmanFilter` (by Denys Sene)

---

## 🚀 Installation & Setup

1. **Find the Drone's MAC Address:** Run a basic MAC Address scanner sketch on the Drone's ESP32.
2. **Update the Remote Code:** Open the Transmitter code and paste the Drone's MAC address into this array:
```cpp
uint8_t droneMacAddress[] = {0x68, 0x25, 0xDD, 0xCC, 0xAD, 0x70}; 
```
3. **Upload:** Flash the Transmitter code to your Remote, and the Receiver code to your Drone. Ensure CPU Frequency is set to `240MHz` for both.

---

## 🎮 Flight Instructions

### 1. Pre-Flight

* Place the drone on a perfectly flat, level surface.
* **Power on the Remote FIRST.** Do not touch the joysticks for the first 2 seconds (it is calibrating the centers). You will hear 3 beeps when ready.
* **Power on the Drone.** Wait for the fast-blinking LEDs to turn into slow-blinking LEDs (Gyro calibration complete).

### 2. Arming & Takeoff

* Ensure the throttle stick is all the way down.
* Press the **Arm Button (S4)** once. The Remote LED will turn solid, and the drone LEDs will turn solid.
* Push the throttle briskly to ~30-40% to get out of the "ground effect" wash.

### 3. Dynamic Trimming (Mid-Flight)

If the drone drifts, use the trim buttons to counter it:

* **Drifts Right?** Press **S6 (Left Trim)**.
* **Drifts Forward?** Press **S7 (Back Trim)**.

**Trim Step Sizing:**
If the trim isn't strong enough, press **S1** to increase the step size (5 -> 10 -> 20 -> 25 -> 50 -> 75 -> 100). The remote will beep to confirm. Press **S2** to decrease it.
If things get out of hand, press **S3 (Reset)** to zero out all trims instantly.

---

## ⚠️ Safety Warning

**ALWAYS REMOVE PROPELLERS** when testing code on the bench. Quadcopter motors spin at high RPMs and can cause severe injury. Only attach propellers when you are outdoors in an open space, ready to fly.
