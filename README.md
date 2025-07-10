# Payload Logger and Servo Deployment

This repository contains Arduino code for a payload that monitors and logs sensor data (altitude, acceleration, gyroscope, etc.) from an **IMU** (Inertial Measurement Unit) to an SD card at a rate of 100 Hz. Once certain landing conditions (based on altitude and acceleration thresholds) are detected, a servo mechanism is triggered to change its position. Additionally, an onboard LED provides a simple visual status indicator.

<p align="center">
  <img src="https://github.com/user-attachments/assets/0e199425-5c4e-425d-8099-5e25e6f3ebda" alt="Image 1">
</p>

<p align="center">
  <img src="https://github.com/user-attachments/assets/f163ba24-ae6e-469d-ab4a-cc56ddeccd5b" alt="Image 2">
</p>

---

## Table of Contents

- [Features](#features)
- [Hardware Requirements](#hardware-requirements)
- [Dependencies](#dependencies)
- [Project Structure](#project-structure)
- [Setup and Usage](#setup-and-usage)
- [Code Overview](#code-overview)
  - [Important Functions](#important-functions)
  - [Thresholds and States](#thresholds-and-states)
- [Configuration](#configuration)
- [License](#license)

---

## Features

- **IMU Data Acquisition**: Fetches and processes acceleration, gyro, magnetometer, and altitude data at 100 Hz.
- **Data Logging**: Uses an SD card to store sensor readings, including time stamps, IMU readings, and derived calculations (like `acceleration magnitude`).
- **Landing Detection**: Monitors altitude and acceleration to detect a landing event.
- **Servo Control**: Moves a servo once landing is confirmed. 
- **Onboard LED Indication**: Blinks while in flight, turns solid on landing.

---

## Hardware Requirements

1. **Arduino Board** (Teensy 4.1):
   - Must support hardware timers for 100 Hz logging.
2. **IMU Sensor** (VectorNAV VN100).
3. **SD Card Module** (with microSD card).
4. **Servo** (or any PWM-controlled actuator).
5. **LED** (onboard or external; the code uses the onboard LED on pin 13).
6. **Wiring**: Ensure the following pins are correctly set:
   - `SERVO_PIN` (default: **4**)
   - `LATCH_PIN` (default: **11**)
   - Onboard LED pin (default: **13**)

---

## Dependencies

- **Arduino Core** Teensyduino
- **SPI** and **SD** libraries for reading/writing to the SD card.
- **IntervalTimer** (Teensy) Hardware timer for precise timing interrupts at 100 Hz.
- Custom libraries:
  - `DataLogger.h`
  - `IMU.h`
---

## Project Structure

```
.
├── DataLogger.h         # Custom DataLogger class header
├── DataLogger.cpp       # Implementation of the DataLogger class
├── IMU.h                # Custom IMU class header
├── IMU.cpp              # Implementation of the IMU class
└── main.cpp            # Arduino sketch (setup & loop)
```

---

## Setup and Usage

1. **Clone or Download** this repository.
2. **Open the Project** in the Arduino IDE or PlatformIO.
3. **Check Pin Assignments**:
   - `SERVO_PIN` (default: `4`)
   - `LATCH_PIN` (default: `11`)
   - Ensure these match your hardware wiring.
4. **Update Thresholds (if necessary)**:
   - `landingAccMagThreshold`
   - `landingAltitudeThreshold`
   - `initialAltitudeThreshold`
5. **Install Libraries**:
   - Install `SPI`, `SD`, and any IMU-related library in the Arduino Library Manager.
6. **Compile** and **Upload** to your Arduino-compatible board.
7. **Monitor Serial Output** (optional):
   - Open the Serial Monitor at **115200 baud**.
   - You will see log statements and altitude data printouts.

---

## Code Overview

### Important Functions

- **`setup()`**  
  Initializes Serial communication, configures pins, begins IMU and SD card usage, writes the data header to the log file, and starts the 100 Hz logging timer.

- **`loop()`**  
  Runs continuously:
  1. Calls `blinkLED()` to manage LED blinking or solid on.
  2. Calls `detectLanding()` to determine if a landing event has occurred.
  3. Calls `updateServo()` to update the servo’s position if a landing is detected.

- **`logData()`** (triggered by `IntervalTimer`)  
  - Reads IMU data.
  - Logs data (time stamp, IMU readings, altitude, etc.) to the SD card.

- **`detectLanding(float altitude, float accel_mag)`**  
  - Checks if the payload has reached an initial altitude (`initialAltitudeThreshold`).
  - If it has, and then descends below `landingAltitudeThreshold` while exceeding `landingAccMagThreshold`, it sets `landedState` to `true`.

- **`updateServo()`**  
  - Based on `landedState`, outputs PWM signals to rotate the servo (and toggles a latch pin).

### Thresholds and States

- **`landedState`**  
  A boolean that becomes `true` when the payload is considered landed.
- **`initialAltitudeAchieved`**  
  Indicates if the payload has surpassed the `initialAltitudeThreshold`.
- **`landingAccMagThreshold`**  
  Minimum acceleration magnitude required to consider a landing impact.
- **`landingAltitudeThreshold`**  
  Altitude threshold below which we consider landing (if acceleration threshold is also met).
- **`initialAltitudeThreshold`**  
  Altitude threshold to confirm the payload has taken off.

---

## Configuration

- **Altitude Calibration**  
  Modify `GroundLevel` to match the local ground altitude your IMU calculates. This helps ensure accurate altitude thresholds.
  
- **Logging Rate**  
  The logging timer is configured in microseconds (e.g., `logTimer.begin(logData, 10000);` for ~100 Hz). Adjust as needed.

- **Servo Pulse Width**  
  The `delayMicroseconds()` calls in `updateServo()` determine the servo angle. Tune these values according to your specific servo’s requirements.

---

## License
This project is provided as-is for educational and experimental purposes. Feel free to adapt and modify the code for your own use. If you share or distribute it, please give appropriate credit.
---

**Happy Logging and Safe Landings!**  
Feel free to open issues or pull requests if you encounter any bugs or have suggestions for improvements. Note: Main Contribution: TruuEE (Commit 13cb9e5).
