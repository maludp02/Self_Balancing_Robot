# 👾 SELF BALANCING ROBOT - ACADEMIC PROJECT

![Status](https://img.shields.io/badge/Status-Completed-success)
![Language](https://img.shields.io/badge/Language-C%2B%2B%20%2F%20Arduino-blue)

**Developed by:**
* **Maria Luisa**
* **Ana Carolina**

## 📄 Project Overview

This project was developed as a practical application for the **Control Systems II** course. It implements a classic **Inverted Pendulum** system, a fundamental benchmark in control theory to demonstrate the stability of inherently unstable systems.

The main objective is to keep a two-wheeled robot upright using a closed-loop control system. Unlike libraries that rely on the MPU6050's internal DMP (Digital Motion Processor), this project implements **raw physics calculations** and **sensor fusion** manually on the Arduino, providing a deeper insight into the mathematics of control theory.

## 🔑 Key Features

The core logic of this code differentiates itself by using:

* **Raw I2C Implementation:** Direct communication with the MPU6050 registers (`0x3B` and `0x43`) without relying on heavy external libraries, optimizing execution speed.
* **Complementary Filter:** Implements a manual sensor fusion algorithm ($0.98 \times Gyro + 0.02 \times Accel$) to combine the stability of the accelerometer with the speed of the gyroscope, filtering out vibration and drift.
* **Custom PID Controller:** Features a Proportional-Integral-Derivative controller with **Integral Windup Protection** (clamping the integral error when the robot falls or stalls).
* **Deadzone Compensation:** A `min_pwm` threshold (set to 64) ensures the motors overcome internal gearbox friction and start moving immediately when a correction is needed.
* **Teleplot Ready:** The serial output is formatted specifically for the **Teleplot** extension (VS Code), allowing real-time plotting of angles and PID errors (`>var:value`).

## ⚙️ Required Hardware

* **Microcontroller:** Arduino Uno or Mega 2560
* **Sensor:** MPU6050 (Accelerometer + Gyroscope)
* **Motor Driver:** H-Bridge (L298N or similar compatible with PWM)
* **Motors:** 2x DC Motors with Gearbox
* **Power Supply:** Li-Ion Battery (e.g., 2x 18650 or 3S LiPo)

## 🔌 Connections

### MPU6050 (I2C)

| MPU6050 Pin | Arduino Pin | Note |
| :--- | :--- | :--- |
| VCC | 5V | |
| GND | GND | |
| SCL | SCL (Pin 21 on Mega) | I2C Clock |
| SDA | SDA (Pin 20 on Mega) | I2C Data |
| INT | Not Used | Polling method used |

### Motor Driver (H-Bridge)

| Driver Pin | Arduino Pin | Function |
| :--- | :--- | :--- |
| IN1 | 7 | Left Motor PWM A |
| IN2 | 6 | Left Motor PWM B |
| IN3 | 5 | Right Motor PWM A |
| IN4 | 4 | Right Motor PWM B |

*> Note: The code logic handles direction by switching which pin receives PWM and which is set to LOW.*

## 📍 Calibration & Setup

The code uses hardcoded `constexpr` offsets derived from testing. The initialization sequence in `setup()` performs an initial accelerometer reading to set the starting angle before the loop begins.

### Current Calibration Offsets
These values are specific to our hardware implementation (found in the global constants of the code). You should recalibrate if you use different hardware.

```cpp
constexpr int offset_accelX = -338;
constexpr int offset_accelY = -479;
constexpr int offset_accelZ = 1462;
constexpr int offset_gyroX = 131;
