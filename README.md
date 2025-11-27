# 👾 SELF BALANCING ROBOT - ACADEMIC PROJECT

**Developed by:**
- Maria Luisa
- Ana carolina

## 📄 Project Overview 

This project was developed as a practical application for the **Control Systems II** course. It implements a classic **Inverted Pendulum** system, a fundamental benchmark in control theory to demonstrate the stability of inherently unstable systems.

The main objective is to keep a two-wheeled robot upright using a closed-loop control system. By reading the tilt angle from the MPU6050 sensor (using sensor fusion), the Arduino Mega calculates the necessary correction via a **PID (Proportional-Integral-Derivative) algorithm** and adjusts the DC motors' speed in real-time to maintain vertical equilibrium.

##  🔑 KEY FEATURES 
The core logic of this code uses:

1.  **MPU6050 DMP**: Angle calculation is performed inside the sensor chip.
2.  **Hardware Interrupts**: Data reading is synchronized with the sensor using INT pin.
3.  **FIFO Management**: Prevents buffer overflow to improve the robot's reaction to the most recent data.
4.  **Outlier Filter**: Rejects spurious readings or excessive sensor noise.
5.  **I2C Watchdog**: Prevents microcontroller freezes if the sensor connection fails momentarily.

## ⚙️ Required Hardware 

- **Microcontroller**: Arduino Mega 2560
- **Sensor**: MPU6050 (Accelerometer + Gyroscope)
- **Motor Driver**: L298N H-Bridge or similar
- **Motors**: 2x DC Motors with Gearbox
- **Power Supply**: Li-Ion Battery (e.g., 2x 18650 or 3S LiPo)

## 🔌 Connections (Arduino Mega) 

### MPU6050
| MPU6050 Pin | Arduino Mega Pin | Note |
|-------------|------------------|------|
| VCC         | 5V               |      |
| GND         | GND              |      |
| **SCL** | **21** | I2C Communication |
| **SDA** | **20** | I2C Communication |
| **INT** | **2** | **Mandatory** for interrupt |

### L298N Driver
| L298N Pin | Arduino Mega Pin | Function |
|-----------|------------------|----------|
| IN1       | 7                | Left Motor PWM |
| IN2       | 6                | Left Motor PWM |
| IN3       | 5                | Right Motor PWM |
| IN4       | 4                | Right Motor PWM |
| ENA/ENB   | 5V (Jumper)      | Enable Motors |


## 📍Calibration

To achieve stability, the MPU6050 must be calibrated to find its "natural zero" offsets.

### Calibration vs. Control Code
It is important to distinguish between the two steps:

1.  **Calibration Code (Utility):** This is a separate sketch run **once**. It reads the sensor while it is perfectly still and flat to calculate the hardware error (offsets).

2.  **Control Code (Main Project):** This is the robot's logic. It takes the offsets found in step 1 and hardcodes them into the `setup()` to correct the sensor readings in real-time.

### Our Calibration Results
After running the calibration utility on our specific hardware, we obtained the following offsets which are implemented in the main code:

```cpp
mpu.setXAccelOffset(-338);
mpu.setYAccelOffset(-479);
mpu.setZAccelOffset(1462);
mpu.setXGyroOffset(131);
mpu.setYGyroOffset(6);
mpu.setZGyroOffset(-18);
```

## Configuration and Tuning (PID)

The PID parameters are manually defined in the code and may require tuning depending on the weight and center of gravity of your robot:

```cpp
double kp = 1.15; //  Proportional (Reaction force)
double ki = 0.0015; // Integral (Continuous error correction)
double kd = 0.002; // Derivation (Oscillation dampening)
```

**These parameters are preliminary and under adjustment**