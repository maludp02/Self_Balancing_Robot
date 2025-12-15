#include <Arduino.h>
#include <Wire.h>

#define MPU 0X68

// --- PINOUTS  ---
#define MOTOR_LEFT_IN1 7  
#define MOTOR_LEFT_IN2 6  
#define MOTOR_RIGHT_IN3 5  
#define MOTOR_RIGHT_IN4 4

// --- CONSTANTS ---

// Calibration Offsets
constexpr int offset_accelX = -338;
constexpr int offset_accelY = -479;
constexpr int offset_accelZ = 1462;
constexpr int offset_gyroX = 131;

// Control Variables
constexpr uint8_t min_pwm = 64;
constexpr int16_t max_pwm = 245;

//PID Tuning
constexpr float Kp = 10;
constexpr float Ki = 1;
constexpr float Kd = 0.1; 

// PID Setpoint
constexpr float setpoint = 0.0;

// --- GLOBAL VARIABLES m--
// Sensor Variables (Raw Data)
int16_t raw_accelX;
int16_t raw_accelY;
int16_t raw_accelZ;
int16_t raw_gyroX;

// Angles
float accel_angle = 0;
float filtered_angle = 0;

// PID Variables
float proportional_error = 0;
float last_proportional_error = 0;
float integral_error = 0;
float derivative_error = 0;
float pid_output = 0;

// Time Management
unsigned long previous_time = 0;
float delta_time = 0;

// Teleplot Counter
uint8_t teleplot_counter = 0;

// --- FUNCTION PROTOTYPES ---
void initializedMPU();
void accelerometerReadings();
void gyroscopeReadings();
float calculateAccelAngle(float ax, float ay, float az);
void moveMotors(float control_signal);

void setup()
{
    Serial.begin(115200);

    // Initialize PINS
    pinMode(MOTOR_LEFT_IN1, OUTPUT);
    pinMode(MOTOR_LEFT_IN2, OUTPUT);
    pinMode(MOTOR_RIGHT_IN3, OUTPUT);
    pinMode(MOTOR_RIGHT_IN4, OUTPUT);

    moveMotors(0);
    initializedMPU();

    // Reset PID variables for safety
    pid_output = 0;
    integral_error = 0;
    proportional_error = 0;
    last_proportional_error = 0;
    filtered_angle = 0;
    accel_angle = 0;


    // --- INITIAL STABILIZATION ---
    accelerometerReadings();

    float start_accelX = raw_accelX + offset_accelX;
    float start_accelY = raw_accelY + offset_accelY;
    float start_accelZ = raw_accelZ + offset_accelZ;

    filtered_angle = calculateAccelAngle(start_accelX, start_accelY, start_accelZ);

    previous_time = micros();
}

void loop()
{
    // --- 1. SENSOR READINGS ---
    accelerometerReadings();
    gyroscopeReadings();

    // Apply Offsets
    float real_accelX = raw_accelX + offset_accelX;
    float real_accelY = raw_accelY + offset_accelY;
    float real_accelZ = raw_accelZ + offset_accelZ;
    float real_gyroX = raw_gyroX + offset_gyroX;


    // --- 2. ANGLE CALCULATIONS (Complementary Filter) ---
    // Delta Time Calculation
    unsigned long current_time = micros();
    delta_time = (current_time - previous_time) / 1000000.0;
    previous_time = current_time;


    accel_angle = calculateAccelAngle(real_accelX, real_accelY, real_accelZ);
    float gyro_rateX = real_gyroX / 131.0;

    // Ensure delta_time is positive and non-zero
    if (delta_time <= 0) return; 

    // Filtered Angle Update
    filtered_angle = 0.98 * (filtered_angle + gyro_rateX*delta_time ) + 0.01 * accel_angle;

    // --- 3. PID CONTROL ---
    // This line calculates the error between the desired setpoint and the current angle
    // Allowed the roobt have a balance not only at 0 degrees but also at other angles if needed
    proportional_error = filtered_angle - setpoint;


    // Integral with safety check
    // After 40 degrees the robots falls too fast, so we avoid integral wind-up
    if (abs(filtered_angle) < 40) 
    {
        integral_error += (proportional_error + last_proportional_error)* delta_time/ 2.0;
    } else 
    {
        integral_error = 0; 
    }

    // Prevents the accumulated error from increasing indefinitely (Windup) if the robot stalls (stuck)
    integral_error = constrain(integral_error, -max_pwm, max_pwm);

    // Derivative
    //Gyro rate is already the derivative of angle
    derivative_error = gyro_rateX;
    //derivative_error = (proportional_error - last_proportional_error ) / delta_time;

    last_proportional_error = proportional_error;

    // PID Output
    pid_output = (Kp * proportional_error) + (Ki * integral_error) + (Kd*derivative_error);

    // --- 4. MOTOR ACTUATION ---
    // Safety check: If angle too large, stop motors to avoid damage
    if (abs(filtered_angle) > 45) 
    {
        moveMotors(0);
    } else 
    {
        moveMotors(pid_output);
    }

    // --- 5. TELEMETRY ---

    teleplot_counter++;

    if (teleplot_counter >= 40) 
    {
        Serial.print(">filtered angle:"); Serial.println(filtered_angle); 
        Serial.print(">PWM:"); Serial.println(pid_output); 
        // Showing current PID values so you can track them
        Serial.print(">P_error:"); Serial.println(proportional_error);
        Serial.print(">I_error:"); Serial.println(integral_error);
        Serial.print(">D_error:"); Serial.println(derivative_error);
        Serial.print(">DeltaTime:"); Serial.println(delta_time, 6);
        teleplot_counter = 0;
    }

}

// --- FUNCTION DEFINITIONS ---

void initializedMPU() 
{
    Wire.begin();
    Wire.setClock(400000); 
    Wire.setWireTimeout(3000, true);
    Wire.beginTransmission(MPU);
    Wire.write(0x6B); 
    Wire.write(0);     
    Wire.endTransmission(true);
}

void accelerometerReadings()
{
    Wire.beginTransmission(MPU);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 6, true); 

    raw_accelX = Wire.read() << 8 | Wire.read(); 
    raw_accelY = Wire.read() << 8 | Wire.read(); 
    raw_accelZ = Wire.read() << 8 | Wire.read(); 
}

void gyroscopeReadings()
{
    Wire.beginTransmission(MPU);
    Wire.write(0x43); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU, 2, true); 
    raw_gyroX = Wire.read() << 8 | Wire.read(); 
}

float calculateAccelAngle(float accelX, float accelY, float accelZ)
{
    float denominator = sqrt((accelY * accelY) + (accelZ * accelZ));
    float angle_rad = atan2(accelX, denominator);

    return angle_rad * (180.0 / PI);
}

void moveMotors(float control_signal)
{
    uint16_t motor_power = abs(control_signal);

    if (motor_power > 0) motor_power += min_pwm; 


    int pwm_out = constrain(motor_power, 0, max_pwm);

    if (control_signal > 0) 
    {
        analogWrite(MOTOR_LEFT_IN1, 0);
        analogWrite(MOTOR_LEFT_IN2, pwm_out);
        analogWrite(MOTOR_RIGHT_IN3, pwm_out);
        analogWrite(MOTOR_RIGHT_IN4, 0);

    } 
    else if (control_signal < -0) 
    {
        analogWrite(MOTOR_LEFT_IN1, pwm_out);
        analogWrite(MOTOR_LEFT_IN2, 0);
        analogWrite(MOTOR_RIGHT_IN3, 0);
        analogWrite(MOTOR_RIGHT_IN4, pwm_out);
    } 
    else 
    {
        analogWrite(MOTOR_LEFT_IN1, 0);
        analogWrite(MOTOR_LEFT_IN2, 0);
        analogWrite(MOTOR_RIGHT_IN3, 0);
        analogWrite(MOTOR_RIGHT_IN4, 0);
    }
}

