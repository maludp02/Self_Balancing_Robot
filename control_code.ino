#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "Wire.h"

// ================================================================
// 1. HARDWARE PINOUT (#define)
// ================================================================

#define MOTOR_LEFT_IN1  7
#define MOTOR_LEFT_IN2  6
#define MOTOR_RIGHT_IN3 5
#define MOTOR_RIGHT_IN4 4
#define MPU_INT_PIN     2

// ================================================================
// 2. PID CONTROL VARIABLES
// ================================================================

// Tuning Parameters (Gains)
double pid_kp = 1.15;
double pid_ki = 0.0015;
double pid_kd = 0.002;

// Configuration and Limits
double pid_setPoint = 0.0;
double pid_limitIntegralMax = 20.0;  // Anti-windup max
double pid_limitIntegralMin = -20.0; // Anti-windup min
double pid_maxOutput = 255.0;        // PWM limit

// System State (Memory)
double pid_error = 0;
double pid_lastError = 0;
double pid_integral = 0;
double pid_derivative = 0;
double pid_output = 0;

// Timing
unsigned long pid_lastProcessTime = 0;

// ================================================================
// 3. MPU6050 SENSOR VARIABLES
// ================================================================

MPU6050 mpu;

// Filter Configuration
const float FILTER_ALPHA = 0.7;   
const float MAX_ANGLE_CHANGE = 30.0;

// MPU Control/Status
bool dmpReady = false;  
uint8_t mpuIntStatus;   
uint8_t devStatus;     
uint16_t packetSize;    
uint16_t fifoCount;    
uint8_t fifoBuffer[64]; 

// Orientation/Motion
Quaternion q;           // [w, x, y, z]   
VectorFloat gravity;    // [x, y, z]            
float ypr[3];           // [yaw, pitch, roll]   

// Processed Angle
float currentAngle = 0.0;
float lastValidAngle = 0.0;
unsigned int outlierCount = 0;

// Interrupt Flag
volatile bool mpuInterrupt = false;

// ================================================================
// INTERRUPT ROUTINE
// ================================================================

void dmpDataReady() 
{
    mpuInterrupt = true;
}

// ================================================================
// FUNCTION PROTOTYPES
// ================================================================

void initMPU();
void controlMotors(double output);

// ================================================================
// SETUP
// ================================================================

void setup() 
{
    // 1. Initialize Communication
    Wire.begin();
    Wire.setClock(400000); 
    Serial.begin(115200);

    // 2. Initialize Motor Pins
    pinMode(MOTOR_LEFT_IN1, OUTPUT);
    pinMode(MOTOR_LEFT_IN2, OUTPUT);
    pinMode(MOTOR_RIGHT_IN3, OUTPUT);
    pinMode(MOTOR_RIGHT_IN4, OUTPUT);

    // 3. Initialize MPU6050
    initMPU();

    // 4. Initialize PID Timer
    pid_lastProcessTime = millis();
}

// ================================================================
// MAIN LOOP
// ================================================================

void loop() 
{
    // If MPU failed or no new data, return
    if (!dmpReady || !mpuInterrupt) return;

    // Reset interrupt flag
    mpuInterrupt = false;

    // --- MPU READ SEQUENCE ---
  
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    // Check for FIFO Overflow
  
    if ((mpuIntStatus & 0x10) || fifoCount == 1024) 
    {
        mpu.resetFIFO();
        Serial.println(F("FIFO Overflow!"));
    }
      
    // Check for DMP Data Ready
    else if (mpuIntStatus & 0x02) 
    {
        // Wait for correct packet size
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

        // Read packet
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        fifoCount -= packetSize;

        // Extract Data
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

        // Convert Radians to Degrees
        float rawAngle = ypr[1] * 180 / M_PI; 

        // --- OUTLIER FILTERING ---
        float deltaAngle = abs(rawAngle - lastValidAngle);

        if (deltaAngle < MAX_ANGLE_CHANGE || outlierCount > 5) 
        {
            // Valid reading
            lastValidAngle = rawAngle;
            outlierCount = 0;
            
            // Low-Pass Filter
            currentAngle = (FILTER_ALPHA * currentAngle) + ((1.0 - FILTER_ALPHA) * rawAngle);
        } else 
        {
            // Invalid reading (noise spike)
            outlierCount++;
        }

        // --- PID CALCULATION ---
        unsigned long now = millis();
        double dt = (now - pid_lastProcessTime) / 1000.0; // Seconds

        // Safety check for dt
        if(dt <= 0 || dt > 0.5) dt = 0.01; 
        pid_lastProcessTime = now;

        // 1. Error
        pid_error = pid_setPoint - currentAngle;

        // 2. Integral (with Anti-Windup)
        pid_integral += (pid_ki * pid_error * dt);
        
        if (pid_integral > pid_limitIntegralMax) pid_integral = pid_limitIntegralMax;
        if (pid_integral < pid_limitIntegralMin) pid_integral = pid_limitIntegralMin;

        // 3. Derivative
        pid_derivative = (pid_error - pid_lastError) / dt;

        // 4. Output
        double P = pid_kp * pid_error;
        double I = pid_integral;
        double D = pid_kd * pid_derivative;

        pid_output = P + I + D;
        pid_lastError = pid_error;

        // --- MOTOR CONTROL ---
        controlMotors(pid_output);
    }
}

// ================================================================
// HELPER FUNCTIONS
// ================================================================

void initMPU() 
{
    Serial.println(F("Initializing I2C..."));
    mpu.initialize();
    
    Serial.println(F("Testing MPU connection..."));
  
    if(!mpu.testConnection()) 
    {
        Serial.println(F("MPU6050 connection failed!"));
        while(1);
    }

    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    // --------------------------------------------------------
    // IMPORTANT: INSERT YOUR CALIBRATED OFFSETS HERE
    // --------------------------------------------------------
  
    mpu.setXAccelOffset(-338);
    mpu.setYAccelOffset(-479);
    mpu.setZAccelOffset(1462);
    mpu.setXGyroOffset(131);
    mpu.setYGyroOffset(6);
    mpu.setZGyroOffset(-18);
  
    // --------------------------------------------------------

    if (devStatus == 0) 
    {
        mpu.setDMPEnabled(true);
        attachInterrupt(digitalPinToInterrupt(MPU_INT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
        Serial.println(F("DMP Ready!"));
    } else 
    {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
}

void controlMotors(double output) 
{
    // Limit PWM
    if (output > pid_maxOutput) output = pid_maxOutput;
    if (output < -pid_maxOutput) output = -pid_maxOutput;

    int pwmVal = abs((int)output);

    // Deadzone check (optional)
    // if (pwmVal < 30 && pwmVal > 0) pwmVal = 30;

    if (output > 0)
    {
        // Forward
        analogWrite(MOTOR_LEFT_IN1, pwmVal);
        analogWrite(MOTOR_LEFT_IN2, 0);
        analogWrite(MOTOR_RIGHT_IN3, pwmVal);
        analogWrite(MOTOR_RIGHT_IN4, 0);
    } else if (output < 0) 
    {
        // Backward
        analogWrite(MOTOR_LEFT_IN1, 0);
        analogWrite(MOTOR_LEFT_IN2, pwmVal);
        analogWrite(MOTOR_RIGHT_IN3, 0);
        analogWrite(MOTOR_RIGHT_IN4, pwmVal);
    } else 
    {
        // Stop
        digitalWrite(MOTOR_LEFT_IN1, LOW);
        digitalWrite(MOTOR_LEFT_IN2, LOW);
        digitalWrite(MOTOR_RIGHT_IN3, LOW);
        digitalWrite(MOTOR_RIGHT_IN4, LOW);
    }
}
