/* MPU6050 Calibration Sketch

    Authors: Maria Luisa and Ana Carolina

    This code is based on the calibration method described by: Luis/Rodenas
    
    Description:
    This sketch calculates the offsets for the MPU6050 sensor to ensure 
    that it reads 0 for all axes (except Z accel which should be 16384) 
    when the robot is perfectly still and vertical.
*/

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

// ------------------------------------------------------
// CONFIGURATION & SETTINGS
// ------------------------------------------------------

MPU6050 mpu;

// Number of readings to calculate the average
const int BUFFER_SIZE = 1000; 

// Deadzone (Acceptable error range)
int acel_deadzone = 8;      // Accelerometer error tolerance
int giro_deadzone = 1;      // Gyroscope error tolerance

// ------------------------------------------------------
// SENSOR DATA VARIABLES
// ------------------------------------------------------

// Raw data storage
int16_t ax, ay, az, gx, gy, gz;

// Calculated averages
long mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz;

// Calculated Offsets
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

// ------------------------------------------------------
// FUNCTION DECLARATIONS
// ------------------------------------------------------

void startCalibration();
void meansensors();
void calibration();

// ------------------------------------------------------
// MAIN SETUP
// ------------------------------------------------------

void setup() 
{
    Wire.begin();
    Wire.setClock(400000); // Fast I2C
    Serial.begin(115200);

    // Initialize MPU
    mpu.initialize();
    
    Serial.println("\n========================================");
    Serial.println("    MPU6050 CALIBRATION SKETCH");       
    Serial.println("========================================");

    // Connection test
    Serial.println("Testing connection...");
    if (mpu.testConnection()) 
    {
        Serial.println("MPU6050 connected successfully!");
    } 
    else 
    {
        Serial.println("ERROR: MPU6050 not found. Check wiring.");
        while (1); // Halt
    }

    // User instructions
    Serial.println("\nIMPORTANT INSTRUCTIONS:");
    Serial.println("1. Keep the robot PERFECTLY STILL.");
    Serial.println("2. Keep the robot in the VERTICAL BALANCE POSITION.");
    Serial.println("3. The sensor must be level (Z axis pointing up).");
    Serial.println("\nSend any character to start...");
    
    // Wait for user input
    while (Serial.available() && Serial.read()); // Clear buffer
    while (!Serial.available());                 // Wait for input
    while (Serial.available() && Serial.read()); // Clear buffer again
    
    Serial.println("\nStarting...");
    
    // Reset offsets to 0 before starting
    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);

    startCalibration();
}

void loop() 
{
    // Calibration is a one-time process in setup
}

// ------------------------------------------------------
// CALIBRATION LOGIC
// ------------------------------------------------------

void startCalibration() 
{
    Serial.println("Reading sensors for the first time...");
    meansensors();
    delay(1000);

    Serial.println("Calculating offsets (this may take a while)...");
    calibration();
    delay(1000);

    Serial.println("\n\n========================================");
    Serial.println("          CALIBRATION FINISHED!         ");
    Serial.println("========================================");
    Serial.println("Copy and paste these lines into the setup() of your robot code:");
    Serial.println("");
    
    Serial.print("mpu.setXAccelOffset("); Serial.print(ax_offset); Serial.println(");");
    Serial.print("mpu.setYAccelOffset("); Serial.print(ay_offset); Serial.println(");");
    Serial.print("mpu.setZAccelOffset("); Serial.print(az_offset); Serial.println(");");
    Serial.print("mpu.setXGyroOffset("); Serial.print(gx_offset); Serial.println(");");
    Serial.print("mpu.setYGyroOffset("); Serial.print(gy_offset); Serial.println(");");
    Serial.print("mpu.setZGyroOffset("); Serial.print(gz_offset); Serial.println(");");
    Serial.println("");
    Serial.println("========================================");
}

// Function to calculate sensor averages (discarding initial readings)
void meansensors() 
{
    long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

    while (i < (BUFFER_SIZE + 101)) 
    {
        // Read raw data
        mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

        // Ignore the first 100 readings (stabilization period)
        if (i > 100 && i <= (BUFFER_SIZE + 100)) 
        {
            buff_ax = buff_ax + ax;
            buff_ay = buff_ay + ay;
            buff_az = buff_az + az;
            buff_gx = buff_gx + gx;
            buff_gy = buff_gy + gy;
            buff_gz = buff_gz + gz;
        }
        
        // Calculate average at the end
        if (i == (BUFFER_SIZE + 100)) 
        {
            mean_ax = buff_ax / BUFFER_SIZE;
            mean_ay = buff_ay / BUFFER_SIZE;
            mean_az = buff_az / BUFFER_SIZE;
            mean_gx = buff_gx / BUFFER_SIZE;
            mean_gy = buff_gy / BUFFER_SIZE;
            mean_gz = buff_gz / BUFFER_SIZE;
        }
        i++;
        delay(2); // Small delay to avoid reading duplicate data from internal buffer
    }
}

// Main convergence loop
void calibration() 
{
    // 1. Initial estimate (Empirical division)
    ax_offset = -mean_ax / 8;
    ay_offset = -mean_ay / 8;
    az_offset = (16384 - mean_az) / 8; // 16384 is 1g in the standard range

    gx_offset = -mean_gx / 4;
    gy_offset = -mean_gy / 4;
    gz_offset = -mean_gz / 4;

    // 2. Fine-tuning loop
    while (1) 
    {
        int ready = 0;
        
        // Apply current offsets
        mpu.setXAccelOffset(ax_offset);
        mpu.setYAccelOffset(ay_offset);
        mpu.setZAccelOffset(az_offset);
        mpu.setXGyroOffset(gx_offset);
        mpu.setYGyroOffset(gy_offset);
        mpu.setZGyroOffset(gz_offset);

        // Read averages again with new offsets
        meansensors();
        
        // Visual feedback
        Serial.print(".");

        // Check Accelerometer X
        if (abs(mean_ax) <= acel_deadzone) ready++;
        else ax_offset = ax_offset - mean_ax / acel_deadzone;

        // Check Accelerometer Y
        if (abs(mean_ay) <= acel_deadzone) ready++;
        else ay_offset = ay_offset - mean_ay / acel_deadzone;

        // Check Accelerometer Z (Target: 16384)
        if (abs(16384 - mean_az) <= acel_deadzone) ready++;
        else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

        // Check Gyroscope X
        if (abs(mean_gx) <= giro_deadzone) ready++;
        else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

        // Check Gyroscope Y
        if (abs(mean_gy) <= giro_deadzone) ready++;
        else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

        // Check Gyroscope Z
        if (abs(mean_gz) <= giro_deadzone) ready++;
        else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

        // If all 6 axes are within the deadzone, break the loop
        if (ready == 6) break;
    }
}