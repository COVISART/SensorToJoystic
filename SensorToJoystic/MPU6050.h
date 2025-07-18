#ifndef MPU6050_H
#define MPU6050_H

#include <Arduino.h>
#include <SoftwareWire.h>

class MPU6050 {
public:
    // Constructor
    MPU6050(SoftwareWire* wire = nullptr, uint8_t address = 0x68);

    // Set the I2C wire
    void setWire(SoftwareWire* wire);

    // Wake sensor from sleep mode
    int begin(int id = 0);

    // Read sensor data
    int readGyroData(float &gX, float &gY, float &gZ);
    int readAccelData(float &aX, float &aY, float &aZ);
    int readTempData(float &temp);

    // Convert raw data to meaningful units
    int rawGyroToDPS(float rawGX, float rawGY, float rawGZ, float &dpsGX, float &dpsGY, float &dpsGZ);
    int rawAccelToGForce(float rawAX, float rawAY, float rawAZ, float &gForceAX, float &gForceAY, float &gForceAZ);

    // Calculate angles and offsets
    int dpsToAngles(float dpsGX, float dpsGY, float dpsGZ, float &actGX, float &actGY, float &actGZ);
    int calculateGyroOffset(uint16_t sampleCount, double &gyroOffsetX, double &gyroOffsetY, double &gyroOffsetZ, int id);
    int calculateAnglesFromAccel(float aX, float aY, float aZ, float &pitch, float &roll);
    int calculateAccelOffset(uint16_t sampleCount, double &accelOffsetX, double &accelOffsetY, int id);

    // Apply complementary filter
    int complementaryFilter(float dpsGyro, float accelAngle, float alpha, float deltaTime, float &filteredAngle);

    void update(float &pitch, float &roll, float &yaw);
    void setComplementaryFilterAlpha(float alpha);
private:
    SoftwareWire* wire;
    uint8_t address;
    float complementaryFilterAlpha = 0.1;
    // MPU6050 register addresses
    enum MPU6050_Registers : uint8_t {
        PWR_MGMT_1  = 0x6B,
        GYRO_XOUT_H = 0x43,
        ACCEL_XOUT_H = 0x3B,
        TEMP_OUT_H = 0x41
    };

    // I2C error codes
    enum I2C_Errors : uint8_t {
        DATA_TOO_LONG_FOR_TRANSMIT_BUFFER = 1,
        ADDRESS_TRANSMIT_NACK = 2,
        DATA_TRANSMIT_NACK = 3,
        OTHER_ERROR = 4,
        TIMEOUT = 5
    };

    // Internal timestamp for angle calculations
    unsigned long previousTime;

    // Calibrate gyroscope and accelerometer for both sensors
    double gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    double accelOffsetX, accelOffsetY;
};

#endif