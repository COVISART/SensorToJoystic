// MotorcycleIMU.h

#pragma once

#include <KalmanFilter.h>

#include <mpu6050.h>

class MotorcycleIMU {
public:
    MotorcycleIMU(uint8_t mpuAddress = 0x68);

    void begin(const char* msg = "");
    void update();
    float getAngleX(){ return angleX;};
    float getAngleY(){ return angleY;};
    void setKalmanFilterIntensity(float intensity);

private:
    uint8_t MPU_ADDRESS;
    
    float rawGX, rawGY, rawGZ;
    float dpsGX, dpsGY, dpsGZ;
    float rawAX, rawAY, rawAZ;
    float gForceAX, gForceAY, gForceAZ;
    float aPitch, aRoll;
    double gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    double accelOffsetX, accelOffsetY, accelOffsetZ;

    float interval;
    long preInterval;
    float angleX, angleY;
    KalmanFilter kalmanX, kalmanY;

    int mappedX;

    float kalmanFilterIntensity = 0.5f;
    unsigned long lastTime;
};
