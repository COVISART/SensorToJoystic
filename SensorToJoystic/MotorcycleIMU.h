// MotorcycleIMU.h

#pragma once

#include <KalmanFilter.h>

#include "MPU6050.h"

class MotorcycleIMU {
public:
    MotorcycleIMU(uint8_t mpuAddress = 0x68);

    void begin(const char* msg = "");
    void update();
    void setWire(TwoWire* wire);
    float getAngleX(){ return angleX;};
    float getAngleY(){ return angleY;};
    float getAngleZ(){ return angleZ;};
    float getPitch(){ return aPitch;};
    float getRoll(){ return aRoll;};
    void setKalmanFilterIntensity(float intensity);

private:
    uint8_t MPU_ADDRESS;
    //char buffer[40];
    float rawGX, rawGY, rawGZ;
    float dpsGX, dpsGY, dpsGZ;
    float rawAX, rawAY, rawAZ;
    float gForceAX, gForceAY, gForceAZ;
    float aPitch, aRoll;
    double gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    double accelOffsetX, accelOffsetY, accelOffsetZ;
    float angleX, angleY, angleZ;
    KalmanFilter kalmanX, kalmanY;

    int mappedX;

    float kalmanFilterIntensity = 0.5f;
    unsigned long lastTime;
};
