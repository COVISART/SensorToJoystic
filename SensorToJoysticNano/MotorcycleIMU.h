// MotorcycleIMU.h

#pragma once

#include "MPU6050.h"

class MotorcycleIMU {
public:
    MotorcycleIMU(uint8_t mpuAddress = 0x68);

    void begin(const char* msg = "");
    void update(float dt);
    void setWire(TwoWire* wire);
    float getAngleX(){ return angleX;};
    float getAngleY(){ return angleY;};
    float getAngleZ(){ return angleZ;};
    float getPitch(){ return aPitch;};
    float getRoll(){ return aRoll;};

private:
    uint8_t MPU_ADDRESS;
    double gyroOffsetX, gyroOffsetY, gyroOffsetZ;
    double accelOffsetX, accelOffsetY, accelOffsetZ;
    float angleX, angleY, angleZ;
    float aPitch, aRoll;
    int mappedX;
};
