// MotorcycleIMU.cpp

#include "MotorcycleIMU.h"

MotorcycleIMU::MotorcycleIMU(uint8_t mpuAddress) 
    : MPU_ADDRESS(mpuAddress), lastTime(0), preInterval(0) {
}

void MotorcycleIMU::begin(const char* msg = "") {
    
    wakeSensor(MPU_ADDRESS);
    delay(1000);
    calculateGyroOffset(MPU_ADDRESS, gyroOffsetX, gyroOffsetY, gyroOffsetZ, msg);
    calculateAccelOffset(MPU_ADDRESS, accelOffsetX, accelOffsetY, "Brake");

    kalmanX.setMeasure(kalmanFilterIntensity);
    kalmanY.setMeasure(kalmanFilterIntensity);

    lastTime = millis();
}

void MotorcycleIMU::setKalmanFilterIntensity(float intensity){
  kalmanFilterIntensity = intensity;
}
void MotorcycleIMU::update() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    readGyroData(MPU_ADDRESS, rawGX, rawGY, rawGZ);
    rawGyroToDPS(rawGX, rawGY, rawGZ, dpsGX, dpsGY, dpsGZ);

    readAccelData(MPU_ADDRESS, rawAX, rawAY, rawAZ);
    rawAccelToGForce(rawAX, rawAY, rawAZ, gForceAX, gForceAY, gForceAZ);
    calculateAnglesFromAccel(gForceAX, gForceAY, gForceAZ, aPitch, aRoll);

    dpsGX -= gyroOffsetX;
    dpsGY -= gyroOffsetY;
    dpsGZ -= gyroOffsetZ;

    aPitch -= accelOffsetX;
    aRoll  -= accelOffsetY;

    interval = (millis() - preInterval) * 0.001;
    angleX = kalmanX.getAngle(aPitch, dpsGX, interval);
    angleY = kalmanY.getAngle(aRoll, dpsGY, interval);
    preInterval = millis();

}
