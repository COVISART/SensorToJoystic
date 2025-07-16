// MotorcycleIMU.cpp

#include "MotorcycleIMU.h"

MotorcycleIMU::MotorcycleIMU(uint8_t mpuAddress) 
    : MPU_ADDRESS(mpuAddress), lastTime(0) {
}

void MotorcycleIMU::begin(const char* msg = "") {
    
    wakeSensor(MPU_ADDRESS);
    delay(100);
    calculateGyroOffset(MPU_ADDRESS, 3000, gyroOffsetX, gyroOffsetY, gyroOffsetZ, msg);
    calculateAccelOffset(MPU_ADDRESS, 3000, accelOffsetX, accelOffsetY, msg);
    kalmanX.setMeasure(kalmanFilterIntensity);
    kalmanY.setMeasure(kalmanFilterIntensity);

    lastTime = millis();
}
void MotorcycleIMU::setWire(TwoWire* wire) {
    ::setWire(wire);
}
void MotorcycleIMU::setKalmanFilterIntensity(float intensity){
    kalmanFilterIntensity = intensity;
    kalmanX.setMeasure(kalmanFilterIntensity);
    kalmanY.setMeasure(kalmanFilterIntensity);
}
void MotorcycleIMU::update() {
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;

    int error = readGyroData(MPU_ADDRESS, rawGX, rawGY, rawGZ);
    if (error != 0) {
        Serial.print("Gyro data read error: "); Serial.println(error);
        return;
    }
    rawGyroToDPS(rawGX, rawGY, rawGZ, dpsGX, dpsGY, dpsGZ);

    error = readAccelData(MPU_ADDRESS, rawAX, rawAY, rawAZ);
    if (error != 0) {
        Serial.print("Accel data read error: "); Serial.println(error);
        return;
    }
    rawAccelToGForce(rawAX, rawAY, rawAZ, gForceAX, gForceAY, gForceAZ);
    calculateAnglesFromAccel(gForceAX, gForceAY, gForceAZ, aPitch, aRoll);

    dpsGX -= gyroOffsetX;
    dpsGY -= gyroOffsetY;
    dpsGZ -= gyroOffsetZ;

    aPitch -= accelOffsetX;
    aRoll  -= accelOffsetY;

    // Tamamlayıcı filtre ile açı hesaplama (örnek alpha = 0.98)
    float alpha = 0.995;
    angleX += dpsGX * dt;
    angleY += dpsGY * dt;
    angleZ += dpsGZ * dt;
    //angleX = alpha * (angleX + dpsGX * dt) + (1 - alpha) * aPitch;
    //angleY = alpha * (angleY + dpsGY * dt) + (1 - alpha) * aRoll;

    //angleX = kalmanX.getAngle(aPitch, dpsGX, dt);
    //angleY = kalmanY.getAngle(aRoll, dpsGY, dt);
}
