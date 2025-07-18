#include "MPU6050.h"
#include <stdint.h>

MPU6050::MPU6050(SoftwareWire* wire, uint8_t address) 
: wire(wire), address(address), previousTime(millis()) {}

void MPU6050::setWire(SoftwareWire* wire) {
    wire = wire;
}

int MPU6050::begin(int id = 0) {
    wire->beginTransmission(address);
    wire->write(PWR_MGMT_1);
    wire->write(0x00);
    uint8_t error = wire->endTransmission(0);
    if (error != 0) {
        Serial.print("wakeSensor hata, adres: 0x");
        Serial.print(address, HEX);
        Serial.print(" Kod: ");
        Serial.println(error);
    }
    switch(error) {
        case 0: break;
        case 1: return DATA_TOO_LONG_FOR_TRANSMIT_BUFFER;
        case 2: return ADDRESS_TRANSMIT_NACK;
        case 3: return DATA_TRANSMIT_NACK;
        case 5: return TIMEOUT;
        default: return OTHER_ERROR;
    }
    calculateGyroOffset(3000, gyroOffsetX, gyroOffsetY, gyroOffsetZ, id);
    calculateAccelOffset(3000, accelOffsetX, accelOffsetY, id);
    Serial.println("Setup complete for both sensors!");
    return 0;
}

int MPU6050::readGyroData(float &gX, float &gY, float &gZ) {
    wire->beginTransmission(address);
    wire->write(GYRO_XOUT_H);
    uint8_t error = wire->endTransmission(0);
    switch(error) {
        case 0: break;
        case 1: return DATA_TOO_LONG_FOR_TRANSMIT_BUFFER;
        case 2: return ADDRESS_TRANSMIT_NACK;
        case 3: return DATA_TRANSMIT_NACK;
        case 5: return TIMEOUT;
        default: return OTHER_ERROR;
    }
    wire->requestFrom(address, (uint8_t)6, (uint8_t)1);
    if (wire->available() < 6) {
        return DATA_TRANSMIT_NACK;
    }
    gX = wire->read() << 8 | wire->read();
    gY = wire->read() << 8 | wire->read();
    gZ = wire->read() << 8 | wire->read();
    return 0;
}

int MPU6050::readAccelData(float &aX, float &aY, float &aZ) {
    wire->beginTransmission(address);
    wire->write(ACCEL_XOUT_H);
    uint8_t error = wire->endTransmission(0);
    switch(error) {
        case 0: break;
        case 1: return DATA_TOO_LONG_FOR_TRANSMIT_BUFFER;
        case 2: return ADDRESS_TRANSMIT_NACK;
        case 3: return DATA_TRANSMIT_NACK;
        case 5: return TIMEOUT;
        default: return OTHER_ERROR;
    }
    wire->requestFrom(address, (uint8_t)6, (uint8_t)1);
    if (wire->available() < 6) {
        return DATA_TRANSMIT_NACK;
    }
    aX = wire->read() << 8 | wire->read();
    aY = wire->read() << 8 | wire->read();
    aZ = wire->read() << 8 | wire->read();
    return 0;
}

int MPU6050::readTempData(float &temp) {
    wire->beginTransmission(address);
    wire->write(TEMP_OUT_H);
    uint8_t error = wire->endTransmission(0);
    switch(error) {
        case 0: break;
        case 1: return DATA_TOO_LONG_FOR_TRANSMIT_BUFFER;
        case 2: return ADDRESS_TRANSMIT_NACK;
        case 3: return DATA_TRANSMIT_NACK;
        case 5: return TIMEOUT;
        default: return OTHER_ERROR;
    }
    wire->requestFrom(address, (uint8_t)2, (uint8_t)1);
    if (wire->available() < 2) {
        return DATA_TRANSMIT_NACK;
    }
    float rawTemp = wire->read() << 8 | wire->read();
    temp = (rawTemp / 340.0) + 36.53;
    return 0;
}

int MPU6050::rawGyroToDPS(float rawGX, float rawGY, float rawGZ, float &dpsGX, float &dpsGY, float &dpsGZ) {
    dpsGX = rawGX / 131.0;
    dpsGY = rawGY / 131.0;
    dpsGZ = rawGZ / 131.0;
    return 0;
}

int MPU6050::rawAccelToGForce(float rawAX, float rawAY, float rawAZ, float &gForceAX, float &gForceAY, float &gForceAZ) {
    gForceAX = rawAX / 16384.0;
    gForceAY = rawAY / 16384.0;
    gForceAZ = rawAZ / 16384.0;
    return 0;
}

int MPU6050::dpsToAngles(float dpsGX, float dpsGY, float dpsGZ, float &actGX, float &actGY, float &actGZ) {
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;
    actGX += dpsGX * deltaTime;
    actGY += dpsGY * deltaTime;
    actGZ += dpsGZ * deltaTime;
    return 0;
}

int MPU6050::calculateGyroOffset(uint16_t sampleCount, double &gyroOffsetX, double &gyroOffsetY, double &gyroOffsetZ, int id) {
    float gX, gY, gZ;
    float dpsGX, dpsGY, dpsGZ;
    gyroOffsetX = 0;
    gyroOffsetY = 0;
    gyroOffsetZ = 0;
    Serial.println("\n========================================");
    Serial.print("Calculating gyro offsets for: ");
    Serial.println(id);
    Serial.print("DO NOT MOVE MPU6050");
    for (int i = 0; i < sampleCount; i++) {
        if (i % 100 == 0) {
            Serial.print(".");
        }
        readGyroData(gX, gY, gZ);
        rawGyroToDPS(gX, gY, gZ, dpsGX, dpsGY, dpsGZ);
        gyroOffsetX += dpsGX;
        gyroOffsetY += dpsGY;
        gyroOffsetZ += dpsGZ;
    }
    gyroOffsetX /= sampleCount;
    gyroOffsetY /= sampleCount;
    gyroOffsetZ /= sampleCount;
    Serial.println("\nDone!");
    Serial.print("X : "); Serial.println(gyroOffsetX);
    Serial.print("Y : "); Serial.println(gyroOffsetY);
    Serial.print("Z : "); Serial.println(gyroOffsetZ);
    Serial.println("Program will start .....");
    Serial.println("========================================");
    return 0;
}

int MPU6050::calculateAnglesFromAccel(float aX, float aY, float aZ, float &pitch, float &roll) {
    pitch = atan2(-aX, sqrt(pow(aY, 2) + pow(aZ, 2))) * RAD_TO_DEG;
    roll = atan2(aY, aZ) * RAD_TO_DEG;
    return 0;
}

int MPU6050::calculateAccelOffset(uint16_t sampleCount, double &accelOffsetX, double &accelOffsetY, int id) {
    float aX, aY, aZ;
    float gForceAX, gForceAY, gForceAZ;
    float aPitch, aRoll;
    accelOffsetX = 0;
    accelOffsetY = 0;
    Serial.println("\n========================================");
    Serial.print("Calculating accel offsets for: ");
    Serial.println(id);
    Serial.print("DO NOT MOVE MPU6050");
    for (int i = 0; i < sampleCount; i++) {
        if (i % 100 == 0) {
            Serial.print(".");
        }
        readAccelData(aX, aY, aZ);
        rawAccelToGForce(aX, aY, aZ, gForceAX, gForceAY, gForceAZ);
        calculateAnglesFromAccel(gForceAX, gForceAY, gForceAZ, aPitch, aRoll);
        accelOffsetX += aPitch;
        accelOffsetY += aRoll;
    }
    accelOffsetX /= sampleCount;
    accelOffsetY /= sampleCount;
    Serial.println("\nDone!");
    Serial.print("X : "); Serial.println(accelOffsetX);
    Serial.print("Y : "); Serial.println(accelOffsetY);
    Serial.println("Program will start .....");
    Serial.println("========================================");
    return 0;
}

int MPU6050::complementaryFilter(float dpsGyro, float accelAngle, float alpha, float deltaTime, float &filteredAngle) {
    filteredAngle = alpha * (filteredAngle + dpsGyro * deltaTime) + (1 - alpha) * accelAngle;
    return 0;
}

void MPU6050::setComplementaryFilterAlpha(float alpha){
    complementaryFilterAlpha = alpha;
}
void MPU6050::update(float &pitch, float &roll, float &yaw) {
    float gX, gY, gZ; // Raw gyro data
    float aX, aY, aZ; // Raw accel data
    float dpsGX, dpsGY, dpsGZ; // Gyro in degrees per second
    float gForceAX, gForceAY, gForceAZ; // Accel in g-force
    float filteredPitch = 0, filteredRoll = 0; // Filtered angle using complementary filter
    float temp; // Temperature

    // Read sensor data
    if (readGyroData(gX, gY, gZ) == 0 && 
        readAccelData(aX, aY, aZ) == 0 && 
        readTempData(temp) == 0) {

        // Convert raw data to meaningful units
        rawGyroToDPS(gX, gY, gZ, dpsGX, dpsGY, dpsGZ);
        rawAccelToGForce(aX, aY, aZ, gForceAX, gForceAY, gForceAZ);

        // Apply gyroscope offsets
        dpsGX -= gyroOffsetX;
        dpsGY -= gyroOffsetY;
        dpsGZ -= gyroOffsetZ;

        // Calculate pitch and roll from accelerometer
        calculateAnglesFromAccel(gForceAX, gForceAY, gForceAZ, pitch, roll);

        // Apply accelerometer offsets
        pitch -= accelOffsetX;
        roll -= accelOffsetY;

        // Apply complementary filter to pitch and roll
        unsigned long currentTime = millis();
        float timeDelta = (currentTime - previousTime) / 1000.0;
        previousTime = currentTime;
        complementaryFilter(dpsGX, pitch, complementaryFilterAlpha, timeDelta, filteredPitch);
        complementaryFilter(dpsGY, roll, complementaryFilterAlpha, timeDelta, filteredRoll);

        // Update yaw using gyroscope data
        yaw += dpsGZ * timeDelta; // Integrate gyroscope Z-axis for yaw

        // Set final angles
        pitch = filteredPitch;
        roll = filteredRoll;

        // Output data (commented out for production)
        /*Serial.print("Temp: "); Serial.print(temp); Serial.print(" °C, ");
        Serial.print("Pitch: "); Serial.print(pitch); Serial.print(" deg, ");
        Serial.print("Filtered Pitch: "); Serial.print(filteredPitch); Serial.print(" deg, ");
        Serial.print("Roll: "); Serial.print(roll); Serial.print(" deg, ");
        Serial.print("Yaw: "); Serial.print(yaw); Serial.println(" deg");*/
    } 
    else {
        Serial.println("Error reading sensor data!");
    }
}