#include <SoftwareWire.h>
#include "MPU6050.h"
#include <Joystick.h>
#include "Logger.h"

// MPU6050 sensörlerinin I2C adresi
#define SENSOR_ADDRESS 0x68
#define UPDATE_INTERVAL 50
#define SENSOR_COUNT 5

// 5 farklı SoftwareWire hattı için pin tanımlamaları (her biri için SDA ve SCL pinleri)
SoftwareWire Wire1(4, 5);   // Hat 1: SDA = D4, SCL = D5
SoftwareWire Wire2(6, 7);   // Hat 2: SDA = D6, SCL = D7
SoftwareWire Wire3(8, 9);   // Hat 3: SDA = D8, SCL = D9
SoftwareWire Wire4(10, 11); // Hat 4: SDA = D10, SCL = D11
SoftwareWire Wire5(12, 13); // Hat 5: SDA = D12, SCL = D13
SoftwareWire* wires[] = {&Wire1, &Wire2, &Wire3, &Wire4, &Wire5}; // Sensör dizisi

// 5 adet MPU6050 sensörü
MPU6050 mpu1(&Wire1);
MPU6050 mpu2(&Wire2);
MPU6050 mpu3(&Wire3);
MPU6050 mpu4(&Wire4);
MPU6050 mpu5(&Wire5);
MPU6050* mpus[] = {&mpu1, &mpu2, &mpu3, &mpu4, &mpu5}; // Sensör dizisi

Logger logger;

float xAngles[5];
float yAngles[5];
float zAngles[5];

float complementaryFilterAlpha = 0.1;

int steeringValue = 0, frontBrakeValue = 0, rearBrakeValue = 0, leverValue = 0, shifter = 0;

void setup() {

  logger.init(9600);
  while (!Serial);
  logger.setLogLevel(INFO);
  logger.write(INFO, "TCA9548A Test Başladı");

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    mpus[i]->setWire(wires[i]);
    mpus[i]->setComplementaryFilterAlpha(complementaryFilterAlpha);
    if (mpus[i]->begin(i) != 0) {
      Serial.println("Failed to wake MPU6050 1!");
    }
  }
  Joystick.begin();
  Serial.println("Setup complete!");
}

void loop() {
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    mpus[i]->update(xAngles[i], yAngles[i], zAngles[i]);

    Serial.print(" ID_");
    Serial.print(i);
    Serial.print("_X_Angle:");
    Serial.print(xAngles[i]);

    Serial.print(" ID_");
    Serial.print(i);
    Serial.print("_Y_Angle:");
    Serial.print(yAngles[i]);

    Serial.print(" ID_");
    Serial.print(i);
    Serial.print("_Z_Angle:");
    Serial.print(zAngles[i]);
  }
  Serial.println();
  if (xAngles[0] > -25 && xAngles[0] < 25)
    steeringValue   = map(xAngles[0], -25, 25, -127, 127);

  if (xAngles[1]  > 0 && xAngles[1]  < 20)
    frontBrakeValue = map(xAngles[1], 0, 20, -127, 127);

  if (yAngles[2] > 0 && yAngles[2] < 20)
    rearBrakeValue  = map(yAngles[2], 0, 20, -127, 127);

  if (xAngles[3] > 0 && xAngles[3] < 20)
    leverValue      = map(xAngles[3], 0, 20, 0, 360);

  if (zAngles[4] > -20 && zAngles[4] < 20)
    shifter         = map(zAngles[4], -20, 20, 0, 360);

  Joystick.setXAxis(steeringValue);
  Joystick.setYAxis(frontBrakeValue);
  Joystick.setZAxis(rearBrakeValue);
  Joystick.setXAxisRotation(leverValue);
  Joystick.setYAxisRotation(shifter);
  //Joystick.setZAxisRotation();
  Joystick.setThrottle(0);
}