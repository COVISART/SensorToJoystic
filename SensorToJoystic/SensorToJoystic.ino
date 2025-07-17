#include <Wire.h>
#include "MotorcycleIMU.h"
#include <Joystick.h>
#include "Logger.h"

#define I2C_WIRE Wire 
#define RESET_PIN 7 // TCA9548A'nın RESET pinine bağlı Arduino pini
#define UPDATE_INTERVAL 50
#define TCAADDR 0x77 // TCA9548A default I2C address
#define SENSOR_COUNT 2

MotorcycleIMU mpu1(0x68); // 1. sensör
MotorcycleIMU mpu2(0x68); // 2. sensör
MotorcycleIMU mpu3(0x68); // 3. sensör
MotorcycleIMU mpu4(0x68); // 4. sensör
MotorcycleIMU mpu5(0x68); // 5. sensör

Logger logger;

MotorcycleIMU* mpus[] = {&mpu1, &mpu2, &mpu3, &mpu4, &mpu5}; // Sensör dizisi
char buffer[20];

float xAngles[5];
float yAngles[5];
float zAngles[5];

int steeringValue = 0, frontBrakeValue = 0, rearBrakeValue = 0, leverValue = 0, shifter = 0;

unsigned long lastTime;

bool tcaSelect(uint8_t channel, TwoWire* wire) {
  if (channel > 7) return false;

  wire->setWireTimeout(100000, true); // 1 saniye timeout ve resetle devam
  wire->beginTransmission(TCAADDR);
  wire->write(1 << channel);
  uint8_t result = wire->endTransmission();
  
  if (result != 0) {
    Serial.print("Hata: TCA9548A kanal seçimi basarisiz! Kanal: ");
    Serial.print(channel);
    Serial.print(" Kod: ");
    Serial.println(result);
    digitalWrite(RESET_PIN, LOW);
      delay(100);
    digitalWrite(RESET_PIN, HIGH);
    return false;
  }
  Serial.print("TCA9548A kanal seçimi basarili! Kanal: ");
  Serial.print(channel);
  delay(5); // Kanal seçiminden sonra kısa bir gecikme
  return true;
}

void setup() {
  pinMode(RESET_PIN, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);

  pinMode(0, OUTPUT);
  digitalWrite(RESET_PIN, HIGH);
  pinMode(1, OUTPUT);
  digitalWrite(RESET_PIN, LOW);
  I2C_WIRE.begin(); 

  logger.init(9600);
  while (!Serial);
  logger.setLogLevel(INFO);
  logger.write(INFO, "TCA9548A Test Başladı");

  //Her MPU6050 için başlatma işlemi
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    if(!tcaSelect(i, &I2C_WIRE)) return;
    mpus[i]->setWire(&I2C_WIRE);
    sprintf(buffer, "MPU6050 ID %d", i);
    mpus[i]->begin(buffer);
  }
  lastTime = millis();
  Joystick.begin();
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime < UPDATE_INTERVAL) return;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    delay(1000);
    if(!tcaSelect(i, &I2C_WIRE)) return;
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;
    mpus[i]->update(dt);
    
    xAngles[i] = mpus[i]->getAngleX();
    yAngles[i] = mpus[i]->getAngleY();
    zAngles[i] = mpus[i]->getAngleZ();
    //char message[32]; // Mesaj için yeterli alan
    //char angleStr[10];
    //dtostrf(xAngles[i], 6, 1, angleStr);
    //snprintf(message, sizeof(message), "ID_%d_X_Angle: %s", i + 1, angleStr);
    //logger.write(DEBUG, message);
    Serial.print(" ID_");
    Serial.print(i);
    Serial.print("_X_Angle:");
    Serial.print(xAngles[i]);
  }
  Serial.println();
  delay(10);

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