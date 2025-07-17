#include <Wire.h>
#include "MotorcycleIMU.h"
#include "Logger.h"

#define SENSOR_ADDRESS 0x68 // İlk cihazın I2C adresi (örneğin MPU6050)
#define SLAVE_ADDRESS 0x08 // Arduino Nano kendi cihazı

#define I2C_SENSOR Wire 
#define ID 1
#define UPDATE_INTERVAL 50

MotorcycleIMU mpu(SENSOR_ADDRESS); // 1. sensör

byte dataToSend = 0; // Master'a gönderilecek veri
byte receivedData = 0; // Master'dan alınan veri

Logger logger;

char buffer[20];

float xAngles;
float yAngles;
float zAngles;
unsigned long lastTime;

void setup() {
 
  logger.init(9600);
  //while (!Serial);
  I2C_SENSOR.begin(SLAVE_ADDRESS); 
  logger.setLogLevel(INFO);
  logger.write(INFO, "Master Test Başladı");

  mpu.setWire(&I2C_SENSOR);
  sprintf(buffer, "MPU6050 ID %d", ID);
  mpu.begin(buffer);
  lastTime = millis();
}

void loop() {
  unsigned long currentTime = millis();
  if (currentTime - lastTime >= UPDATE_INTERVAL) {
    float dt = (currentTime - lastTime) / 1000.0;
    lastTime = currentTime;
    mpu.update(dt);
    xAngles = mpu.getAngleX();
    yAngles = mpu.getAngleY();
    zAngles = mpu.getAngleZ();
    Serial.print(" ID_");
    Serial.print(ID);
    Serial.print("_X_Angle:");
    Serial.print(xAngles);
    Serial.println();
  }
}
// Master'dan veri alındığında çağrılır
void receiveEvent(int byteCount) {
  while (Wire.available()) {
    receivedData = Wire.read(); // Gelen veriyi oku
    Serial.print("Alınan veri: ");
    Serial.println(receivedData);
    dataToSend = receivedData + 1; // Örnek: Gelen veriyi artır ve sakla
  }
}

// Master veri istediğinde çağrılır
void requestEvent() {
  Wire.write(dataToSend); // Master'a veri gönder
  Serial.print("Gönderilen veri: ");
  Serial.println(dataToSend);
}