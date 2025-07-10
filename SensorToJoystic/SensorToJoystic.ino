#include "MotorcycleIMU.h"
#include "Joystick.h"

MotorcycleIMU steering(0x69);
MotorcycleIMU frontBrake(0x68);

//This 0x68 is for front brake, 0x69 is for steering
// Make sure to set the correct addresses for your IMU sensors

int steeringValue = 0, frontBrakeValue = 0, rearBrakeValue = 0, leverClutchValue = 0;

void setup() {
    Serial.begin(115200);
    frontBrake.begin("Front Brake");
    steering.begin("Steering");
    Joystick.begin();
    frontBrake.setKalmanFilterIntensity(2.0f);
    steering.setKalmanFilterIntensity(2.0f);
}

void loop() {
    frontBrake.update();
    steering.update();
    
    if (frontBrake.getAngleY() > 0 && frontBrake.getAngleY() < 20)
        frontBrakeValue = map(frontBrake.getAngleY(), 0, 20, -127, 127);
    if (steering.getAngleY() > -25 && steering.getAngleY() < 25)
        steeringValue = map(steering.getAngleY(), -25, 25, -127, 127);
    Serial.print("Steering Angle: ");
    Serial.println(steering.getAngleY());
    Joystick.setXAxis(steeringValue);
    Joystick.setZAxis(frontBrakeValue);
    Joystick.setThrottle(0);
    delay(1);
}