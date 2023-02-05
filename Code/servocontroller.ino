#include <Wire.h>

#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver ServoController = Adafruit_PWMServoDriver(0x40);

#define SERVOMINPULSE  110
#define SERVOMAXPULSE  590

void setup() {
  Serial.begin(115200);
  ServoController.begin();
  ServoController.setPWMFreq(60);
}

void loop() {
  ServoController.setPWM(0, 0, angleToPulse(180));
  ServoController.setPWM(1, 0, angleToPulse(180));
  delay(2500);
  ServoController.setPWM(0, 0, angleToPulse(0));
  ServoController.setPWM(1, 0, angleToPulse(0));
  delay(1000);
}

int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMINPULSE,SERVOMAXPULSE);
   return pulse;
}
