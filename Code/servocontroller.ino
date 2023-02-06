#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver ServoController = Adafruit_PWMServoDriver(0x40);

#define SERVOMINPULSE  110
#define SERVOMAXPULSE  590
#define elevatorL      0
#define elevatorR      1
#define aileronL       2
#define aileronR       3
#define bay            4
#define releaseMchnsm  5

void setup() {
  Serial.begin(115200);
  ServoController.begin();
  ServoController.setPWMFreq(60);
}

void loop() {
  ServoController.setPWM(elevatorL, 0, angleToPulse(180));
  delay(2500);
  ServoController.setPWM(elevatorL, 0, angleToPulse(0));
  delay(1000);
}

int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMINPULSE,SERVOMAXPULSE);
   return pulse;
}
