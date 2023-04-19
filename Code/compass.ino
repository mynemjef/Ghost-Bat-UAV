#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup(void){
  Serial.begin(115200);
  if(!mag.begin()){
    while(1);
  }
}

void loop(void){
  sensors_event_t event;
  mag.getEvent(&event);

  float heading = atan2(event.magnetic.y, event.magnetic.x);

  float declinationAngle = 0.096;
  heading += declinationAngle;

  if(heading < 0)
  heading += 2*PI;

  if(heading > 2*PI)
  heading -= 2*PI;

  float headingDegrees = heading * 180/M_PI;

  Serial.println(headingDegrees);
}
