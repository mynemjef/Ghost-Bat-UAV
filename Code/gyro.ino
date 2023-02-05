#include<Wire.h>
 
#define MPU_addr  0x68

int16_t X,Y,Z;
double roll, pitch;
 
void setup(){
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  Serial.begin(115200);
}

void loop(){
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);
  X=Wire.read()<<8|Wire.read();
  Y=Wire.read()<<8|Wire.read();
  Z=Wire.read()<<8|Wire.read();

  roll = atan2(Y , Z) * (180.0 / PI);
  pitch = atan2(X , Z) * (180.0 / PI) - 1.9;
  Serial.println(pitch);
  Serial.println(roll);
}