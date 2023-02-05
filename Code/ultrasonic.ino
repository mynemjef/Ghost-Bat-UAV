#include <SoftwareSerial.h>
 
#define SDA_PIN     21
#define SCL_PIN     22
 
SoftwareSerial US100Serial(SCL_PIN, SDA_PIN);
 
unsigned int MSByteDist, LSByteDist, mmDist;
 
void setup() {
    Serial.begin(115200);
    US100Serial.begin(9600);
}
 
void loop() {
    US100Serial.flush();
    US100Serial.write(0x55); 
    delay(50);
    MSByteDist = US100Serial.read(); 
    LSByteDist = US100Serial.read();
    mmDist  = MSByteDist * 256 + LSByteDist; 
    Serial.print("Distance: ");
    Serial.print(mmDist, DEC);
    Serial.println(" mm");
}