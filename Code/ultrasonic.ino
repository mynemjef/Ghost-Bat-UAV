#include <SoftwareSerial.h>
 
#define TRIGGER_PIN  5
#define ECHO_PIN     18
 
SoftwareSerial US100Serial(TRIGGER_PIN, ECHO_PIN);
 
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
