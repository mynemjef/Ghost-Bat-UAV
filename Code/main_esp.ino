
#define SERVOMINPULSE  110
#define SERVOMAXPULSE  590
#define TRIGGER_PIN     5
#define ECHO_PIN     18
#define MPU_addr  0x68
#define relay1   2
#define relay2   4

static const int RXPin = 16, TXPin = 17;
static const uint32_t GPSBaud = 9600;
double roll, pitch;
int16_t X,Y,Z;

#include <Adafruit_MLX90614.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>

Adafruit_MLX90614 mlx = Adafruit_MLX90614();
Adafruit_PWMServoDriver ServoController = Adafruit_PWMServoDriver(0x40);
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);
SoftwareSerial US100Serial(TRIGGER_PIN, ECHO_PIN);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);

void setup() {
  pinMode(relay1, OUTPUT);
  pinMode(relay2, OUTPUT);
  Serial.begin(115200);
  mlx.begin();
  US100Serial.begin(9600);
  ServoController.begin();
  ServoController.setPWMFreq(60);
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);
  Wire.write(0);
  Wire.endTransmission(true);
  ss.begin(GPSBaud);
}

int angleToPulse(int ang){
   int pulse = map(ang,0, 180, SERVOMINPULSE,SERVOMAXPULSE);
   return pulse;
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do 
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

static void printFloat(float val, bool valid, int len, int prec)
{
  if (!valid)
  {
    while (len-- > 1)
      Serial.print('*');
    Serial.print(' ');
  }
  else
  {
    Serial.print(val, prec);
    int vi = abs((int)val);
    int flen = prec + (val < 0.0 ? 2 : 1); // . and -
    flen += vi >= 1000 ? 4 : vi >= 100 ? 3 : vi >= 10 ? 2 : 1;
    for (int i=flen; i<len; ++i)
      Serial.print(' ');
  }
  smartDelay(0);
}

static void printInt(unsigned long val, bool valid, int len)
{
  char sz[32] = "*****************";
  if (valid)
    sprintf(sz, "%ld", val);
  sz[len] = 0;
  for (int i=strlen(sz); i<len; ++i)
    sz[i] = ' ';
  if (len > 0) 
    sz[len-1] = ' ';
  Serial.print(sz);
  smartDelay(0);
}

static void printDateTime(TinyGPSDate &d, TinyGPSTime &t)
{
  if (!d.isValid())
  {
    Serial.print(F("********** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d/%02d/%02d ", d.month(), d.day(), d.year());
    Serial.print(sz);
  }
  
  if (!t.isValid())
  {
    Serial.print(F("******** "));
  }
  else
  {
    char sz[32];
    sprintf(sz, "%02d:%02d:%02d ", t.hour(), t.minute(), t.second());
    Serial.print(sz);
  }

  printInt(d.age(), d.isValid(), 5);
  smartDelay(0);
}

static void printStr(const char *str, int len)
{
  int slen = strlen(str);
  for (int i=0; i<len; ++i)
    Serial.print(i<slen ? str[i] : ' ');
  smartDelay(0);
}

void loop() {
  digitalWrite(relay1, LOW);
  digitalWrite(relay2, LOW);
  delay(1000); 
  digitalWrite(relay1, HIGH);
  digitalWrite(relay2, HIGH);
  delay(1000);

  US100Serial.flush();
  US100Serial.write(0x55); 
  delay(50);
  MSByteDist = US100Serial.read(); 
  LSByteDist = US100Serial.read();
  mmDist  = MSByteDist * 256 + LSByteDist; 
  Serial.print("Distance: ");
  Serial.print(mmDist, DEC);
  Serial.println(" mm");
  delay(200);
  
  Serial.print("Ambient = "); Serial.print(mlx.readAmbientTempC());
  Serial.print("*C\tObject = "); Serial.print(mlx.readObjectTempC()); Serial.println("*C");

  ServoController.setPWM(0, 0, angleToPulse(180));
  delay(1000);
  ServoController.setPWM(0, 0, angleToPulse(0));

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

  static const double LONDON_LAT = 51.508131, LONDON_LON = -0.128002;

  printInt(gps.satellites.value(), gps.satellites.isValid(), 5);
  printFloat(gps.hdop.hdop(), gps.hdop.isValid(), 6, 1);
  printFloat(gps.location.lat(), gps.location.isValid(), 11, 6);
  printFloat(gps.location.lng(), gps.location.isValid(), 12, 6);
  printInt(gps.location.age(), gps.location.isValid(), 5);
  printDateTime(gps.date, gps.time);
  printFloat(gps.altitude.meters(), gps.altitude.isValid(), 7, 2);
  printFloat(gps.course.deg(), gps.course.isValid(), 7, 2);
  printFloat(gps.speed.kmph(), gps.speed.isValid(), 6, 2);
  printStr(gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ", 6);

  unsigned long distanceKmToLondon =
    (unsigned long)TinyGPSPlus::distanceBetween(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON) / 1000;
  printInt(distanceKmToLondon, gps.location.isValid(), 9);

  double courseToLondon =
    TinyGPSPlus::courseTo(
      gps.location.lat(),
      gps.location.lng(),
      LONDON_LAT, 
      LONDON_LON);

  printFloat(courseToLondon, gps.location.isValid(), 7, 2);

  const char *cardinalToLondon = TinyGPSPlus::cardinal(courseToLondon);

  printStr(gps.location.isValid() ? cardinalToLondon : "*** ", 6);

  printInt(gps.charsProcessed(), true, 6);
  printInt(gps.sentencesWithFix(), true, 10);
  printInt(gps.failedChecksum(), true, 9);
  Serial.println();
  
  smartDelay(1000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));



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
