#define SERVOMINPULSE   110
#define SERVOMAXPULSE   590
#define TRIGGER_PIN     5
#define ECHO_PIN        18
#define MPU_addr        0x68
#define relay1          2
#define relay2          4
#define RXPin           16
#define TXPin           17
#define DEVICE          "ESP32"
#define WIFI_SSID       "A1_51B2"
#define WIFI_PASSWORD   "08ASD176VAG124569"
#define INFLUXDB_URL    "http://192.168.100.56:8086"
#define INFLUXDB_TOKEN  "1bddkVUqyzZ1ErEB-DegJ591yQ2xSX5e3oF6lLTOC3yiuLoXGwMnN8eHOicM89usEpjWnLv3BzItOZi0Ppxaog=="
#define INFLUXDB_ORG    "f8e3ad3c7ba285ca"
#define INFLUXDB_BUCKET "niga"
#define TZ_INFO         "UTC3"
#define LEFT_WING_SERVO_PIN 0
#define RIGHT_WING_SERVO_PIN 1
#define LEFT_STABILIZER_SERVO_PIN 2
#define RIGHT_STABILIZER_SERVO_PIN 3
#define LEFT_BAY_DOOR_SERVO_PIN 4
#define RIGHT_BAY_DOOR_SERVO_PIN 5
#define ESC_PIN_1 26
#define ESC_PIN_2 27
#define ESC_PIN_3 32
#define ESC_PIN_4 33




const int WING_ANGLE_MAX = 120;
const int WING_ANGLE_MIN = 60;
const int ROLL_ANGLE_MAX = 30;
const int PITCH_ANGLE_MAX = 20;


// Constants for throttle adjustments
const int THROTTLE_MAX = 100;
const int THROTTLE_CLIMB = 80;
const int THROTTLE_DESCEND = 60;
const float SPEED_THRESHOLD = 40; // km/h


// Define the waypoints
struct Waypoint {
  float altitude;
  float latitude;
  float longitude;
  int tag;
};


// Predefined waypoints
Waypoint waypoints[] = {
//  {alt, lat, longit, 0},   // Waypoint 1
//  {alt, lat, longit, 0},   // Waypoint 2
//  {alt, lat, longit, 0},   // Waypoint 3
//  {alt, lat, longit, 0}    // Waypoint 4
};


//Tags
//0 - head towards it, disable IR sensor readings
//1 - head towards it and calculate drop distance so package lands on those coordinates, head to next waypoint after dropping
//2 - head towards it, enable IR sensor readings for missiles when reached
//3 - head towards it, enable IR sensor readings for missiles when reached


#include <Adafruit_PWMServoDriver.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_MLX90614.h>
#include <Adafruit_Sensor.h>
#include <SoftwareSerial.h>
#include <TinyGPSPlus.h>
#include <Wire.h>
#include <WiFiMulti.h>
#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>


InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
Adafruit_PWMServoDriver ServoController = Adafruit_PWMServoDriver(0x40);
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
SoftwareSerial US100Serial(TRIGGER_PIN, ECHO_PIN);
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
SoftwareSerial ss(RXPin, TXPin);
Point sensor("wifi_status");
TinyGPSPlus gps;




static const uint32_t GPSBaud = 9600;
double roll, pitch;
int16_t X,Y,Z;




void setup() {
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Connecting to wifi");
  while (wifiMulti.run() != WL_CONNECTED) {
    Serial.print(".");
    delay(100);
  }
  Serial.println();
  // Add tags to the data point
  sensor.addTag("device", DEVICE);
  sensor.addTag("SSID", WiFi.SSID());
  //sensor.addTag(“TAG_NAME”, TAG_VARIABLE);
 
  // Accurate time is necessary for certificate validation and                   writing in batches
  // We use the NTP servers in your area as provided by:   https://www.pool.ntp.org/zone/
  // Syncing progress and the time will be printed to Serial.
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");
 
  // Check server connection
  if (client.validateConnection()) {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  } else {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
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
  //Activate servo battery
  digitalWrite(relay1, HIGH);
  surfaceTest();
  pinMode(ESC_PIN_1, OUTPUT);
  pinMode(ESC_PIN_2, OUTPUT);
  pinMode(ESC_PIN_3, OUTPUT);
  pinMode(ESC_PIN_4, OUTPUT);
}


void surfaceTest() {
  // Move servos 30 degrees up
  setServoAngle(LEFT_WING_SERVO_PIN, 60);
  setServoAngle(RIGHT_WING_SERVO_PIN, 60);
  setServoAngle(LEFT_STABILIZER_SERVO_PIN, 60);
  setServoAngle(RIGHT_STABILIZER_SERVO_PIN, 60);


  delay(2000); // Wait for 2 seconds


  setServoAngle(LEFT_WING_SERVO_PIN, 120);
  setServoAngle(RIGHT_WING_SERVO_PIN, 120);
  setServoAngle(LEFT_STABILIZER_SERVO_PIN, 120);
  setServoAngle(RIGHT_STABILIZER_SERVO_PIN, 120);


  delay(2000);


  // Return servos to 0 position (90 degrees)
  setServoAngle(LEFT_WING_SERVO_PIN, 90);
  setServoAngle(RIGHT_WING_SERVO_PIN, 90);
  setServoAngle(LEFT_STABILIZER_SERVO_PIN, 90);
  setServoAngle(RIGHT_STABILIZER_SERVO_PIN, 90);
  // Close bay doors (set them at 0 degrees)
  setServoAngle(LEFT_BAY_DOOR_SERVO_PIN, 0);
  setServoAngle(RIGHT_BAY_DOOR_SERVO_PIN, 0);
}


int readWiFiRSSI() {
  return WiFi.RSSI();
}




int readDistance() {
  US100Serial.flush();
  US100Serial.write(0x55);
  delay(50);
  int MSByteDist = US100Serial.read();
  int LSByteDist = US100Serial.read();
  int mmDist = MSByteDist * 256 + LSByteDist;
  return mmDist;
}




float readAmbientTemperature() {
  return mlx.readAmbientTempC();
}




float readObjectTemperature() {
  return mlx.readObjectTempC();
}




void moveServoToAngle(int angle) {
  int pulse = map(angle, 0, 180, SERVOMINPULSE, SERVOMAXPULSE);
  ServoController.setPWM(0, 0, pulse);
}




void readAccelerometerData(int16_t& x, int16_t& y, int16_t& z) {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true);
  x = Wire.read() << 8 | Wire.read();
  y = Wire.read() << 8 | Wire.read();
  z = Wire.read() << 8 | Wire.read();
}




float calculatePitch() {
  return atan2(readAccelerometerData(X, Y, Z).y, readAccelerometerData(X, Y, Z).z) * (180.0 / PI);
}




float calculateRoll() {
  return atan2(readAccelerometerData(X, Y, Z).x, readAccelerometerData(X, Y, Z).z) * (180.0 / PI) - 1.9;
}




int readSatelliteCount() {
  return gps.satellites.value();
}




float readHDOP() {
  return gps.hdop.hdop();
}




float readLatitude() {
  return gps.location.lat();
}




float readLongitude() {
  return gps.location.lng();
}




unsigned long readLocationAge() {
  return gps.location.age();
}




void printDateTime(TinyGPSDate& d, TinyGPSTime& t) {
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




float readAltitude() {
  return gps.altitude.meters();
}




float readCourse() {
  return gps.course.deg();
}




float readSpeed() {
  return gps.speed.kmph();
}




const char* readCardinalDirection() {
  return gps.course.isValid() ? TinyGPSPlus::cardinal(gps.course.deg()) : "*** ";
}




int readCharsProcessed() {
  return gps.charsProcessed();
}




int readSentencesWithFix() {
  return gps.sentencesWithFix();
}




int readFailedChecksum() {
  return gps.failedChecksum();
}




float readHeadingDegrees() {
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 0.096;
  heading += declinationAngle;
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;
  return heading * 180 / M_PI;
}


float calculateHeading(float currentLat, float currentLon, float waypointLat, float waypointLon) {
  float currentLatRad = radians(currentLat);
  float currentLonRad = radians(currentLon);
  float waypointLatRad = radians(waypointLat);
  float waypointLonRad = radians(waypointLon);


  float dLon = waypointLonRad - currentLonRad;


  float y = sin(dLon) * cos(waypointLatRad);
  float x = cos(currentLatRad) * sin(waypointLatRad) - sin(currentLatRad) * cos(waypointLatRad) * cos(dLon);


  float heading = atan2(y, x);
  heading = degrees(heading);
  heading = fmod((heading + 360), 360); // Normalize heading to range from 0 to 360 degrees


  return heading;
}


void setThrottlePercentage(int throttlePercent) {
  // Convert the throttle percentage to a value between 0 and 255
  int throttleValue = map(throttlePercent, 0, 100, 0, 255);


  // Set the ESCs to the calculated throttle value
  analogWrite(ESC_PIN_1, throttleValue);
  analogWrite(ESC_PIN_2, throttleValue);
  analogWrite(ESC_PIN_3, throttleValue);
  analogWrite(ESC_PIN_4, throttleValue);
}


void loop(){
  float heading = calculateHeading(currentLat, currentLon, waypointLat, waypointLon);


  sensor.clearFields();


  sensor.addField("rssi", readWiFiRSSI());
  Serial.print("Writing: ");
  Serial.println(sensor.toLineProtocol());
  int currentWaypoint = 0;
  int distance = readDistance();
  float ambientTemperature = readAmbientTemperature();
  float objectTemperature = readObjectTemperature();
  int satelliteCount = readSatelliteCount();
  float hdop = readHDOP();
  float latitude = readLatitude();
  float longitude = readLongitude();
  unsigned long locationAge = readLocationAge();
  float altitude = readAltitude();
  float course = readCourse();
  float speed = readSpeed();
  const char* cardinalDirection = readCardinalDirection();
  int charsProcessed = readCharsProcessed();
  int sentencesWithFix = readSentencesWithFix();
  int failedChecksum = readFailedChecksum();
  float headingDegrees = readHeadingDegrees();


  float rollAngle = calculateRoll();
  float pitchAngle = calculatePitch();
  float currentLat = readLatitude();
  float currentLon = readLongitude();
 
  // Calculate the heading to the next waypoint
  int currentWaypoint = 0; // Change this to the appropriate waypoint index
  float waypointLat = waypoints[currentWaypoint].latitude;
  float waypointLon = waypoints[currentWaypoint].longitude;
  float heading = calculateHeading(currentLat, currentLon, waypointLat, waypointLon);


  // Adjust control surfaces based on roll and pitch angles
  int leftWingAngle = map(static_cast<int>(rollAngle), -90, 90, WING_ANGLE_MIN, WING_ANGLE_MAX);
  int rightWingAngle = map(static_cast<int>(rollAngle), -90, 90, WING_ANGLE_MAX, WING_ANGLE_MIN);
  int leftStabilizerAngle = map(static_cast<int>(pitchAngle), -90, 90, WING_ANGLE_MIN, WING_ANGLE_MAX);
  int rightStabilizerAngle = map(static_cast<int>(pitchAngle), -90, 90, WING_ANGLE_MAX, WING_ANGLE_MIN);


  // Move control surfaces to the adjusted angles
  setServoAngle(LEFT_WING_SERVO_PIN, leftWingAngle);
  setServoAngle(RIGHT_WING_SERVO_PIN, rightWingAngle);
  setServoAngle(LEFT_STABILIZER_SERVO_PIN, leftStabilizerAngle);
  setServoAngle(RIGHT_STABILIZER_SERVO_PIN, rightStabilizerAngle);


  // Calculate throttle adjustment based on speed
  float currentSpeed = readSpeed();
  int throttlePercent = THROTTLE_MAX;
  if (currentSpeed < SPEED_THRESHOLD) {
    throttlePercent = THROTTLE_CLIMB;
  } else {
    throttlePercent = THROTTLE_DESCEND;
  }


  // Set the throttle to the calculated adjustment
  setThrottlePercentage(throttlePercent);


  // Check if the plane has reached the current waypoint
  // You can use the distance between the current location and the waypoint to determine this
  float distanceToWaypoint = calculateDistance(currentLat, currentLon, waypointLat, waypointLon);
  if (distanceToWaypoint < DISTANCE_THRESHOLD) {
    // Plane has reached the waypoint, move to the next one
    currentWaypoint++;
    if (currentWaypoint >= sizeof(waypoints) / sizeof(waypoints[0])) {
      // All waypoints reached, you can implement a landing or return behavior here
      return;
    }


    // Update the waypoint coordinates for the next waypoint
    waypointLat = waypoints[currentWaypoint].latitude;
    waypointLon = waypoints[currentWaypoint].longitude;
  }


  // Send sensor data to InfluxDB
  Point measurement("sensor_data");
    measurement.addField("rssi", WiFi.RSSI());


  Serial.print("Writing: ");
  Serial.println(measurement.toLineProtocol());


  if (client.writePoint(measurement)) {
    Serial.println("InfluxDB write success");
  } else {
    Serial.println("InfluxDB write failed");
  }
}
