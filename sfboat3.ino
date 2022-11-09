#include <SoftwareSerial.h>

#include <TinyGPS++.h>
TinyGPSPlus gps;

//SoftwareSerial ss(4, 3);
// only certain pins can rx
SoftwareSerial ss(10, 30);

void setup() {
  Serial.begin(9600);
  ss.begin(9600);
  Serial.println("start");
}

void loop() {
  while (ss.available() > 0) {
    gps.encode(ss.read());
    // get the byte data from the GPS
   // Serial.println("loop");
    //byte gpsData = ss.read();
    //Serial.write(gpsData);
    //Serial.println("loopend");
  }
  Serial.print("LAT=");  Serial.println(gps.location.lat(), 6);
  Serial.print("LONG="); Serial.println(gps.location.lng(), 6);
  Serial.print("ALT=");  Serial.println(gps.altitude.meters());
  Serial.println(gps.satellites.value());
  Serial.println(gps.hdop.value());
}
