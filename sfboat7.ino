
//Library: https://github.com/bolderflight/MPU9250
#include "MPU9250.h"
#include <TinyGPS++.h>

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;
float target_heading;
float avg_heading;
float magx;
float magy;
float mylat;
float mylong;
float targetlat;
float targetlong;

//Bearing function from https://gis.stackexchange.com/questions/252672/calculate-bearing-between-two-decimal-gps-coordinates-arduino-c
/*
  lat = current gps latitude
  lon = current gps longitude
  lat2 = destiny gps latitude
  lon2 = destiny gps longitude */
float bearing(float lat, float lon, float lat2, float lon2) {

  float teta1 = radians(lat);
  float teta2 = radians(lat2);
  float delta1 = radians(lat2 - lat);
  float delta2 = radians(lon2 - lon);

  //==================Heading Formula Calculation================//

  float y = sin(delta2) * cos(teta2);
  float x = cos(teta1) * sin(teta2) - sin(teta1) * cos(teta2) * cos(delta2);
  float brng = atan2(y, x);
  brng = degrees(brng);// radians to degrees
  brng = ( ((int)brng + 360) % 360 );

  Serial.print("Heading GPS: ");
  Serial.println(brng);

  return brng;


}
//from https://www.codeproject.com/Articles/59789/Calculate-the-Real-Difference-Between-Two-Angles-K
float calculateDifferenceBetweenAngles(float firstAngle, float secondAngle)
  {
        float difference = secondAngle - firstAngle;
        while (difference < -180) difference += 360;
        while (difference > 180) difference -= 360;
        return difference;
 }

void setup() {
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  avg_heading = 0;
  magx = 0;
  magy = 0;

  target_heading = 300;

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  mylat = 37.409; //house lat = 37.409
  mylong = -122.136; //house long = -122.136
  targetlat = 37.433; //shorline lake lat = 37.433
  targetlong = -122.091; //shorline lake long = -122.091

  target_heading = TinyGPSPlus::courseTo(mylat, mylong, targetlat, targetlong);
}

void loop() {
  // read the sensor
  IMU.readSensor();

  magx = 0.02 * IMU.getMagX_uT() + 0.98 * magx;
  magy = 0.02 * IMU.getMagY_uT() + 0.98 * magy;

  float heading = (atan2(-1 * magy, magx));
  float heading_deg = heading * 180 / PI;
  float heading_deg_180 = heading_deg + 180;
  avg_heading = heading_deg_180; //0.02 * heading_deg_180 + 0.98 * avg_heading;
  //float dif = avg_heading - target_heading;
  float dif = calculateDifferenceBetweenAngles(avg_heading,target_heading);
  Serial.print("target_heading= ");
  Serial.print(target_heading);
  Serial.print(" dif= ");
  Serial.print(dif);
  // if negetive -- turn left, if positive -- turn right
  Serial.print(" heading= ");
  Serial.print(heading_deg_180);
  Serial.print(" avg heading= ");
  Serial.println(avg_heading);

}
