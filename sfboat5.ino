//Library: https://github.com/bolderflight/MPU9250


#include "MPU9250.h"

// an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
MPU9250 IMU(Wire, 0x68);
int status;

void setup() {
  // serial to display data
  Serial.begin(115200);
  while (!Serial) {}

  // start communication with IMU
  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }
}

void loop() {
  // read the sensor
  IMU.readSensor();
/*
  Serial.print("MagX: ");
  Serial.print(IMU.getMagX_uT(), 6);
  Serial.print("  ");
  Serial.print("MagY: ");
  Serial.print(IMU.getMagY_uT(), 6);
  Serial.print("  ");
  Serial.print("MagZ: ");
  Serial.println(IMU.getMagZ_uT(), 6);
*/
  float magx = IMU.getMagX_uT();
  float magy = IMU.getMagY_uT();
  float heading = (atan2((int)magy,(int)magx));
  float heading_deg = heading * 180 / PI;
  float heading_deg_180 = heading_deg + 180;
  Serial.println(heading_deg_180);
}
