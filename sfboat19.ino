//start program

#include "MPU9250.h" //Library: https://github.com/bolderflight/MPU9250
#include <TinyGPS++.h>

TinyGPSPlus gps;
unsigned long time;

MPU9250 IMU(Wire, 0x68); // an MPU9250 object with the MPU-9250 sensor on I2C bus 0 with address 0x68
int status;
float target_heading;
float avg_heading;
float magx;
float magy;
float mylat;
float mylong;
float targetlat;
float targetlong;
float distance_to_target;
int sp;  // speed
int st;  // steer
int lpump;
int rpump;
int loc_counter = 0;

// shoreline
double latarray[4] = {37.432185, 37.432235, 37.43341, 37.433402};
double longarray[4] = { -122.090440, -122.091529, -122.091550, -122.090187};

// barron park
//double latarray[4] = {37.408266, 37.407486, 37.409382, 37.409918};
//double longarray[4] = { -122.137790, -122.137441, -122.135521, -122.136509};


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
  Serial2.begin(9600); //(16,17)      GPS
  Serial1.begin(9600); //(19,18)      Bluetooth
  Serial.begin(157600);//( 1 and 0)   COMM

  while (!Serial) {}
  Serial1.println("Hello from Arduino");   //bluetooth
  Serial1.println("setup-start");          //bluetooth

  avg_heading = 0;
  magx = 0;
  magy = 0;

  Serial1.println(" mag.calibrate ");     //bluetooth
  IMU.setMagCalX(33.0, 0.95);
  IMU.setMagCalX(-15.0, 1.10);
  IMU.setMagCalX(37.62, 0.99);



  // Setup pins for RC control input
  pinMode(6, INPUT);
  pinMode(5, INPUT);


  // Setup Pins for Pump Thruster control
  lpump = 46;
  rpump = 48;
  pinMode(lpump, OUTPUT);
  pinMode(rpump, OUTPUT);
  // Start by truning the pumps off
  digitalWrite(lpump, HIGH);
  digitalWrite(rpump, HIGH);

  //target_heading = 300;

  // start communication with IMU
  Serial1.println(" mag.comm-start ");      //bluetooth
  status = IMU.begin();
  if (status < 0) {                         //wait for communication with IMU
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while (1) {}
  }

  mylat = 37.409; //house lat = 37.409
  mylong = -122.136; //house long = -122.136
  targetlat = latarray[loc_counter]; //shorline lake lat = 37.433
  targetlong = longarray[loc_counter]; //shorline lake long = -122.091


  target_heading = TinyGPSPlus::courseTo(mylat, mylong, targetlat, targetlong);
  distance_to_target = TinyGPSPlus::distanceBetween(mylat, mylong, targetlat, targetlong) / 1000.0;
}

void rc_control() {
  Serial1.println(" rc-start ");
  Serial.print(sp);
  Serial.print(" ");
  Serial.println(st);
  if ((sp > 1500) && (sp < 2100)) {       //Go forward
    if ((st > 1700) && (st < 2100)) {     //Right turn
      digitalWrite(lpump, LOW);
      digitalWrite(rpump, HIGH);
      Serial.print(" turn right");
      Serial1.println(" rc-right ");
    }
    else if ((st > 900) && (st < 1300)) { //Left turn
      digitalWrite(lpump, HIGH);
      digitalWrite(rpump, LOW);
      Serial.print(" turn left");
      Serial1.println(" rc-left ");
    }
    else {
      digitalWrite(lpump, LOW);
      digitalWrite(rpump, LOW);
      Serial.print(" go strait");
      Serial1.println(" rc-forward ");
    }
  }  else {
    digitalWrite(lpump, HIGH);
    digitalWrite(rpump, HIGH);
    Serial.print(" stop ");
    Serial1.println(" rc-stop ");
  }

  }

void autonomous_control() {
  Serial1.println(" auto-start ");
  distance_to_target = TinyGPSPlus::distanceBetween(mylat, mylong, targetlat, targetlong) / 1000.0;

  //read gps data
  while (Serial2.available() > 0) {
    gps.encode(Serial2.read());
  }
  mylat = gps.location.lat(); //house lat = 37.409
  mylong = gps.location.lng(); //house long = -122.136

  targetlat = latarray[loc_counter];
  targetlong = longarray[loc_counter];


  target_heading = TinyGPSPlus::courseTo(mylat, mylong, targetlat, targetlong);
  distance_to_target = TinyGPSPlus::distanceBetween(mylat, mylong, targetlat, targetlong) / 1000.0;


  /* Serial.print(mylat,6);
     Serial.print(" ");

    Serial.print(mylong,6);
     Serial.print(" ");

    Serial.print(targetlat,6);
     Serial.print(" ");

    Serial.print(targetlong,6);
     Serial.println  (" ");*/

  // read the sensor
  IMU.readSensor();

  Serial.print("distance_to_target:  ");
  Serial.print(distance_to_target * 1000);
  Serial.print(" ");

  magx = 0.02 * IMU.getMagX_uT() + 0.98 * magx;
  magy = 0.02 * IMU.getMagY_uT() + 0.98 * magy;

  float heading = (atan2(-1 * magy, magx));
  float heading_deg = heading * 180 / PI;
  float heading_deg_180 = heading_deg + 180;
  avg_heading = heading_deg_180; //0.02 * heading_deg_180 + 0.98 * avg_heading;
  //float dif = avg_heading - target_heading;
  avg_heading = gps.course.deg();
  float dif = calculateDifferenceBetweenAngles(avg_heading, target_heading);
  Serial.print("target_heading= ");
  Serial.print(target_heading);
  Serial.print(" dif= ");
  Serial.print(dif);
  // if negetive -- turn left, if positive -- turn right
  // Serial.print(" heading= ");
  //Serial.print(heading_deg_180);
  Serial.print(" avg heading= ");
  Serial.print(avg_heading);

  time = millis();
  int sec = (int) time / 1000;

  if ((dif < 25) && (dif > -25) && (distance_to_target > 0.005)) {
    Serial1.print(" auto-forward . head = ");
    Serial1.print(avg_heading);
    Serial1.print(" dif = ");
    Serial1.print(dif);
    Serial1.print(" dist = ");
    Serial1.println(distance_to_target);
    digitalWrite(lpump, LOW);
    digitalWrite(rpump, LOW);
    Serial.print(" Both ");
  }
  else if ((dif < -25) && (distance_to_target > 0.005)) {
    Serial1.print(" auto-left . head = ");
    Serial1.print(avg_heading);
    Serial1.print(" dif = ");
    Serial1.print(dif);
    Serial1.print(" dist = ");
    Serial1.println(distance_to_target);
    //turn left
    if (sec % 5 == 0)
      digitalWrite(lpump, HIGH);
    else
      digitalWrite(lpump, LOW);
    digitalWrite(rpump, LOW);


    Serial.print(" Turn Left ");

  }
  else if ((dif > 25) && (distance_to_target > 0.005)) {
    Serial1.println(" auto-right . head = ");
    Serial1.print(avg_heading);
    Serial1.print(" dif = ");
    Serial1.print(dif);
    Serial1.print(" dist = ");
    Serial1.println(distance_to_target);
    //turn right

    digitalWrite(lpump, LOW);
    if (sec % 5 == 0)
      digitalWrite(rpump, HIGH);
    else
      digitalWrite(rpump, LOW);

    Serial.print(" Turn Right ");
  }
  else if (distance_to_target < 0.005) {
    Serial1.println(" auto-stop . head = ");
    Serial1.print(avg_heading);
    Serial1.print(" dif = ");
    Serial1.print(dif);
    Serial1.print(" dist = ");
    Serial1.println(distance_to_target);
    //stop moving
    digitalWrite(lpump, HIGH);
    digitalWrite(rpump, HIGH);
    loc_counter = loc_counter + 1;
    if (loc_counter >= 4) {
      loc_counter = 0;
    }
  }
  Serial.print(" counter ");
  Serial.println(loc_counter);
  Serial1.print(" count = ");
  Serial1.print(loc_counter);
}


void loop() {
  Serial1.print(" L ");

  //read the RC data
  st = pulseIn(5, HIGH, 30000);
  sp = pulseIn(6, HIGH, 30000);

  if (sp > 0 ) { // RC control is active
    rc_control();
  } else {  //no rc, use autonomous mode
    autonomous_control();
  }
  //delay(50);
}
