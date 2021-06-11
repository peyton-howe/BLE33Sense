/*
  HTS221 - Read Sensors

  This example reads data from the on-board HTS221 sensor of the
  Nano 33 BLE Sense and prints the temperature and humidity sensor
  values to the Serial Monitor once a second.

  The circuit:
  - Arduino Nano 33 BLE Sense

  This example code is in the public domain.
*/

//#include <Arduino_HTS221.h>
#include <Arduino_LPS22HB.h>
#include <Arduino_LSM9DS1.h>

unsigned long previousMillis = 0;
unsigned long previousMillis1 = 0;
long interval = 500;
long interval2 = 750;

float temperature, pressure, humidity, altitude;

int heading;                          
float pitch, roll;
float magX,magY,magZ;
float xAcc, yAcc, zAcc;
float magXcomp, magYcomp;
float accXnorm, accYnorm;
float xAcc_norm, yAcc_norm;

void setup() {
  Serial.begin(115200);

  if (/*!HTS.begin() or*/ !BARO.begin() or !IMU.begin()) { 
    Serial.println("Failed to initialize the sensors!");
    while (1);
  }

    //Magnetometer setup
    IMU.setMagnetFS(0);  
    IMU.setMagnetODR(8); 
    IMU.setMagnetOffset(11.148682, 9.513550, -0.106812);           // place calibration data here
    IMU.setMagnetSlope (0.004445, 0.004172, 0.004430);             // place calibration data here
    
    //Accelerometer setup
    IMU.setAccelFS(2);
    IMU.setAccelODR(5);
    IMU.setAccelOffset(0.002635, -0.033756, 0.001941);             // place calibration data here
    IMU.setAccelSlope (0.995365, 1.000206, 1.003778);              // place calibration data here
}

void weather() {
  //read all the sensor values
  //temperature = (1.1965 * HTS.readTemperature()) - 22.891;
  //humidity = HTS.readHumidity();
  pressure = (BARO.readPressure() * 10) - 1.5775;
  altitude = abs((1 - pow((pressure/1013.25), 0.190284)) * 145366.45);

  //Serial.print(temperature);
  //Serial.print(",");
  //Serial.print(humidity);
  //Serial.print(",");
  Serial.print(pressure);
  Serial.print(",");
  Serial.println(altitude);
}

void compass () {  
  
  //Read Magnetometer & Accelerometer values if available
  if (IMU.accelerationAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(xAcc, yAcc, zAcc);
    IMU.readMagneticField(magX, magY, magZ);
  }

  //Correct the coordinate system for calculations
  magX = -magX;
  magY = -magY;
  magZ = -magZ;

  //Normalize raw accelerometer values.
  xAcc_norm = yAcc/sqrt(xAcc * xAcc + yAcc * yAcc + zAcc * zAcc);
  yAcc_norm = -xAcc/sqrt(xAcc * xAcc + yAcc * yAcc + zAcc * zAcc);

  //Calculate pitch and roll  
  pitch = -asin(yAcc_norm);
  roll = asin(xAcc_norm/cos(pitch));

  //Calculate the new tilt-compensated values
  magXcomp = magX*cos(pitch)+magZ*sin(pitch); 
  magYcomp = magX*sin(roll)*sin(pitch)+magY*cos(roll)-magZ*sin(roll)*cos(pitch);

  //Calculate heading
  heading = (180 * atan2(magYcomp, magXcomp)) / PI;

  //Correct heading to prevent negative values
  if (heading < 0) {
    heading += 360;
  }

  //Print the heading value
  Serial.print(heading);

  //Print the direction after the heading
  if ((heading >= 348.75) or (heading < 11.25)) {
    Serial.println(" N");
  }else if ((heading >= 11.25) && (heading < 33.75)){
    Serial.println(" NNE");
  }else if ((heading >= 33.75) && (heading < 56.25)){
    Serial.println(" NE");
  }else if ((heading >= 56.25) && (heading < 78.75)){
    Serial.println(" ENE");
  }else if ((heading >= 78.75) && (heading < 101.25)){
    Serial.println(" E");
  }else if ((heading >= 101.25) && (heading < 123.75)){
    Serial.println(" ESE");
  }else if ((heading >= 123.75) && (heading < 146.25)){
    Serial.println(" SE");
  }else if ((heading >= 146.25) && (heading < 168.75)){
    Serial.println(" SSE");
  }else if ((heading >= 168.75) && (heading < 191.25)){
    Serial.println(" S");
  }else if ((heading >= 191.25) && (heading < 213.75)){
    Serial.println(" SSW");
  }else if ((heading >= 213.75) && (heading < 236.25)){
    Serial.println(" SW");
  }else if ((heading >= 236.25) && (heading < 258.75)){
    Serial.println(" WSW");
  }else if ((heading >= 258.75) && (heading < 281.25)){
    Serial.println(" W");
  }else if ((heading >= 281.25) && (heading < 303.75)){
    Serial.println(" WNW");
  }else if ((heading >= 303.75) && (heading < 326.25)){
    Serial.println(" NW");
  }else if ((heading >= 326.25) && (heading < 348.75)){
    Serial.println(" NNW");
  }
    
}

void loop() {
  if(millis() - previousMillis > interval){
    previousMillis = millis();
    weather();
  }
  if(millis() - previousMillis1 > interval2) {
    previousMillis1 = millis();
    compass();
  }
}
