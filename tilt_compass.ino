/* Tilt-compensated 16-point compass example for the Nano 33 BLE (Sense)
 * You need to run Femme Verbeek's LMS9DS1 library to run this example 
 * 
 * The compass must be calibrated for the magnetic disturbance of the setup and the environment.
 * Run the DIY calibration program for both the Magnetometer and Accelerometer from Verbeek's Library first,
 * and then copy/paste the Magnetometer and Accelerometer calibration data below where indicated.
 * 
 * Original calculations for pitch and roll are based on the LSM9DS0 IMU, which has a different coordinate system,
 * so the axis have to be corrected for the LSM9DS1 chip in the BLE 33 Sense board.
 * 
 * Written by Peyton Howe
 *     13 May 2021  
 * Released to the public domain
*/

#include <Arduino.h>
#include <Arduino_LSM9DS1.h>

//Time variables
unsigned long previousMillis = 0;
long interval = 500;

// Heading calculation variables
int heading;                          //change to float if you want decimal places
float pitch, roll;
float magX,magY,magZ;
float xAcc, yAcc, zAcc;
float magXcomp, magYcomp;
float accXnorm, accYnorm;
float xAcc_norm, yAcc_norm;


void setup() {
  Serial.begin(115200);
  
  // wait till the serial monitor connects
  while(!Serial) {
    delay(1); 
  }
  
  // initialize the magnetometer and accelerometer
  if (!IMU.begin()) {               
    Serial.println("Failed to initialize IMU!");  
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


void compass (){  
  
  if (IMU.accelerationAvailable() && IMU.magneticFieldAvailable()) {
    IMU.readAcceleration(xAcc, yAcc, zAcc);
    IMU.readMagnet(magX, magY, magZ);
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
    Serial.println("° N");
  }else if ((heading >= 11.25) && (heading < 33.75)){
    Serial.println("° NNE");
  }else if ((heading >= 33.75) && (heading < 56.25)){
    Serial.println("° NE");
  }else if ((heading >= 56.25) && (heading < 78.75)){
    Serial.println("° ENE");
  }else if ((heading >= 78.75) && (heading < 101.25)){
    Serial.println("° E");
  }else if ((heading >= 101.25) && (heading < 123.75)){
    Serial.println("° ESE");
  }else if ((heading >= 123.75) && (heading < 146.25)){
    Serial.println("° SE");
  }else if ((heading >= 146.25) && (heading < 168.75)){
    Serial.println("° SSE");
  }else if ((heading >= 168.75) && (heading < 191.25)){
    Serial.println("° S");
  }else if ((heading >= 191.25) && (heading < 213.75)){
    Serial.println("° SSW");
  }else if ((heading >= 213.75) && (heading < 236.25)){
    Serial.println("° SW");
  }else if ((heading >= 236.25) && (heading < 258.75)){
    Serial.println("° WSW");
  }else if ((heading >= 258.75) && (heading < 281.25)){
    Serial.println("° W");
  }else if ((heading >= 281.25) && (heading < 303.75)){
    Serial.println("° WNW");
  }else if ((heading >= 303.75) && (heading < 326.25)){
    Serial.println("° NW");
  }else if ((heading >= 326.25) && (heading < 348.75)){
    Serial.println("° NNW");
  }
    
}


void loop() {
  if (millis() - previousMillis > interval) {
    previousMillis = millis();
    compass();
  }
}
