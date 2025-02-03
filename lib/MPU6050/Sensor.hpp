#ifndef Sensor_h
#define Sensor_h

#include "Variables.hpp"
#include "math.h"
#include <Wire.h>

void MPU_getData(void) {
  Wire.beginTransmission(0x68);                                   //Start communication with the gyro.
  Wire.write(0x3B);                                               //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                         //End the transmission.
  Wire.requestFrom(0x68, 14);                                     //Request 14 bytes from the gyro.
  uint64_t timer = micros64();                                      //Save the actual timestamp
  while (Wire.available() < 14 && (micros64() - timer < durchlaufT))//Wait until the 14 bytes are received. and check if the Gyro doesn´t response
  {
    // yield();
  }

  if (Wire.available() < 14)                                     //If the Gyro didn´t response everything expected
  {
    ax = 0;
    ay = 0;
    az = 0;
    gx = 0;
    gy = 0;
    g_z = 0;
    HardwareIssues = hardwareError((uint8_t)HardwareIssues | GYRO);
    debugPrint("Sensor took too long to respond\n");
    return;
  }

  HardwareIssues = hardwareError((uint8_t)HardwareIssues & ~GYRO);
  ax = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_x variable.
  ay = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_y variable.
  az = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_z variable.
  temperature = Wire.read() << 8 | Wire.read();                  //Add the low and high byte to the temperature variable.
  gx = Wire.read() << 8 | Wire.read();                           //Read high and low part of the angular data.
  gy = Wire.read() << 8 | Wire.read();                           //Read high and low part of the angular data.
  g_z = Wire.read() << 8 | Wire.read();                           //Read high and low part of the angular data.
}

void getAccelAngles(float accX, float accY, float accZ, float &accelPitch, float &accelRoll)
{
  //Accelerometer angle calculations
  float acc_total_vector = sqrt((accX * accX) + (accY * accY) + (accZ * accZ)); //Calculate the total accelerometer vector.
  if(acc_total_vector != 0)
  {
    // if (abs(accY) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
      accelRoll = asin((float)accY / acc_total_vector) * -(1.0/(M_PI / 180.0));       //Calculate the pitch angle.
    // }
    // if (abs(accX) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
      accelPitch = asin((float)accX / acc_total_vector) * (1.0/(M_PI / 180.0));       //Calculate the roll angle.
    // }
    anglePitch = (anglePitch * 0.9996) + (accelPitch * 0.0004);
    angleRoll = (angleRoll * 0.9996) + (accelRoll * 0.0004);
  }
}
bool ping_gyro(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x75);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 1);
  uint64_t timer = millis() + 100;
  while (Wire.available() < 1 && timer > micros64())              //Wait till the Gyro responded with 1 Byte, or wait 100 ms... what happens earlier
  {
    //do nothing
  }
  if (Wire.read() == 0x68)                                       //If the Delivered Byte is the hardware address then return success
  {
    return true;
  }
  else
  {
    return false;
  }
}
void Sensor() {
  MPU_getData();

  gx = gx - gxC;
  gy = gy - gyC;
  g_z = g_z - gzC;

  // ax = ax - axC;    //Kalibrierwerte Beschl. Sensor
  // ay = ay - ayC;
  // az = az - azC;

  accX = float(ax / 4096.0);
  accY = float(ay / 4096.0);
  accZ = float(az / 4096.0);

  gyroX = float(gx / 65.5) * invX;
  gyroY = float(gy / 65.5) * invY;
  gyroZ = float(g_z / 65.5) * invZ;
  
  //Gyro angle calculations
  anglePitch += gyroY / Frequenz;                                  //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angleRoll += gyroX / Frequenz;                                    //Calculate the traveled roll angle and add this to the angle_roll variable.

  anglePitch -= angleRoll * sin(gyroZ / Frequenz * (M_PI / 180.0));                //If the IMU has yawed transfer the roll angle to the pitch angle.
  angleRoll += anglePitch * sin(gyroZ / Frequenz * (M_PI / 180.0));                //If the IMU has yawed transfer the pitch angle to the roll angle.

  getAccelAngles(accX, accY, accZ, accelPitch, accelRoll);
  // debugSensor();
}

void SensorInit() {
  Wire.begin();
  Wire.setClock(400000);                                                //I2C Frequenz auf 400kHz setzen
  debugPrint("Pinging Gyro\n");
  if (ping_gyro()) {
    uint8_t success = false;
    debugPrint("Found Gyro\n");
    Wire.beginTransmission(0x68);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                  //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                  //Set the register bits as 00000000 to activate the gyro
    success = Wire.endTransmission();                                            //End the transmission with the gyro.
    delay(10);
    if(success != 0)
    {
      debugPrint("Failed to configure sensor1: ");
      debugPrint(success);
      debugPrint("\n");
      return;
    }
    //Configure the accelerometer (+/-8g)
    Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
    Wire.write(0x1C);                                                    //Send the requested starting register
    Wire.write(0x10);                                                    //Set the requested starting register
    success = Wire.endTransmission();                                              //End the transmission
    delay(10);
    if(success != 0)
    {
      debugPrint("Failed to configure sensor2: ");
      debugPrint(success);
      debugPrint("\n");
      return;
    }
    //Configure the gyro (500dps full scale)
    Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
    Wire.write(0x1B);                                                    //Send the requested starting register
    Wire.write(0x08);                                                    //Set the requested starting register
    success = Wire.endTransmission() ;                                              //End the transmission
    delay(10);
    if(success != 0)
    {
      debugPrint("Failed to configure sensor3: ");
      debugPrint(success);
      debugPrint("\n");
      return;
    }
    Wire.beginTransmission(0x68);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    success = Wire.endTransmission() ;                                                    //End the transmission with the gyro
    delay(10);
    if(success != 0)
    {
      debugPrint("Failed to configure sensor4: ");
      debugPrint(success);
      debugPrint("\n");
      return;
    }
    debugPrint("Configured Sensor\n");


    for (uint8_t b = 0; b < 10; b++) {
      debugPrint("Calibration..\n");
      for (uint16_t i = 0; i < 1000; i++) { //Sensor kalibrieren
        MPU_getData();
        gxC += gx;
        gyC += gy;
        gzC += g_z;
        // axC += ax;
        // ayC += ay;
        // azC += az;
        yield();
      }
    }
    gxC /= 10000.0;
    gyC /= 10000.0;
    gzC /= 10000.0;
    // axC /= 10000.0;
    // ayC /= 10000.0;
    // azC /= 10000.0;
    delay(10);
    accX = float(ax / 4096.0);
    accY = float(ay / 4096.0);
    accZ = float(az / 4096.0);
    getAccelAngles(accX, accY, accZ, accelPitch, accelRoll);
    anglePitch = accelPitch;
    angleRoll = accelRoll;
  }
  else
  {
    HardwareIssues = hardwareError((uint8_t)HardwareIssues | GYRO); //Irgendwas stimmt mit hasi nicht
    debugPrint("Gyro not found!!\n");
  }
}

#endif
