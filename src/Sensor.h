#ifndef Sensor_h
#define Sensor_h

#include "Variables.h"
#include "math.h"

void Sensor() {
  MPU_getData();

  gx = gx - gxC;
  gy= gy- gyC;
  g_z = g_z - gzC;

  ax = ax - axC;    //Kalibrierwerte Beschl. Sensor
  ay = ay - ayC;
  az = az - azC;

  gyroX = float(gx / 65.5) * invX;
  gyroY = float(gy/ 65.5) * invY;
  gyroZ = float(g_z / 65.5) * invZ;
  
  //Gyro angle calculations
  anglePitch += gyroY / Frequenz;                                  //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  angleRoll += gyroX / Frequenz;                                    //Calculate the traveled roll angle and add this to the angle_roll variable.

  anglePitch -= angleRoll * sin(gyroZ / Frequenz * (M_PI / 180.0));                //If the IMU has yawed transfer the roll angle to the pitch angle.
  angleRoll += anglePitch * sin(gyroZ / Frequenz * (M_PI / 180.0));                //If the IMU has yawed transfer the pitch angle to the roll angle.

  //Accelerometer angle calculations
  acc_total_vector = sqrt((ax * ax) + (ay * ay) + (az * az)); //Calculate the total accelerometer vector.

  if (abs(ay) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
    accY = asin((float)ay / acc_total_vector) * 57.296;       //Calculate the pitch angle.
  }
  if (abs(ax) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
    accX = asin((float)ax / acc_total_vector) * -57.296;       //Calculate the roll angle.
  }
  // anglePitch = (anglePitch * 0.9996) + (accY * 0.0004);
  // angleRoll = (angleRoll * 0.9996) + (accX * 0.0004);
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
        axC += ax;
        ayC += ay;
        azC += az;
        yield();
      }
    }
    gxC /= 10000.0;
    gyC /= 10000.0;
    gzC /= 10000.0;
    axC /= 10000.0;
    ayC /= 10000.0;
    azC /= 10000.0;
    delay(10);

    //Accelerometer angle calculations
    acc_total_vector = sqrt((ax * ax) + (ay * ay) + (az * az)); //Calculate the total accelerometer vector.

    if (abs(ax) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
      anglePitch = asin((float)ax / acc_total_vector) * 57.296;       //Calculate the pitch angle.
    }
    if (abs(ay) < acc_total_vector) {                                      //Prevent the asin function to produce a NaN
      angleRoll = asin((float)ay / acc_total_vector) * -57.296;       //Calculate the roll angle.
    }
  }
  else
  {
    HardwareIssues = hardwareError((uint8_t)HardwareIssues | GYRO); //Irgendwas stimmt mit hasi nicht
    debugPrint("Gyro not found!!\n");
  }
}

#endif
