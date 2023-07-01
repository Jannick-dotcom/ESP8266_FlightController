#ifndef Sensor_h
#define Sensor_h

#include "Variables.h"

void Sensor() {
  MPU_getData();

  temp->gx = temp->gx - temp->gxC;
  temp->gy= temp->gy- temp->gyC;
  temp->gz = temp->gz - temp->gzC;

  temp->ax = temp->ax - temp->axC;    //Kalibrierwerte Beschl. Sensor
  temp->ay = temp->ay - temp->ayC;
  temp->az = temp->az - temp->azC;

  temp->gyroX = float(temp->gx / 65.5) * temp->invX;
  temp->gyroY = float(temp->gy/ 65.5) * temp->invY;
  temp->gyroZ = float(temp->gz / 65.5) * temp->invZ;
  
  //Gyro angle calculations
  temp->anglePitch += temp->gyroY / temp->Frequenz;                                  //Calculate the traveled pitch angle and add this to the angle_pitch variable.
  temp->angleRoll += temp->gyroX / temp->Frequenz;                                    //Calculate the traveled roll angle and add this to the angle_roll variable.

  temp->anglePitch -= temp->angleRoll * sin(temp->gyroZ / temp->Frequenz * (M_PI / 180.0));                //If the IMU has yawed transfer the roll angle to the pitch angle.
  temp->angleRoll += temp->anglePitch * sin(temp->gyroZ / temp->Frequenz * (M_PI / 180.0));                //If the IMU has yawed transfer the pitch angle to the roll angle.

  //Accelerometer angle calculations
  temp->acc_total_vector = sqrt((temp->ax * temp->ax) + (temp->ay * temp->ay) + (temp->az * temp->az)); //Calculate the total accelerometer vector.

  if (abs(temp->ay) < temp->acc_total_vector) {                                      //Prevent the asin function to produce a NaN
    temp->accY = asin((float)temp->ay / temp->acc_total_vector) * 57.296;       //Calculate the pitch angle.
  }
  if (abs(temp->ax) < temp->acc_total_vector) {                                      //Prevent the asin function to produce a NaN
    temp->accX = asin((float)temp->ax / temp->acc_total_vector) * -57.296;       //Calculate the roll angle.
  }
  // temp->anglePitch = (temp->anglePitch * 0.9996) + (temp->accY * 0.0004);
  // temp->angleRoll = (temp->angleRoll * 0.9996) + (temp->accX * 0.0004);
}

void SensorInit() {
  Wire.begin();
  Wire.setClock(400000);                                                //I2C Frequenz auf 400kHz setzen

  if (ping_gyro()) {
    Wire.beginTransmission(0x68);                                      //Start communication with the address found during search.
    Wire.write(0x6B);                                                  //We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                                                  //Set the register bits as 00000000 to activate the gyro
    Wire.endTransmission();                                            //End the transmission with the gyro.
    delay(10);
    //Configure the accelerometer (+/-8g)
    Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
    Wire.write(0x1C);                                                    //Send the requested starting register
    Wire.write(0x10);                                                    //Set the requested starting register
    Wire.endTransmission();                                              //End the transmission
    delay(10);
    //Configure the gyro (500dps full scale)
    Wire.beginTransmission(0x68);                                        //Start communicating with the MPU-6050
    Wire.write(0x1B);                                                    //Send the requested starting register
    Wire.write(0x08);                                                    //Set the requested starting register
    Wire.endTransmission();                                              //End the transmission
    delay(10);
    
    Wire.beginTransmission(0x68);                                      //Start communication with the address found during search
    Wire.write(0x1A);                                                          //We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    Wire.endTransmission();                                                    //End the transmission with the gyro
    delay(10);


    for (uint8_t b = 0; b < 10; b++) {
      for (uint16_t i = 0; i < 1000; i++) { //Sensor kalibrieren
        MPU_getData();
        temp->gxC += temp->gx;
        temp->gyC += temp->gy;
        temp->gzC += temp->gz;
        temp->axC += temp->ax;
        temp->ayC += temp->ay;
        temp->azC += temp->az;
        yield();
      }
    }
    temp->gxC /= 10000.0;
    temp->gyC /= 10000.0;
    temp->gzC /= 10000.0;
    temp->axC /= 10000.0;
    temp->ayC /= 10000.0;
    temp->azC /= 10000.0;
    delay(10);

    //Accelerometer angle calculations
    temp->acc_total_vector = sqrt((temp->ax * temp->ax) + (temp->ay * temp->ay) + (temp->az * temp->az)); //Calculate the total accelerometer vector.

    if (abs(temp->ax) < temp->acc_total_vector) {                                      //Prevent the asin function to produce a NaN
      temp->anglePitch = asin((float)temp->ax / temp->acc_total_vector) * 57.296;       //Calculate the pitch angle.
    }
    if (abs(temp->ay) < temp->acc_total_vector) {                                      //Prevent the asin function to produce a NaN
      temp->angleRoll = asin((float)temp->ay / temp->acc_total_vector) * -57.296;       //Calculate the roll angle.
    }
  }
  else
  {
    temp->HardwareIssues = hardwareError((uint8_t)temp->HardwareIssues | GYRO); //Irgendwas stimmt mit hasi nicht
    debugPrint("Gyro not found!!\n");
  }
}

#endif
