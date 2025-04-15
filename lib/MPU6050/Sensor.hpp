#ifndef Sensor_h
#define Sensor_h

#include "Variables.hpp"
#include "math.h"
#include <Wire.h>
#include "quaternion.hpp"

void MPU_getData(void)
{
  Wire.beginTransmission(0x68);                                // Start communication with the gyro.
  Wire.write(0x3B);                                            // Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                      // End the transmission.
  Wire.requestFrom(0x68, 14);                                  // Request 14 bytes from the gyro.
  uint64_t timer = micros64();                                 // Save the actual timestamp
  while (Wire.available() < 14 && (micros64() - timer < 10000)) // Wait until the 14 bytes are received. and check if the Gyro doesn´t response
  {
    // yield();
  }

  if (Wire.available() < 14) // If the Gyro didn´t response everything expected
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
  ax = Wire.read() << 8 | Wire.read();          // Add the low and high byte to the acc_x variable.
  ay = Wire.read() << 8 | Wire.read();          // Add the low and high byte to the acc_y variable.
  az = Wire.read() << 8 | Wire.read();          // Add the low and high byte to the acc_z variable.
  temperature = Wire.read() << 8 | Wire.read(); // Add the low and high byte to the temperature variable.
  gx = Wire.read() << 8 | Wire.read();          // Read high and low part of the angular data.
  gy = Wire.read() << 8 | Wire.read();          // Read high and low part of the angular data.
  g_z = Wire.read() << 8 | Wire.read();         // Read high and low part of the angular data.
}

bool ping_gyro(void)
{
  Wire.beginTransmission(0x68);
  Wire.write(0x75);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 1);
  uint64_t timer = millis() + 100;
  while (Wire.available() < 1 && timer > micros64()) // Wait till the Gyro responded with 1 Byte, or wait 100 ms... what happens earlier
  {
    // do nothing
  }
  if (Wire.read() == 0x68) // If the Delivered Byte is the hardware address then return success
  {
    return true;
  }
  else
  {
    return false;
  }
}

MadgwickAHRS filter(0.01f); // Lower beta = smoother, higher = more reactive

uint64_t lastSensor = 0;
void Sensor()
{
  MPU_getData();
  uint64_t currentTime = micros64();
  float deltaT = (currentTime - lastSensor) / 1000000.0;
  lastSensor = currentTime;

  gx = gx - gxC;
  gy = gy - gyC;
  g_z = g_z - gzC;

  // ax = ax - axC;    //Kalibrierwerte Beschl. Sensor
  // ay = ay - ayC;
  // az = az - azC;

  accX = float(ax / 4096.0);
  accY = float(ay / 4096.0);
  accZ = float(az / 4096.0);

  gyroX = float(gx / 65.5);
  gyroY = float(gy / 65.5);
  gyroZ = float(g_z / 65.5);

  // Gyro angle calculations
  // anglePitch += gyroY * deltaT; // Calculate the traveled pitch angle and add this to the angle_pitch variable.
  // angleRoll += gyroX * deltaT;  // Calculate the traveled roll angle and add this to the angle_roll variable.

  // getAccelAngles(accX, accY, accZ, accelPitch, accelRoll);
  // anglePitch -= angleRoll * sin((gyroZ * deltaT) * (M_PI / 180.0)); // If the IMU has yawed transfer the roll angle to the pitch angle.
  // angleRoll += anglePitch * sin((gyroZ * deltaT) * (M_PI / 180.0)); // If the IMU has yawed transfer the pitch angle to the roll angle.

  // MadgwickQuaternionUpdate(accX, accY, accZ, gyroX, gyroY, gyroZ, deltaT);
  
  float acc[3] = {accX, accY, accZ};  // Assuming the device is flat on the ground
  float gyro[3] = {DEG2RAD(gyroX), DEG2RAD(gyroY), DEG2RAD(gyroZ)};  // Small rotation around the y-axis
  
  filter.updateIMU(gyro[0], gyro[1], gyro[2], acc[0], acc[1], acc[2], deltaT);
  filter.getEuler(angleRoll, anglePitch, yawImu);
  // debugSensor();
}

void SensorInit()
{
  Wire.begin();
  Wire.setClock(400000); // I2C Frequenz auf 400kHz setzen
  debugPrint("Pinging Gyro\n");
  if (ping_gyro())
  {
    uint8_t success = false;
    debugPrint("Found Gyro\n");
    Wire.beginTransmission(0x68);     // Start communication with the address found during search.
    Wire.write(0x6B);                 // We want to write to the PWR_MGMT_1 register (6B hex)
    Wire.write(0x00);                 // Set the register bits as 00000000 to activate the gyro
    success = Wire.endTransmission(); // End the transmission with the gyro.
    delay(10);
    if (success != 0)
    {
      debugPrint("Failed to configure sensor1: ");
      debugPrint(success);
      debugPrint("\n");
      return;
    }
    // Configure the accelerometer (+/-8g)
    Wire.beginTransmission(0x68);     // Start communicating with the MPU-6050
    Wire.write(0x1C);                 // Send the requested starting register
    Wire.write(0x10);                 // Set the requested starting register
    success = Wire.endTransmission(); // End the transmission
    delay(10);
    if (success != 0)
    {
      debugPrint("Failed to configure sensor2: ");
      debugPrint(success);
      debugPrint("\n");
      return;
    }
    // Configure the gyro (500dps full scale)
    Wire.beginTransmission(0x68);     // Start communicating with the MPU-6050
    Wire.write(0x1B);                 // Send the requested starting register
    Wire.write(0x08);                 // Set the requested starting register
    success = Wire.endTransmission(); // End the transmission
    delay(10);
    if (success != 0)
    {
      debugPrint("Failed to configure sensor3: ");
      debugPrint(success);
      debugPrint("\n");
      return;
    }
    Wire.beginTransmission(0x68);     // Start communication with the address found during search
    Wire.write(0x1A);                 // We want to write to the CONFIG register (1A hex)
    Wire.write(0x03);                 // Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
    success = Wire.endTransmission(); // End the transmission with the gyro
    delay(10);
    if (success != 0)
    {
      debugPrint("Failed to configure sensor4: ");
      debugPrint(success);
      debugPrint("\n");
      return;
    }
    debugPrint("Configured Sensor\n");

    debugPrint("Calibration..\n");
    
    for (uint16_t i = 0; i < 12000; i++)
    { // Sensor kalibrieren
      MPU_getData();
      gxC += gx;
      gyC += gy;
      gzC += g_z;
      axC += ax;
      ayC += ay;
      azC += az;
      yield();
    }
    gxC /= 12000.0;
    gyC /= 12000.0;
    gzC /= 12000.0;
    axC /= 12000.0;
    ayC /= 12000.0;
  }
  else
  {
    HardwareIssues = hardwareError((uint8_t)HardwareIssues | GYRO); // Irgendwas stimmt mit hasi nicht
    debugPrint("Gyro not found!!\n");
  }
}

#endif
