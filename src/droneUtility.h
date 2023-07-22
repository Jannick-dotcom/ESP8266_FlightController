#ifndef droneUtility_h
#define droneUtility_h

#include <Wire.h>
#include "Variables.h"
#include "IBusBM.h"
IBusBM IBus;
#include "Debugging.h"
#include "StallardOSPID.hpp"

#ifdef ESP8266
#include "core_esp8266_waveform.h" //Nützlich für PWM
#endif

void writePWM(uint8_t pin, uint16_t dur)
{
#ifdef ESP8266
  if (!debugging)
    startWaveform(pin, dur, 4000 - dur);
#endif
#ifdef ESP32

#endif
}

void resetDrone()
{
  debugPrint("RESET!");
  ESP.restart();
  yield();
}

uint16_t xx = 0;
void Funk_Lesen() {
  if(debugging) return;
  IBus.loop();
  for (uint8_t i = 0; i < 6; i++)
  {
    Received[i] = IBus.readChannel(i);
    if(abs(1500 - Received[i]) < 5)
    {
      Received[i] = 1500;
    }
    if (Received[i] < 1000 || Received[i] > 2000)  //Wenn kein Signal von Fernsteuerung
    {
      Arming = 1000;
      HardwareIssues = hardwareError((uint8_t)HardwareIssues | RECEIVER);
      return;
    }
    else
    {
      HardwareIssues = hardwareError((uint8_t)HardwareIssues & ~RECEIVER);
    }
  }
  Roll = Received[0];
  Pitch = Received[1];
  Throttle = Received[2];
  Yaw = Received[3];
  Arming = Received[4];
  Mode = Received[5];
  debugReceiver();
}

void MPU_getData(void) {
  if((HardwareIssues & GYRO) && debugging)
  {
    ax = rand() % (uint16_t)(pow(2,16) - 1);//0;
    ay = rand() % (uint16_t)(pow(2,16) - 1);
    az = rand() % (uint16_t)(pow(2,16) - 1);
    gx = rand() % (uint16_t)(pow(2,16) - 1);
    gy = rand() % (uint16_t)(pow(2,16) - 1);
    g_z = rand() % (uint16_t)(pow(2,16) - 1);
    return; 
  }
  Wire.beginTransmission(0x68);                                   //Start communication with the gyro.
  Wire.write(0x3B);                                               //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                         //End the transmission.
  Wire.requestFrom(0x68, 14);                                     //Request 14 bytes from the gyro.
  uint16_t timer = micros();                                      //Save the actual timestamp
  while (Wire.available() < 14 && (micros() - timer < durchlaufT))//Wait until the 14 bytes are received. and check if the Gyro doesn´t response
  {
    yield();
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

bool ping_gyro(void) {
  Wire.beginTransmission(0x68);
  Wire.write(0x75);
  Wire.endTransmission();
  Wire.requestFrom(0x68, 1);
  uint32_t timer = millis() + 100;
  while (Wire.available() < 1 && timer > millis())              //Wait till the Gyro responded with 1 Byte, or wait 100 ms... what happens earlier
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

void calculate_pid() {
  //Roll calculations
  float pid_error_temp;
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if (pid_i_mem_roll > pid_max) pid_i_mem_roll = pid_max;
  else if (pid_i_mem_roll < pid_max * -1) pid_i_mem_roll = pid_max * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if (pid_output_roll > pid_max) pid_output_roll = pid_max;
  else if (pid_output_roll < pid_max * -1) pid_output_roll = pid_max * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if (pid_i_mem_pitch > pid_max) pid_i_mem_pitch = pid_max;
  else if (pid_i_mem_pitch < pid_max * -1) pid_i_mem_pitch = pid_max * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if (pid_output_pitch > pid_max) pid_output_pitch = pid_max;
  else if (pid_output_pitch < pid_max * -1) pid_output_pitch = pid_max * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if (pid_i_mem_yaw > pid_max) pid_i_mem_yaw = pid_max;
  else if (pid_i_mem_yaw < pid_max * -1) pid_i_mem_yaw = pid_max * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if (pid_output_yaw > pid_max) pid_output_yaw = pid_max;
  else if (pid_output_yaw < pid_max * -1) pid_output_yaw = pid_max * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

void calculate_STOS_pid()
{
  pid_output_pitch = pitch.calculate_pid(pid_pitch_setpoint, gyro_pitch_input);
  pid_output_roll = roll.calculate_pid(pid_roll_setpoint, gyro_roll_input);
  pid_output_yaw = yaw.calculate_pid(pid_yaw_setpoint, gyro_yaw_input);
}

void berechnen() {
  if(controlMode == 1) return; //If checking the motors -> don't overwrite
  if (Arming < 1500)    //Wenn disarming
  {
    //PID´s zurücksetzen
    pid_i_mem_roll = 0;
    pid_last_roll_d_error = 0;
    pid_i_mem_pitch = 0;
    pid_last_pitch_d_error = 0;
    pid_i_mem_yaw = 0;
    pid_last_yaw_d_error = 0;
    pitch.reset();
    roll.reset();
    yaw.reset();
    //PWM für esc´s aktualisieren
    writePWM(hr, 1000);
    writePWM(vr, 1000);
    writePWM(hl, 1000);
    writePWM(vl, 1000);
  }
  else
  {
    Throttle = (((Throttle - 1000) / 1000.0) * (1000.0 - pid_max)) + 1000; //Ändern des maximalen Throttle punkts auf 1800 damit auch bei vollem schub noch manöver möglich sind

    pid_roll_setpoint = ((Roll - 1500) / 500.0) * degpersec; //Eingabe auf +-500 verschieben -> dann in °/s umrechnen
    pid_pitch_setpoint = ((Pitch - 1500) / 500.0) * degpersec; //Wird wahrscheinlich nicht mit autolevelmode funktionieren!!!!!!!!!!!!!!
    pid_yaw_setpoint = ((Yaw - 1500) / 500.0) * degpersec;

    if (Mode > 1300 && Mode <= 2000) {      //Autolevel; Funktioniert das??????????????????
      pid_roll_setpoint = Roll - (angleRoll * 15.0);
      pid_pitch_setpoint = Pitch - (anglePitch * 15.0);
    }

    gyro_roll_input = gyroX;
    gyro_pitch_input = gyroY;
    gyro_yaw_input = gyroZ;

    // calculate_pid(); //PID Werte für alle Achsen berechnen
    calculate_STOS_pid();

    esc[0] = Throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;//HR
    esc[1] = Throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;//VR
    esc[2] = Throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;//HL
    esc[3] = Throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;//VL

    for (uint8_t i = 0; i < sizeof(esc) / sizeof(esc[0]); i++)
    {
      if (esc[i] > 2000)        //Ausgang auf 2000 limitieren
      {
        esc[i] = 2000;
      }
      else if (esc[i] < minPulse)   //Motoren am laufen halten wenn minimaler schub
      {
        esc[i] = minPulse;
      }
    }
    debugPWM();

    //PWM für esc´s aktualisieren
    writePWM(hr, esc[0]);
    writePWM(vr, esc[1]);
    writePWM(hl, esc[2]);
    writePWM(vl, esc[3]);
    //writePWM(camServo,Campitch);
    // startWaveform(16, Campitch, 20000 - Campitch, 0);
  }
}

#endif
