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

void Funk_Lesen() {
  // if(debugging) return;
  IBus.loop();
  for (uint8_t i = 0; i < 6; i++)
  {
    Received[i] = IBus.readChannel(i);
    if(abs(1500 - Received[i]) < 5)
    {
      Received[i] = 1500;
    }
    if ((Received[i] < 1000 || Received[i] > 2000))  //Wenn kein Signal von Fernsteuerung
    {
      HardwareIssues = hardwareError((uint8_t)HardwareIssues | RECEIVER);
      Arming = 1000;
      return;
    }
    else if((Received[i] >= 1000 && Received[i] <= 2000))
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
  // debugReceiver();
}

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
    pitch.reset();
    roll.reset();
    yaw.reset();
    //PWM für esc´s aktualisieren
    writePWM(hr, 1000);
    writePWM(vr, 1000);
    writePWM(hl, 1000);
    writePWM(vl, 1000);
    return;
  }
  Throttle = (((Throttle - 1000) / 1000.0) * (1000.0 - pid_max)) + 1000; //Ändern des maximalen Throttle punkts auf 2000-pidMax damit auch bei vollem schub noch manöver möglich sind

  if (Mode > 1300 && Mode <= 2000) {      //Autolevel; Funktioniert das??????????????????
    float rollCorrection = (angleRoll * 300.0 / 5.0);
    float pitchCorrection = (anglePitch * 300.0 / 5.0);
    pid_roll_setpoint = Roll - 1500 - constrain(rollCorrection, -300, 300);
    pid_pitch_setpoint = Pitch - 1500 - constrain(pitchCorrection, -300, 300);
  }
  else
  {
    pid_roll_setpoint = ((Roll - 1500) / 500.0) * degpersec; //Eingabe auf +-500 verschieben -> dann in °/s umrechnen
    pid_pitch_setpoint = ((Pitch - 1500) / 500.0) * degpersec;
  }
  pid_yaw_setpoint = ((Yaw - 1500) / 500.0) * degpersec;

  gyro_roll_input = gyroX;
  gyro_pitch_input = gyroY;
  gyro_yaw_input = gyroZ;

  calculate_STOS_pid();

  esc[0] = Throttle + pid_output_pitch - pid_output_roll + pid_output_yaw;//HR
  esc[1] = Throttle - pid_output_pitch - pid_output_roll - pid_output_yaw;//VR
  esc[2] = Throttle + pid_output_pitch + pid_output_roll - pid_output_yaw;//HL
  esc[3] = Throttle - pid_output_pitch + pid_output_roll + pid_output_yaw;//VL

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
  // debugPWM();

  //PWM für esc´s aktualisieren
  writePWM(hr, esc[0]);
  writePWM(vr, esc[1]);
  writePWM(hl, esc[2]);
  writePWM(vl, esc[3]);
  //writePWM(camServo,Campitch);
  // startWaveform(16, Campitch, 20000 - Campitch, 0); // 50Hz für Servos
}

#endif
