#ifndef droneUtility_h
#define droneUtility_h

#include <Wire.h>
#include "Variables.h"
#include "FlySkyIBus.h"
#include "Debugging.h"

#ifdef ESP8266
#include "core_esp8266_waveform.h" //Nützlich für PWM
#endif

void writePWM(uint8_t pin, uint16_t dur)
{
#ifdef ESP8266
  startWaveform(pin, dur, 4000 - dur);
#endif
#ifdef ESP32

#endif

}

void Funk_Lesen() {
  IBus.loop();
  for (uint8_t i = 0; i < 10; i++)
  {
    temp->Received[i] = IBus.readChannel(i);
  }

  temp->Roll = temp->Received[0];
  temp->Pitch = temp->Received[1];
  temp->Throttle = temp->Received[2];
  temp->Yaw = temp->Received[3];
  temp->Arming = temp->Received[4];
  temp->Campitch = temp->Received[9];
  temp->Mode = temp->Received[6];

  for (int i = 0; i < 10; i++)
  {
    if (temp->Received[i] < 1000 || temp->Received[i] > 2000)  //Wenn kein Signal von Fernsteuerung
    {
      temp->Arming = 1000;
      temp->HardwareIssues = 3;
      break;
    }
    else
    {
      if (temp->debugging)
      {
        debugReceiver();
      }
    }
  }
}

void MPU_getData(void) {
  Wire.beginTransmission(0x68);                                   //Start communication with the gyro.
  Wire.write(0x3B);                                               //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                         //End the transmission.
  Wire.requestFrom(0x68, 14);                                     //Request 14 bytes from the gyro.
  uint16_t timer = micros();                                      //Save the actual timestamp
  while (Wire.available() < 14 && (micros() - timer < temp->durchlaufT))//Wait until the 14 bytes are received. and check if the Gyro doesn´t response
  {
    yield();
  }

  if (Wire.available() < 14)                                     //If the Gyro didn´t response everything expected
  {
    temp->ax = 0;
    temp->ay = 0;
    temp->az = 0;
    temp->gx = 0;
    temp->gy = 0;
    temp->gz = 0;
    return;
  }

  temp->ax = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_x variable.
  temp->ay = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_y variable.
  temp->az = Wire.read() << 8 | Wire.read();                           //Add the low and high byte to the acc_z variable.
  temp->temperature = Wire.read() << 8 | Wire.read();                  //Add the low and high byte to the temperature variable.
  temp->gx = Wire.read() << 8 | Wire.read();                           //Read high and low part of the angular data.
  temp->gy = Wire.read() << 8 | Wire.read();                           //Read high and low part of the angular data.
  temp->gz = Wire.read() << 8 | Wire.read();                           //Read high and low part of the angular data.
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
  temp->pid_error_temp = temp->gyro_roll_input - temp->pid_roll_setpoint;
  temp->pid_i_mem_roll += temp->pid_i_gain_roll * temp->pid_error_temp;
  if (temp->pid_i_mem_roll > temp->pid_max) temp->pid_i_mem_roll = temp->pid_max;
  else if (temp->pid_i_mem_roll < temp->pid_max * -1) temp->pid_i_mem_roll = temp->pid_max * -1;

  temp->pid_output_roll = temp->pid_p_gain_roll * temp->pid_error_temp + temp->pid_i_mem_roll + temp->pid_d_gain_roll * (temp->pid_error_temp - temp->pid_last_roll_d_error);
  if (temp->pid_output_roll > temp->pid_max) temp->pid_output_roll = temp->pid_max;
  else if (temp->pid_output_roll < temp->pid_max * -1) temp->pid_output_roll = temp->pid_max * -1;

  temp->pid_last_roll_d_error = temp->pid_error_temp;

  //Pitch calculations
  temp->pid_error_temp = temp->gyro_pitch_input - temp->pid_pitch_setpoint;
  temp->pid_i_mem_pitch += temp->pid_i_gain_pitch * temp->pid_error_temp;
  if (temp->pid_i_mem_pitch > temp->pid_max) temp->pid_i_mem_pitch = temp->pid_max;
  else if (temp->pid_i_mem_pitch < temp->pid_max * -1) temp->pid_i_mem_pitch = temp->pid_max * -1;

  temp->pid_output_pitch = temp->pid_p_gain_pitch * temp->pid_error_temp + temp->pid_i_mem_pitch + temp->pid_d_gain_pitch * (temp->pid_error_temp - temp->pid_last_pitch_d_error);
  if (temp->pid_output_pitch > temp->pid_max) temp->pid_output_pitch = temp->pid_max;
  else if (temp->pid_output_pitch < temp->pid_max * -1) temp->pid_output_pitch = temp->pid_max * -1;

  temp->pid_last_pitch_d_error = temp->pid_error_temp;

  //Yaw calculations
  temp->pid_error_temp = temp->gyro_yaw_input - temp->pid_yaw_setpoint;
  temp->pid_i_mem_yaw += temp->pid_i_gain_yaw * temp->pid_error_temp;
  if (temp->pid_i_mem_yaw > temp->pid_max) temp->pid_i_mem_yaw = temp->pid_max;
  else if (temp->pid_i_mem_yaw < temp->pid_max * -1) temp->pid_i_mem_yaw = temp->pid_max * -1;

  temp->pid_output_yaw = temp->pid_p_gain_yaw * temp->pid_error_temp + temp->pid_i_mem_yaw + temp->pid_d_gain_yaw * (temp->pid_error_temp - temp->pid_last_yaw_d_error);
  if (temp->pid_output_yaw > temp->pid_max) temp->pid_output_yaw = temp->pid_max;
  else if (temp->pid_output_yaw < temp->pid_max * -1) temp->pid_output_yaw = temp->pid_max * -1;

  temp->pid_last_yaw_d_error = temp->pid_error_temp;

  if (temp->debugging)
  {
    debugPID();
  }
}

void berechnen() {
  if (temp->Arming < 1500)    //Wenn disarming
  {
    //PID´s zurücksetzen
    temp->pid_i_mem_roll = 0;
    temp->pid_last_roll_d_error = 0;
    temp->pid_i_mem_pitch = 0;
    temp->pid_last_pitch_d_error = 0;
    temp->pid_i_mem_yaw = 0;
    temp->pid_last_yaw_d_error = 0;
    //PWM für temp->esc´s aktualisieren
    if (!temp->debugging) //Wenn debugmodus aktiv ist keine PWM´s ausgeben
    {
#ifdef ESP8266
      writePWM(temp->hr, 1000);
      writePWM(temp->vr, 1000);
      writePWM(temp->hl, 1000);
      writePWM(temp->vl, 1000);
    }
#endif
  }
  else
  {
    temp->Throttle = (((temp->Throttle - 1000) / 1000.0) * (1000.0 - temp->pid_max)) + 1000; //Ändern des maximalen Throttle punkts auf 1800 damit auch bei vollem schub noch manöver möglich sind

    temp->Roll = (temp->Roll - 1500) / 500.0 * temp->degpersec; //Eingabe auf +-500 verschieben -> dann in °/s umrechnen
    temp->Pitch = (temp->Pitch - 1500) / 500.0 * temp->degpersec; //Wird wahrscheinlich nicht mit autolevelmode funktionieren!!!!!!!!!!!!!!
    temp->Yaw = (temp->Yaw - 1500) / 500.0 * temp->degpersec;

    if (temp->Mode > 1300 && temp->Mode <= 2000) {      //Autolevel; Funktioniert das??????????????????
      temp->Roll = temp->Roll - (temp->angleRoll * 15.0);
      temp->Pitch = temp->Pitch - (temp->anglePitch * 15.0);
    }

    temp->gyro_roll_input = temp->gyroX;
    temp->gyro_pitch_input = temp->gyroY;
    temp->gyro_yaw_input = temp->gyroZ;

    temp->pid_roll_setpoint = temp->Roll;
    temp->pid_pitch_setpoint = temp->Pitch;
    temp->pid_yaw_setpoint = temp->Yaw;

    calculate_pid(); //PID Werte für alle Achsen berechnen

    temp->esc[0] = temp->Throttle + temp->pid_output_pitch + temp->pid_output_roll + temp->pid_output_yaw;//HR
    temp->esc[1] = temp->Throttle - temp->pid_output_pitch + temp->pid_output_roll - temp->pid_output_yaw;//VR
    temp->esc[2] = temp->Throttle + temp->pid_output_pitch - temp->pid_output_roll - temp->pid_output_yaw;//HL
    temp->esc[3] = temp->Throttle - temp->pid_output_pitch - temp->pid_output_roll + temp->pid_output_yaw;//VL

    for (uint8_t i = 0; i < 4; i++)
    {
      if (temp->esc[i] > 2000)        //Ausgang auf 2000 limitieren
      {
        temp->esc[i] = 2000;
      }
      else if (temp->esc[i] < temp->minPulse)   //Motoren am laufen halten wenn minimaler schub
      {
        temp->esc[i] = temp->minPulse;
      }
    }
    if (!temp->debugging)
    {
      //PWM für temp->esc´s aktualisieren
#ifdef ESP8266
      writePWM(temp->hr, temp->esc[0]);
      writePWM(temp->vr, temp->esc[1]);
      writePWM(temp->hl, temp->esc[2]);
      writePWM(temp->vl, temp->esc[3]);
      //writePWM(camServo,Campitch);
      // startWaveform(16, Campitch, 20000 - Campitch, 0);
#endif
    }
  }
}

#endif
