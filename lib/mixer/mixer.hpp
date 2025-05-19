#pragma once

#include "StallardOSPID.hpp"
#include "Variables.hpp"
#include "droneUtility.hpp"

uint64_t lastPid = 0;
void calculate_STOS_pid()
{
  uint64_t currentTime = micros64(); 
  float deltaT = (currentTime - lastPid)/1000000.0;
  lastPid = currentTime;
  pid_output_pitch = pitch.calculate_pid(pid_pitch_setpoint, gyro_pitch_input, deltaT);
  pid_output_roll = roll.calculate_pid(pid_roll_setpoint, gyro_roll_input, deltaT);
  pid_output_yaw = yaw.calculate_pid(pid_yaw_setpoint, gyro_yaw_input, deltaT);
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
  // Throttle = (((Throttle - 1000) / 1000.0) * (1000.0 - pid_max)) + 1000; //Ändern des maximalen Throttle punkts auf 2000-pidMax damit auch bei vollem schub noch manöver möglich sind
  Throttle = (((Throttle - 1000.0) / 1000.0) * ((2000.0 - pid_max) - minPulse)) + minPulse; //Ändern des maximalen Throttle punkts auf 2000-pidMax damit auch bei vollem schub noch manöver möglich sind

  if (Mode > 1300 && Mode <= 2000) {      //Autolevel; Funktioniert das??????????????????
    float rollCorrection = (-angleRoll * 15.0);
    float pitchCorrection = (anglePitch * 15.0);
    pid_roll_setpoint = (Roll - 1500) - rollCorrection;
    pid_pitch_setpoint = (Pitch - 1500) - pitchCorrection;
  }
  else
  {
    pid_roll_setpoint = ((Roll - 1500) / 500.0) * degpersec; //Eingabe auf +-500 verschieben -> dann in °/s umrechnen
    pid_pitch_setpoint = ((Pitch - 1500) / 500.0) * degpersec;
  }
  pid_yaw_setpoint = ((Yaw - 1500) / 500.0) * degpersec;

  gyro_roll_input = -gyroX;
  gyro_pitch_input = gyroY;
  gyro_yaw_input = -gyroZ;

  calculate_STOS_pid();
  // debugPID();
  esc[0] = Throttle - pid_output_pitch - pid_output_roll + pid_output_yaw;//HR
  esc[1] = Throttle + pid_output_pitch - pid_output_roll - pid_output_yaw;//VR
  esc[2] = Throttle - pid_output_pitch + pid_output_roll - pid_output_yaw;//HL
  esc[3] = Throttle + pid_output_pitch + pid_output_roll + pid_output_yaw;//VL

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

  //PWM für esc´s aktualisieren
  writePWM(hr, esc[0]);
  writePWM(vr, esc[1]);
  writePWM(hl, esc[2]);
  writePWM(vl, esc[3]);

  //writePWM(camServo,Campitch);
  // startWaveform(16, Campitch, 20000 - Campitch, 0); // 50Hz für Servos
}