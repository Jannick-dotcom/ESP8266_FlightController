#pragma once

#include "StallardOSPID.hpp"
#include "Variables.hpp"
#include "droneUtility.hpp"

void calculate_STOS_pid()
{
  pid_output_pitch = pitch.calculate_pid(pid_pitch_setpoint, gyro_pitch_input);
  pid_output_roll = roll.calculate_pid(pid_roll_setpoint, gyro_roll_input);
  pid_output_yaw = yaw.calculate_pid(pid_yaw_setpoint, gyro_yaw_input);
}

template <class T>
T constrainValue(T val, uint32_t max, uint32_t min)
{
  if(val > max)
  {
    return max;
  }
  else if(val < min)
  {
    return min;
  }
  else
  {
    return val;
  }
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
    float rollCorrection = (angleRoll * 300.0 / 15.0);
    float pitchCorrection = (anglePitch * 300.0 / 15.0);
    pid_roll_setpoint = Roll - 1500 - constrainValue(rollCorrection, -300, 300);
    pid_pitch_setpoint = Pitch - 1500 - constrainValue(pitchCorrection, -300, 300);
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