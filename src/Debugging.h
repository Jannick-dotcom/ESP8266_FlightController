#ifndef Debugging_h
#define Debugging_h

#include <Arduino.h>
#include "Variables.h"
// #include <WebSerial.h>

template <typename T>
void debugPrint(T str)
{
  if(debugging) 
  Serial.print(str);
}

void debugReceiver() {
  debugPrint(Throttle);
  debugPrint("\t");
  debugPrint(Roll);
  debugPrint("\t");
  debugPrint(Pitch);
  debugPrint("\t");
  debugPrint(Yaw);
  debugPrint("\t");
  debugPrint(Arming);
  debugPrint("\t");
  debugPrint(Mode);
  debugPrint("\n");
}

void debugPWM()
{
  for(uint8_t i = 0; i < sizeof(esc)/ sizeof(esc[0]); i++)
  {
    debugPrint("esc[");
    debugPrint(i);
    debugPrint("]: ");
    debugPrint(esc[i]);
    debugPrint("\t");
  }
  debugPrint("\n");
}

void debugPID() {
  debugPrint("Roll_Output: ");
  debugPrint(pid_output_roll);
  debugPrint("\tPitch_Output: ");
  debugPrint(pid_output_pitch);
  debugPrint("\tYaw_Output: ");
  debugPrint(pid_output_yaw);
  debugPrint("\n");
}

void debugSensor()
{
  debugPrint("AnglePitch\tAngleRoll\tAngleYaw\n");
  debugPrint(anglePitch);
  debugPrint("\t");
  debugPrint(angleRoll);
  debugPrint("\t");
  debugPrint(gyroZ);
  debugPrint("\n");
}

void debugLoop() {
  debugPrint("LOOP");
}

#endif
