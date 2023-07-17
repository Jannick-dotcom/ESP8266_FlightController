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

void debugPID() {
  debugPrint("Roll_Output\tPitch_Output\tYaw_Output\n");
  debugPrint(pid_output_roll);
  debugPrint("\t");
  debugPrint(pid_output_pitch);
  debugPrint("\t");
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
