#ifndef Debugging_h
#define Debugging_h

#include <Arduino.h>
#include "Variables.h"
#include <WebSerial.h>

void debugPrint(float str);
void debugPrint(String str);

void debugReceiver() {
  debugPrint("Throttle\tRoll\tPitch\tYaw\tArm\tMode\tCampitch\n");
  debugPrint(temp->Throttle);
  debugPrint("\t\t");
  debugPrint(temp->Roll);
  debugPrint("\t");
  debugPrint(temp->Pitch);
  debugPrint("\t");
  debugPrint(temp->Yaw);
  debugPrint("\t");
  debugPrint(temp->Arming);
  debugPrint("\t");
  debugPrint(temp->Mode);
  debugPrint("\t");
  debugPrint(temp->Campitch);
  debugPrint("\t\n");
}

void debugPID() {
  debugPrint("Roll_Output\tPitch_Output\tYaw_Output\n");
  debugPrint(temp->pid_output_roll);
  debugPrint("\t");
  debugPrint(temp->pid_output_pitch);
  debugPrint("\t");
  debugPrint(temp->pid_output_yaw);
  debugPrint("\n");
}

void debugSensor()
{
  debugPrint("AnglePitch\tAngleRoll\tAngleYaw\n");
  debugPrint(temp->anglePitch);
  debugPrint("\t");
  debugPrint(temp->angleRoll);
  debugPrint("\t");
  debugPrint(temp->gyroZ);
  debugPrint("\n");
}

void debugLoop() {
  debugPrint("LOOP");
}

void debugPrint(String str)
{
  if(temp->debugging)
  {
    WebSerial.println(str);
  }
}

void debugPrint(float str)
{
  if(temp->debugging)
  {
    WebSerial.println(str);
  }
}

#endif
