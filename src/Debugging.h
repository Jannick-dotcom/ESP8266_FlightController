#ifndef Debugging_h
#define Debugging_h

#include <Arduino.h>
#include "Variables.h"
#include <WebSerial.h>

void debugPrint(float str);
void debugPrint(String str);
uint16_t sincelastupdate = 0;

void debugReceiver() {
  debugPrint(temp->Throttle);
  debugPrint(",");
  debugPrint(temp->Roll);
  debugPrint(",");
  debugPrint(temp->Pitch);
  debugPrint(",");
  debugPrint(temp->Yaw);
  debugPrint(",");
  debugPrint(temp->Arming);
  debugPrint(",");
  debugPrint(temp->Mode);
  debugPrint("\n");
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
  Serial.println(str);
}

void debugPrint(float str)
{
  Serial.println(str);
}

#endif
