#ifndef Debugging_h
#define Debugging_h

#include <Arduino.h>
#include "Variables.h"

void debugReceiver() {
  if (temp->debugging)
  {
    Serial.print("Throttle\tRoll\tPitch\tYaw\tArm\tMode\tCampitch\n");
    Serial.print(temp->Throttle);
    Serial.print("\t\t");
    Serial.print(temp->Roll);
    Serial.print("\t");
    Serial.print(temp->Pitch);
    Serial.print("\t");
    Serial.print(temp->Yaw);
    Serial.print("\t");
    Serial.print(temp->Arming);
    Serial.print("\t");
    Serial.print(temp->Mode);
    Serial.print("\t");
    Serial.print(temp->Campitch);
    Serial.print("\t\n");
  }
}

void debugPID() {
  if (temp->debugging)
  {
    Serial.print("Roll_Output\tPitch_Output\tYaw_Output\n");
    Serial.print(temp->pid_output_roll);
    Serial.print("\t");
    Serial.print(temp->pid_output_pitch);
    Serial.print("\t");
    Serial.print(temp->pid_output_yaw);
    Serial.print("\n");
  }
}

void debugLoop() {
  if (temp->debugging)
  {
    Serial.println("LOOP");
  }
}

#endif
