#ifndef Debugging_h
#define Debugging_h

#include <Arduino.h>
#include <sstream>
// #include <WebSerial.h>

template <typename T>
void debugPrint(T str)
{
  if(debugging) Serial.print(str);
  // std::stringstream ss;
  // ss << str;

  // terminalOutput += ss.str();
}

void debugReceiver() {
  for (uint8_t i = 0; i < 6; i++)
  {
    debugPrint(Received[i]);
    debugPrint("\t");
  }
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
  // debugPrint("AnglePitch\tAngleRoll\tAngleYaw\n");
  debugPrint(anglePitch);
  debugPrint("\t");
  debugPrint(angleRoll);
  debugPrint("\t");

  debugPrint(accX);
  debugPrint("\t"); 
  debugPrint(accY);
  debugPrint("\t");
  debugPrint(accZ);
  debugPrint("\t");

  // debugPrint(gyroX);
  // debugPrint("\t");
  // debugPrint(gyroY);
  // debugPrint("\t");
  // debugPrint(gyroZ);

  debugPrint("\n");
}

void debugLoop() {
  debugPrint("LOOP\n");
}

#endif
