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

#endif
