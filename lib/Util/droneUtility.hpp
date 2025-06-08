#pragma once
#ifdef NATIVE
#include "mockTime.hpp"
#endif
// #include "Variables.hpp"

#ifdef ESP8266
#include "core_esp8266_waveform.h" //Nützlich für PWM
#endif

void writePWM(uint8_t pin, uint16_t dur);

#ifdef ESP8266
void resetDrone();
#endif

template <class T>
void constrainValue(T &val, T max, T min)
{
  if(val > max)
  {
    val = max;
  }
  else if(val < min)
  {
    val = min;
  }
  else
  {
    return;
  }
}