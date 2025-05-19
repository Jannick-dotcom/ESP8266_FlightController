#ifndef droneUtility_h
#define droneUtility_h
#ifndef NATIVE
#include <Wire.h>
#include "Debugging.hpp"
#else
#include "mockTime.hpp"
#endif
#include "Variables.hpp"

#ifdef ESP8266
#include "core_esp8266_waveform.h" //Nützlich für PWM
#endif

void writePWM(uint8_t pin, uint16_t dur)
{
#ifdef ESP8266
  if (!debugging)
    startWaveform(pin, dur, 4000 - dur);
#endif
#ifdef ESP32

#endif
}

#ifdef ESP8266
void resetDrone()
{
  debugPrint("RESET!");
  ESP.restart();
  yield();
}
#endif

template <class T>
void constrainValue(T &val, uint32_t max, uint32_t min)
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
#endif
