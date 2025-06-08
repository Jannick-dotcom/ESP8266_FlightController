#include "droneUtility.hpp"

void writePWM(uint8_t pin, uint16_t dur)
{
#ifdef ESP8266
  startWaveform(pin, dur, 4000 - dur);
#endif
#ifdef ESP32

#endif
}

#ifdef ESP8266
void resetDrone()
{
  ESP.restart();
  yield();
}
#endif