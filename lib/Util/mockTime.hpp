#pragma once

#if !defined(ESP8266) && !defined(ESP32)

#include "stdint.h"

uint64_t micros64()
{
    return 0;
}
#endif