#pragma once
#include "IBus/IBusBM.hpp"
#include "Variables.hpp"


IBusBM IBus;

void Funk_Lesen() {
  // if(debugging) return;
  IBus.loop();
  for (uint8_t i = 0; i < 6; i++)
  {
    Received[i] = IBus.readChannel(i);
    if(abs(1500 - Received[i]) < 5)
    {
      Received[i] = 1500;
    }
    if ((Received[i] < 1000 || Received[i] > 2000))  //Wenn kein Signal von Fernsteuerung
    {
      HardwareIssues = hardwareError((uint8_t)HardwareIssues | RECEIVER);
      Arming = 1000;
      return;
    }
    else if((Received[i] >= 1000 && Received[i] <= 2000))
    {
      HardwareIssues = hardwareError((uint8_t)HardwareIssues & ~RECEIVER);
    }
  }
  Roll = Received[0];
  Pitch = Received[1];
  Throttle = Received[2];
  Yaw = Received[3];
  Arming = Received[4];
  Mode = Received[5];
  // debugReceiver();
}