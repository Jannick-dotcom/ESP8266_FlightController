#include "Variables.h"

#include "Server_Context.h"
#include "Debugging.h"
#include "droneUtility.h"
#include "Sensor.h"

#include <Wire.h>
#include "IBusBM.h"

void setup() {
  IBus.begin(Serial, IBUSBM_NOTIMER);       //Serielle Schnittstelle wird mit 115200 baud initialisiert
  ///////////////////////////////////////////////////////
  setupServer();            //Server vorbereiten und starten -> Server_Context.h
  LittleFS.begin();           //Flash Speicher vorbereiten
  loadconfig();             //Konfigurationen aus Flash Speicher Laden
  ////////////////////////////////////////////////////
  // pinMode(16, OUTPUT); //HR
  if (!debugging)
  {
#ifdef ESP8266
    pinMode(hr, OUTPUT); //HR
    pinMode(vr, OUTPUT); //VR
    pinMode(hl, OUTPUT); //HL
    pinMode(vl, OUTPUT); //VL
    // writePWM(hr,2000); //Calibrate ESC's
    // writePWM(vr,2000);
    // writePWM(hl,2000);
    // writePWM(vl,2000);
    delay(10000);
    writePWM(hr,1000);
    writePWM(vr,1000);
    writePWM(hl,1000);
    writePWM(vl,1000);
#endif
  }
  /////////////////////////////////////////////Sensoroffsets berechnen
  SensorInit();  //MPU6050 kalibrieren. Drohne nicht bewegen!!
  ////////////////////////////Servo und 4 Motoren ansprechen
  ////////////////////////////Erster Schleifendurchlauf festlegen

  if(Frequenz == 0)
  {
    Frequenz = 250;
  }
  durchlaufT = (1e6 / Frequenz);
}

void loop() {
  while (micros64() < nextloop)    //Definierte wiederholrate einhalten
  {
  }
  nextloop = micros64() + durchlaufT; //nächster Schleifendurchlauf festlegen (in us)
  Funk_Lesen();
  if(Arming < 1500 || debugging)
  {
    handleServer();
    yield(); //Feed the Watchdog
  }
  Sensor();
  berechnen();
}