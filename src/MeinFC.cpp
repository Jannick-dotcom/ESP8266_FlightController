#include "Variables.h"

#include "Server_Context.h"
#include "Debugging.h"
#include "droneUtility.h"
#include "Sensor.h"

#include <Wire.h>
#include "IBusBM.h"

void setup() {
  //temp = new struct vals;
  if(!debugging)
  {
    IBus.begin(Serial);       //Serielle Schnittstelle wird mit 115200 baud initialisiert
  }
  else
  {
    Serial.begin(115200);
  }
  ///////////////////////////////////////////////////////
  setupServer();            //Server vorbereiten und starten -> Server_Context.h
  LittleFS.begin();           //Flash Speicher vorbereiten
  loadconfig();             //Konfigurationen aus Flash Speicher Laden
  ////////////////////////////////////////////////////
  /////////////////////////////////////////////Sensoroffsets berechnen
  SensorInit();  //MPU6050 kalibrieren. Drohne nicht bewegen!!
  ////////////////////////////Servo und 4 Motoren ansprechen
  if (!debugging)
  {
#ifdef ESP8266
    // pinMode(hr, OUTPUT); //HR
    // pinMode(vr, OUTPUT); //VR
    // pinMode(hl, OUTPUT); //HL
    // pinMode(vl, OUTPUT); //VL
    // pinMode(camServo, OUTPUT); //CAMSERVO
    writePWM(hr,2000); //Calibrate ESC's
    writePWM(vr,2000);
    writePWM(hl,2000);
    writePWM(vl,2000);
    delay(2000);
    writePWM(hr,1000);
    writePWM(vr,1000);
    writePWM(hl,1000);
    writePWM(vl,1000);
    // startWaveform(16, 1500, 20000 - 1500, 0); //50Hz
#endif
  }
  ////////////////////////////Erster Schleifendurchlauf festlegen
  if(Frequenz < 250.0) Frequenz = 250.0;
  durchlaufT = (1e6 / Frequenz);
}

void loop() {
  while (micros() < nextloop)    //Definierte wiederholrate einhalten
  {
  }
  nextloop = micros() + durchlaufT; //nächster Schleifendurchlauf festlegen (in us)
  Funk_Lesen();
  if(Arming < 1500 || debugging)
  {
    handleServer();
    yield(); //Feed the Watchdog
  }
  else
  {
    Sensor();
  }
  //berechnen();
  //timeNeeded = micros() - (nextloop - durchlaufT);
}
