#include "Variables.h"
struct vals *temp;

#include "Server_Context.h"
#include "Debugging.h"
#include "droneUtility.h"
#include "Sensor.h"

#include <Wire.h>
#include "FlySkyIBus.h"

void setup() {
  temp = new struct vals;
  if(!temp->debugging)
  {
    IBus.begin(Serial);       //Serielle Schnittstelle wird mit 115200 baud initialisiert
  }
  ///////////////////////////////////////////////////////
  setupServer();            //Server vorbereiten und starten -> Server.h
  SPIFFS.begin();           //Flash Speicher vorbereiten
  loadconfig();             //Konfigurationen aus Flash Speicher Laden
  ////////////////////////////////////////////////////
  /////////////////////////////////////////////Sensoroffsets berechnen
  SensorInit();  //MPU6050 kalibrieren. Drohne nicht bewegen!!
  ////////////////////////////Servo und 4 Motoren ansprechen
  if (!temp->debugging)
  {
#ifdef ESP8266
    pinMode(temp->hr, OUTPUT); //HR
    pinMode(temp->vr, OUTPUT); //VR
    pinMode(temp->hl, OUTPUT); //HL
    pinMode(temp->vl, OUTPUT); //VL
    pinMode(temp->camServo, OUTPUT); //CAMSERVO
    writePWM(temp->hr,1000);
    writePWM(temp->vr,1000);
    writePWM(temp->hl,1000);
    writePWM(temp->vl,1000);
    startWaveform(16, 1500, 20000 - 1500, 0); //50Hz
#endif
  }
  ////////////////////////////Erster Schleifendurchlauf festlegen
  if(temp->Frequenz < 250.0) temp->Frequenz = 250.0;
  temp->durchlaufT = (1.0 / temp->Frequenz) * 1000000.0;
}

void loop() {
  while (micros() < temp->nextloop)    //Definierte wiederholrate einhalten
  {
    yield(); //Feed the Watchdog
  }
  temp->nextloop = micros() + temp->durchlaufT; //nächster Schleifendurchlauf festlegen (in us)
  Sensor();
  if(!temp->debugging) Funk_Lesen();
  berechnen();
#if defined ESP8266
  if(temp->Arming < 1500 || temp->debugging) // Wenn motoren nicht gearmt sind dann kümmere dich nur um den Server
  {
    server.handleClient();
    MDNS.update();
    yield();
  }
#endif
  // debugLoop();
}
