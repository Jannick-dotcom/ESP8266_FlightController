#ifndef ESP_SERVER_H
#define ESP_SERVER_H

#include "droneUtility.h"
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include "Variables.h"

const String inBreite = "200";
const String inHoehe = "20";

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "core_esp8266_waveform.h" //Nützlich für PWM
#include <WiFiUdp.h>
#include <ESP8266mDNS.h>
#include "LittleFS.h"
ESP8266WebServer server(80);  //Auf Port 80
#endif

#ifdef ESP32
#include <WiFi.h>
#include <Update.h>
#include <WebServer.h>
#include "LittleFS.h"
#include <ESPmDNS.h>
WebServer server(80);  //Auf Port 80
#endif

void saveConfig()
{
  StaticJsonDocument<800> json;
  if (LittleFS.exists("config.json"))
  {
    LittleFS.remove("/config.json");
  }
  File configFile = LittleFS.open("/config.json", "w");

  json["Frequenz"] = temp->Frequenz; //Durchlauffrequenz
  json["degpersec"] = temp->degpersec; //Maximale Rotationsgeschwindigkeit im Rate Modus
  json["pid_max"] = temp->pid_max;     //Maximaler PID wert

  json["vl"] = temp->vl; //Pin des Vorderen linken Motors
  json["vr"] = temp->vr; //Pin des Vorderen rechten Motors
  json["hl"] = temp->hl; //Pin des Hinteren linken Motors
  json["hr"] = temp->hr; //Pin des Hinteren rechten Motors

  json["pid_p_gain_roll"] = temp->pid_p_gain_roll;              //Gain setting for the roll P-controller.
  json["pid_i_gain_roll"] = temp->pid_i_gain_roll * temp->Frequenz;              //Gain setting for the roll I-controller.
  json["pid_d_gain_roll"] = temp->pid_d_gain_roll * temp->Frequenz;              //Gain setting for the roll D-controller.

  json["pid_p_gain_pitch"] = temp->pid_p_gain_pitch;            //Gain setting for the pitch P-controller.
  json["pid_i_gain_pitch"] = temp->pid_i_gain_pitch * temp->Frequenz;            //Gain setting for the pitch I-controller.
  json["pid_d_gain_pitch"] = temp->pid_d_gain_pitch * temp->Frequenz;            //Gain setting for the pitch D-controller.

  json["pid_p_gain_yaw"] = temp->pid_p_gain_yaw;                //Gain setting for the pitch P-controller.
  json["pid_i_gain_yaw"] = temp->pid_i_gain_yaw * temp->Frequenz;                //Gain setting for the pitch I-controller.
  json["pid_d_gain_yaw"] = temp->pid_d_gain_yaw * temp->Frequenz;                //Gain setting for the pitch D-controller.

  serializeJson(json, configFile);
  configFile.close();
}

bool loadconfig() {
  if (LittleFS.exists("/config.json")) {
    File configFile = LittleFS.open("/config.json", "r");
    StaticJsonDocument<800> json;
    deserializeJson(json, configFile);
    configFile.close();
    temp->Frequenz = json["Frequenz"]; //Frequenz
    temp->degpersec = json["degpersec"]; //Maximale Rotationsgeschwindigkeit im Rate Modus
    temp->pid_max = json["pid_max"];                //Maximum output of the PID-controller (+/-)

    temp->vl = json["vl"]; //Pin des Vorderen linken Motors
    temp->vr = json["vr"]; //Pin des Vorderen rechten Motors
    temp->hl = json["hl"]; //Pin des Hinteren linken Motors
    temp->hr = json["hr"]; //Pin des Hinteren rechten Motors

    temp->pid_p_gain_roll = json["pid_p_gain_roll"];    //Gain setting for the roll P-controller
    temp->pid_i_gain_roll = json["pid_i_gain_roll"];    //Gain setting for the roll I-controller
    temp->pid_d_gain_roll = json["pid_d_gain_roll"];    //Gain setting for the roll D-controller

    temp->pid_p_gain_pitch = json["pid_p_gain_pitch"];  //Gain setting for the pitch P-controller.
    temp->pid_i_gain_pitch = json["pid_i_gain_pitch"];  //Gain setting for the pitch I-controller.
    temp->pid_d_gain_pitch = json["pid_d_gain_pitch"];  //Gain setting for the pitch D-controller.

    temp->pid_p_gain_yaw = json["pid_p_gain_yaw"];      //Gain setting for the pitch P-controller.
    temp->pid_i_gain_yaw = json["pid_i_gain_yaw"];      //Gain setting for the pitch I-controller.
    temp->pid_d_gain_yaw = json["pid_d_gain_yaw"];      //Gain setting for the pitch D-controller.

    temp->pid_p_gain_roll = temp->pid_p_gain_roll;
    temp->pid_i_gain_roll = temp->pid_i_gain_roll / temp->Frequenz;
    temp->pid_d_gain_roll = temp->pid_d_gain_roll / temp->Frequenz;

    temp->pid_p_gain_pitch = temp->pid_p_gain_pitch;
    temp->pid_i_gain_pitch = temp->pid_i_gain_pitch / temp->Frequenz;
    temp->pid_d_gain_pitch = temp->pid_d_gain_pitch / temp->Frequenz;

    temp->pid_p_gain_yaw = temp->pid_p_gain_yaw;
    temp->pid_i_gain_yaw = temp->pid_i_gain_yaw / temp->Frequenz;
    temp->pid_d_gain_yaw = temp->pid_d_gain_yaw / temp->Frequenz;

    temp->pitch = StallardosPID(temp->pid_p_gain_pitch, temp->pid_i_gain_pitch, temp->pid_d_gain_pitch);
    temp->roll = StallardosPID(temp->pid_p_gain_roll, temp->pid_i_gain_roll, temp->pid_d_gain_roll);
    temp->yaw = StallardosPID(temp->pid_p_gain_yaw, temp->pid_i_gain_yaw, temp->pid_d_gain_yaw);
    return 0;
  }
  else
  {
    temp->HardwareIssues = 2; //First time of Boot or Spiffs broken
    LittleFS.format();
    saveConfig();
    return 1;
  }
}

void toString(double value, char *buffer)
{
  dtostrf(value, 4, 3, buffer);
}

void setupDrone()
{
  temp->controlMode = 0;
  String webpage;
  char value[20];
  webpage =  "<html>";
  webpage += "<head><title>ESP8266 Flight Controller</title>";
  webpage += "<style> body { margin:0 auto; background-color: #000000; font-size:40px; font-family: Arial, Helvetica, Sans-Serif; Color: white; height: 90%; width: 90%}";
  webpage += "</style>";
  webpage += "</head>";
  webpage += "<body>";
  webpage += "<h1>Flight Controller</h1>";
  webpage += "<table style='width:100%'> <tr>";
  // webpage += "<input type='number' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(temp->timeNeeded) +"'>";
  webpage += "<form action='/'><input type='submit' value='Back'></form>";
  webpage += "<form action='/save' method='POST'>";
  toString(temp->hl, value);
  webpage += "<tr><td>RearLeft Pin:</td><td><input type='number' step='1' name='RearLeft' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='0' max='20'></td></tr>";
  toString(temp->hr, value);
  webpage += "<tr><td>RearRight Pin:</td><td><input type='number' step='1' name='RearRight' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='0' max='20'></td></tr>";
  toString(temp->vl, value);
  webpage += "<tr><td>FrontLeft Pin:</td><td><input type='number' step='1' name='FrontLeft' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='0' max='20'></td></tr>";
  toString(temp->vr, value);
  webpage += "<tr><td>FrontRight Pin:</td><td><input type='number' step='1' name='FrontRight' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='0' max='20'></td></tr>";
  webpage += "</table>";
  webpage += "<input type='submit' value='Senden'></form>";
  webpage += "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update' accept='.bin'><input type='submit' value='Update'></form>";
  webpage += "<form method='GET' action='http://drone.local:8080/webserial'><input type='submit' value='Webserial'></form>";
  webpage += "<form method='GET' action='/check'><input type='submit' value='Check Motors'></form>";
  webpage += "<form method='POST' action='/reset'><input type='submit' value='RESET'></form>";
  webpage += "</body>";
  webpage += "</html>";
  server.send(200, "text/html", webpage); // Send a response to the client asking for input
}

void root()
{
  if(loadconfig()) //Wenn das erste mal auf den Server zugegriffen wird das setup aufrufen
  {
    setupDrone();
    return;
  }
  temp->controlMode = 0;
  String webpage;
  char value[20];
  webpage =  "<html>";
  webpage += "<head><title>ESP8266 Flight Controller</title>";
  webpage += "<style> body { margin:0 auto; background-color: #000000; font-size:40px; font-family: Arial, Helvetica, Sans-Serif; Color: white; height: 90%; width: 90%}";
  webpage += "</style>";
  webpage += "</head>";
  webpage += "<body>";
  webpage += "<h1>Flight Controller</h1>";
  if(temp->HardwareIssues > 0)
  {
    webpage += "<h4>Errorcode: " + String(temp->HardwareIssues) + "</h2>";
  }
  webpage += "<table style='width:100%'> <tr>";
  webpage += "<tr><form method='GET' action='/setup'><input type='submit' value='Setup der Drohne aufrufen'></form></tr>";

  webpage += "<form action='/save' method='POST'>";
  toString(temp->Frequenz, value);
  webpage += "<tr><td>Frequenz[Hz]:</td><td><input type='number' step='0.1' name='Frequenz' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='50' max='4000'></td></tr>";
  toString(temp->degpersec, value);
  webpage += "<tr><td>degpersec:</td><td><input type='number' step='0.001' name='degpersec' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='0.001' max='400'></td></tr>";
  toString(temp->pid_max, value);
  webpage += "<tr><td>pid_max:</td><td><input type='number' step='0.001' name='pid_max' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='0.001' max='300'></td></tr>";

  toString(temp->pid_p_gain_roll, value);
  webpage += "<tr><td>pid_p_gain_roll:</td><td><input type='number' step='0.001' name='pid_p_gain_roll' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='0'></td></tr>";
  toString(temp->pid_i_gain_roll, value);
  webpage += "<tr><td>pid_i_gain_roll:</td><td><input type='number' step='0.001' name='pid_i_gain_roll' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='0'></td></tr>";
  toString(temp->pid_d_gain_roll, value);
  webpage += "<tr><td>pid_d_gain_roll:</td><td><input type='number' step='0.001' name='pid_d_gain_roll' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='0'></td></tr>";

  toString(temp->pid_p_gain_pitch, value);
  webpage += "<tr><td>pid_p_gain_pitch:</td><td><input type='number' step='0.001' name='pid_p_gain_pitch' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='0'></td></tr>";
  toString(temp->pid_i_gain_pitch, value);
  webpage += "<tr><td>pid_i_gain_pitch:</td><td><input type='number' step='0.001' name='pid_i_gain_pitch' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='0'></td></tr>";
  toString(temp->pid_d_gain_pitch, value);
  webpage += "<tr><td>pid_d_gain_pitch:</td><td><input type='number' step='0.001' name='pid_d_gain_pitch' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='0'></td></tr>";

  toString(temp->pid_p_gain_yaw, value);
  webpage += "<tr><td>pid_p_gain_yaw:</td><td><input type='number' step='0.001' name='pid_p_gain_yaw' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='0'></td></tr>";
  toString(temp->pid_i_gain_yaw, value);
  webpage += "<tr><td>pid_i_gain_yaw:</td><td><input type='number' step='0.001' name='pid_i_gain_yaw' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='0'></td></tr>";
  toString(temp->pid_d_gain_yaw, value);
  webpage += "<tr><td>pid_d_gain_yaw:</td><td><input type='number' step='0.001' name='pid_d_gain_yaw' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='0'></td></tr>";
  webpage += "</table>";

  webpage += "<input type='submit' value='Senden'></form>";
  webpage += "</body>";
  webpage += "</html>";
  server.send(200, "text/html", webpage); // Send a response to the client asking for input
}

void handleSave() {
  temp->controlMode = 0;
  if (server.args() > 0 ) { // Arguments were received
    for ( uint8_t i = 0; i < server.args(); i++ ) {
      if (server.argName(i) == "Frequenz")
      {
        temp->Frequenz = server.arg(i).toFloat();
        temp->durchlaufT = (1000000.0 / temp->Frequenz);
      }
      else if (server.argName(i) == "degpersec")
      {
        temp->degpersec = server.arg(i).toFloat();
      }
      else if (server.argName(i) == "pid_max")
      {
        temp->pid_max = server.arg(i).toFloat();
      }

      else if (server.argName(i) == "FrontLeft")
      {
        temp->vl = server.arg(i).toInt();
      }
      else if (server.argName(i) == "FrontRight")
      {
        temp->vr = server.arg(i).toInt();
      }
      else if (server.argName(i) == "RearLeft")
      {
        temp->hl = server.arg(i).toInt();
      }
      else if (server.argName(i) == "RearRight")
      {
        temp->hr = server.arg(i).toInt();
      }

      else if (server.argName(i) == "pid_p_gain_roll")
      {
        temp->pid_p_gain_roll = server.arg(i).toFloat();
      }
      else if (server.argName(i) == "pid_i_gain_roll")
      {
        temp->pid_i_gain_roll = server.arg(i).toFloat();
      }
      else if (server.argName(i) == "pid_d_gain_roll")
      {
        temp->pid_d_gain_roll = server.arg(i).toFloat();
      }
      else if (server.argName(i) == "pid_p_gain_pitch")
      {
        temp->pid_p_gain_pitch = server.arg(i).toFloat();
      }
      else if (server.argName(i) == "pid_i_gain_pitch")
      {
        temp->pid_i_gain_pitch = server.arg(i).toFloat();
      }
      else if (server.argName(i) == "pid_d_gain_pitch")
      {
        temp->pid_d_gain_pitch = server.arg(i).toFloat();
      }
      else if (server.argName(i) == "pid_p_gain_yaw")
      {
        temp->pid_p_gain_yaw = server.arg(i).toFloat();
      }
      else if (server.argName(i) == "pid_i_gain_yaw")
      {
        temp->pid_i_gain_yaw = server.arg(i).toFloat();
      }
      else if (server.argName(i) == "pid_d_gain_yaw")
      {
        temp->pid_d_gain_yaw = server.arg(i).toFloat();
      }
    }
    saveConfig();
    temp->pitch = StallardosPID(temp->pid_p_gain_pitch, temp->pid_i_gain_pitch, temp->pid_d_gain_pitch);
    temp->roll = StallardosPID(temp->pid_p_gain_roll, temp->pid_i_gain_roll, temp->pid_d_gain_roll);
    temp->yaw = StallardosPID(temp->pid_p_gain_yaw, temp->pid_i_gain_yaw, temp->pid_d_gain_yaw);
    String webpage;
    webpage += "<META HTTP-EQUIV='Refresh' CONTENT='1; URL=/'>";
    webpage += "<html><style> body { margin:50px auto; background-color: #000000; font-size:60px; font-family: Arial, Helvetica, Sans-Serif; Color: white; height: 90%; width: 90%}";
    webpage += "</style><body>Wurde gespeichert!</body></html>";
    server.send(200, "text/html", webpage);
  }
}

void handleCheck() {
  if (server.args() > 0 ) // Arguments were received
  { 
    String motor = "None";
    uint8_t perc = 0;
    for ( uint8_t i = 0; i < server.args(); i++ ) 
    {
      if(server.argName(i) == "Motor")
      {
        motor = server.arg(i);
      }
      else if(server.argName(i) == "Throttle")
      {
        perc = server.arg(i).toInt();
      }
    }

    if(motor != "None" && perc >= 0 && perc <= 100)
    {
      temp->controlMode = 1;
      if(motor == "FL")
      {
        writePWM(temp->vl, 10 * perc + 1000);
      }
      else if(motor == "FR")
      {
        writePWM(temp->vr, 10 * perc + 1000);
      }
      else if(motor == "RR")
      {
        writePWM(temp->hr, 10 * perc + 1000);
      }
      else if(motor == "RL")
      {
        writePWM(temp->hl, 10 * perc + 1000);
      }
    }
  }
  String webpage;
  webpage += "<META HTTP-EQUIV='Refresh' CONTENT='1; URL=/check'>";
  webpage += "<html><body>Success!!!</body></html>";
  server.send(200, "text/html", webpage);
}

void checkDrone()
{
  String webpage;
  webpage =  "<html>";
  webpage += "<head><title>ESP8266 Flight Controller</title>";
  webpage += "<style> body { margin:0 auto; background-color: #000000; font-size:40px; font-family: Arial, Helvetica, Sans-Serif; Color: white; height: 90%; width: 90%}";
  webpage += "</style>";
  webpage += "</head>";
  webpage += "<body>";
  webpage += "<h1>Flight Controller</h1>";
  webpage += "<form action='/'><input type='submit' value='Back'></form>";
  webpage += "<form action='/check/save' method='POST'>";

  webpage += "<select name='Motor' id='Motor'>";
  webpage += "<option value='FL'>FL</option>";
  webpage += "<option value='FR'>FR</option>";
  webpage += "<option value='RL'>RL</option>";
  webpage += "<option value='RR'>RR</option>";
  webpage += "</select>";
  webpage += "<input type='number' step='1' name='Throttle' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(0) + "' min='0' max='100'>%";

  webpage += "<input type='submit' value='Senden'></form>";
  webpage += "</body>";
  webpage += "</html>";
  server.send(200, "text/html", webpage); // Send a response to the client asking for input
}

void setupServer() {
  ///////////////////////////////Wlan konnektivität starten und verbinden falls möglich
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);       //Als Access-Point konfiguieren
  WiFi.softAP(temp->ssid, temp->pass);  //Access-Point aufmachen
  MDNS.begin("Drone");
  yield();                  //Um background prozesse kümmern
  //////////////////////////////////////Over the Air Uploads
  server.on("/update", HTTP_POST, []() {
    resetDrone();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      #ifdef ESP8266
      WiFiUDP::stopAll();
      #endif
      uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
      Update.begin(maxSketchSpace);
      debugPrint("Start");
    }
    else if (upload.status == UPLOAD_FILE_WRITE) {
      Update.write(upload.buf, upload.currentSize);
      yield();
    }
    else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true))
      {
        String webpage;
        webpage += "<META HTTP-EQUIV='Refresh' CONTENT='10; URL=/'>";
        webpage += "<html><body>Success! But wait while i reboot! You will be redirected</body></html>";
        server.send(200, "text/html", webpage);
        debugPrint("Success!");
      }
      else
      {
        String webpage;
        webpage += "<META HTTP-EQUIV='Refresh' CONTENT='10; URL=/'>";
        webpage += "<html><body>NO Success!!! But wait while i reboot! You will be redirected</body></html>";
        server.send(200, "text/html", webpage);
        debugPrint("NO Success!");
      }
      yield();
    }
  });
  /////////////////////////////Variablenmodifikationsseite
  server.on("/", root);
  server.on("/save", handleSave);
  /////////////////////////////Resetseite
  server.on("/reset", resetDrone);
  /////////////////////////////Setupseite
  server.on("/setup", setupDrone);
  /////////////////////////////Checkseite
  server.on("/check", checkDrone);
  server.on("/check/save", handleCheck);
  /////////////////////////////Server Starten
  server.begin();
  // WebSerial.begin(&serialserver);
  // serialserver.begin();
}

void handleServer()
{
  #if defined ESP8266
  if(temp->Arming < 1500 || temp->debugging) // Wenn motoren nicht gearmt sind dann kümmere dich auch um den Server
  {
    server.handleClient();
    MDNS.update();
  }
  #endif
}

#endif //Server_h
