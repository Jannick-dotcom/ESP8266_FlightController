#ifndef ESP_SERVER_H
#define ESP_SERVER_H

#include "droneUtility.h"
#include <WiFiClient.h>
#include <ArduinoJson.h>
#include "Variables.h"

#ifdef ESP8266
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include "FS.h"
#include "core_esp8266_waveform.h" //Nützlich für PWM
ESP8266WebServer server(80);  //Auf Port 80
#endif

#ifdef ESP32
#include <WiFi.h>
#include <Update.h>
#include <WebServer.h>
#include <SPIFFS.h>
WebServer server(80);  //Auf Port 80
#endif

bool loadconfig() {
  if (SPIFFS.exists("/config.json")) {
    File configFile = SPIFFS.open("/config.json", "r");
    StaticJsonDocument<800> json;
    deserializeJson(json, configFile);
    configFile.close();
    temp->Frequenz = json ["Frequenz"]; //Frequenz
    temp->degpersec = json["degpersec"]; //Maximale Rotationsgeschwindigkeit im Rate Modus
    temp->pid_max = json["pid_max"];                //Maximum output of the PID-controller (+/-)

    temp->pid_p_gain_roll = json["pid_p_gain_roll"];    //Gain setting for the roll P-controller
    temp->pid_i_gain_roll = json["pid_i_gain_roll"];    //Gain setting for the roll I-controller
    temp->pid_d_gain_roll = json["pid_d_gain_roll"];    //Gain setting for the roll D-controller

    temp->pid_p_gain_pitch = json["pid_p_gain_pitch"];  //Gain setting for the pitch P-controller.
    temp->pid_i_gain_pitch = json["pid_i_gain_pitch"];  //Gain setting for the pitch I-controller.
    temp->pid_d_gain_pitch = json["pid_d_gain_pitch"];  //Gain setting for the pitch D-controller.

    temp->pid_p_gain_yaw = json["pid_p_gain_yaw"];      //Gain setting for the pitch P-controller.
    temp->pid_i_gain_yaw = json["pid_i_gain_yaw"];      //Gain setting for the pitch I-controller.
    temp->pid_d_gain_yaw = json["pid_d_gain_yaw"];      //Gain setting for the pitch D-controller.

    /*
      pid_p_gain_roll = (0.004 / Frequenz) * pid_p_gain_roll;
      pid_i_gain_roll = (0.004 / Frequenz) * pid_i_gain_roll;
      pid_d_gain_roll = (0.004 / Frequenz) * pid_d_gain_roll;

      pid_p_gain_pitch = (0.004 / Frequenz) * pid_p_gain_pitch;
      pid_i_gain_pitch = (0.004 / Frequenz) * pid_i_gain_pitch;
      pid_d_gain_pitch = (0.004 / Frequenz) * pid_d_gain_pitch;

      pid_p_gain_yaw = (0.004 / Frequenz) * pid_p_gain_yaw;
      pid_i_gain_yaw = (0.004 / Frequenz) * pid_i_gain_yaw;
      pid_d_gain_yaw = (0.004 / Frequenz) * pid_d_gain_yaw;*/
    return 0;
  }
  else
  {
    temp->HardwareIssues = 2; //First time of Boot or Spiffs broken
    SPIFFS.format();
    StaticJsonDocument<800> json;
    File configFile = SPIFFS.open("/config.json", "w");

    json["Frequenz"] = temp->Frequenz;
    json["degpersec"] = temp->degpersec; //Maximale Rotationsgeschwindigkeit im Rate Modus
    json["pid_max"] = temp->pid_max;     //Maximaler PID wert

    json["pid_p_gain_roll"] = temp->pid_p_gain_roll;               //Gain setting for the roll P-controller
    json["pid_i_gain_roll"] = temp->pid_i_gain_roll;              //Gain setting for the roll I-controller
    json["pid_d_gain_roll"] = temp->pid_d_gain_roll;              //Gain setting for the roll D-controller

    json["pid_p_gain_pitch"] = temp->pid_p_gain_pitch;  //Gain setting for the pitch P-controller.
    json["pid_i_gain_pitch"] = temp->pid_i_gain_pitch;  //Gain setting for the pitch I-controller.
    json["pid_d_gain_pitch"] = temp->pid_d_gain_pitch;  //Gain setting for the pitch D-controller.

    json["pid_p_gain_yaw"] = temp->pid_p_gain_yaw;                //Gain setting for the pitch P-controller. //4.0
    json["pid_i_gain_yaw"] = temp->pid_i_gain_yaw;               //Gain setting for the pitch I-controller. //0.02
    json["pid_d_gain_yaw"] = temp->pid_d_gain_yaw;                //Gain setting for the pitch D-controller.

    serializeJson(json, configFile);
    configFile.close();
    return 1;
  }
}

void toString(double value, char *buffer)
{
  dtostrf(value, 4, 3, buffer);
}

void root()
{
  // if(loadconfig()) //Wenn das erste mal auf den Server zugegriffen wird das setup aufrufen
  // {
  //   setupDrone();
  //   return;
  // }
  String webpage;
  const String inBreite = "200";
  const String inHoehe = "20";
  char value[20];
  webpage =  "<html>";
  webpage += "<head><title>ESP8266 Flight Controller</title>";
  webpage += "<style> body { margin:0 auto; background-color: #000000; font-size:40px; font-family: Arial, Helvetica, Sans-Serif; Color: white; height: 90%; width: 90%}";
  webpage += "</style>";
  webpage += "</head>";
  webpage += "<body>";
  webpage += "<h1>Flight Controller</h1>";
  webpage += "<h4>Errorcode: " + String(temp->HardwareIssues) + "</h2>";
  webpage += "<table style='width:100%'> <tr>";
  webpage += "<tr><form><input type='button' value='Setup der Drohne aufrufen' onclick = window.location.href='http://192.168.4.1/setup'></form></tr>";
  webpage += "<form action='http://192.168.4.1/save' method='POST'>";

  toString(temp->Frequenz, value);
  webpage += "<tr><td>Frequenz[Hz]:</td><td><input type='number' step='0.1' name='Frequenz' style='width: " + inBreite + "px; height: " + inHoehe + "px' value='" + String(value) + "' min='250' max='4000'></td></tr>";

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
  webpage += "<form method='POST' action='/update' enctype='multipart/form-data'><input type='file' name='update'><input type='submit' value='Update'></form>";
  webpage += "</body>";
  webpage += "</html>";
  server.send(200, "text/html", webpage); // Send a response to the client asking for input
}

void handleSave() {
  if (server.args() > 0 ) { // Arguments were received
    for ( uint8_t i = 0; i < server.args(); i++ ) {
      //Serial.print(server.argName(i)); // Display the argument
      //Serial.print(" : ");
      //Serial.println(server.arg(i).toFloat());

      if (server.argName(i) == "Frequenz")
      {
        temp->Frequenz = server.arg(i).toFloat();
        temp->durchlaufT = (1.0 / temp->Frequenz) * 1000000.0;
      }
      else if (server.argName(i) == "degpersec")
      {
        temp->degpersec = server.arg(i).toFloat();
      }
      else if (server.argName(i) == "pid_max")
      {
        temp->pid_max = server.arg(i).toFloat();
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
    StaticJsonDocument<800> json;
    if (SPIFFS.exists("config.json"))
    {
      SPIFFS.remove("/config.json");
    }
    File configFile = SPIFFS.open("/config.json", "w");

    json["Frequenz"] = temp->Frequenz; //Durchlauffrequenz
    json["degpersec"] = temp->degpersec; //Maximale Rotationsgeschwindigkeit im Rate Modus
    json["pid_max"] = temp->pid_max;     //Maximaler PID wert

    json["pid_p_gain_roll"] = temp->pid_p_gain_roll;              //Gain setting for the roll P-controller.
    json["pid_i_gain_roll"] = temp->pid_i_gain_roll;              //Gain setting for the roll I-controller.
    json["pid_d_gain_roll"] = temp->pid_d_gain_roll;              //Gain setting for the roll D-controller.

    json["pid_p_gain_pitch"] = temp->pid_p_gain_pitch;            //Gain setting for the pitch P-controller.
    json["pid_i_gain_pitch"] = temp->pid_i_gain_pitch;            //Gain setting for the pitch I-controller.
    json["pid_d_gain_pitch"] = temp->pid_d_gain_pitch;            //Gain setting for the pitch D-controller.

    json["pid_p_gain_yaw"] = temp->pid_p_gain_yaw;                //Gain setting for the pitch P-controller.
    json["pid_i_gain_yaw"] = temp->pid_i_gain_yaw;                //Gain setting for the pitch I-controller.
    json["pid_d_gain_yaw"] = temp->pid_d_gain_yaw;                //Gain setting for the pitch D-controller.

    serializeJson(json, configFile);
    configFile.close();
    String webpage;
    webpage += "<META HTTP-EQUIV='Refresh' CONTENT='1; URL=http://192.168.4.1'>";
    webpage += "<html><style> body { margin:50px auto; background-color: #000000; font-size:60px; font-family: Arial, Helvetica, Sans-Serif; Color: white; height: 90%; width: 90%}";
    webpage += "</style><body>Wurde gespeichert!</body></html>";
    server.send(200, "text/html", webpage);
  }
}

void setupServer() {
  ///////////////////////////////Wlan konnektivität starten und verbinden falls möglich
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);       //Als Access-Point konfiguieren
  WiFi.softAP(temp->ssid, temp->pass);  //Access-Point aufmachen
  yield();                  //Um background prozesse kümmern
  //////////////////////////////////////Over the Air Uploads
  server.on("/update", HTTP_POST, []() {
    Serial.println("RESET!");
    ESP.restart();
    yield();
  }, []() {
    HTTPUpload& upload = server.upload();
    if (upload.status == UPLOAD_FILE_START) {
      // WiFiUDP::stopAll();
      uint32_t maxSketchSpace = (ESP.getFreeSketchSpace() - 0x1000) & 0xFFFFF000;
      Update.begin(maxSketchSpace);
      Serial.println("Start");
    }
    else if (upload.status == UPLOAD_FILE_WRITE) {
      Update.write(upload.buf, upload.currentSize);
    }
    else if (upload.status == UPLOAD_FILE_END) {
      if (Update.end(true))
      {
        String webpage;
        webpage += "<META HTTP-EQUIV='Refresh' CONTENT='10; URL=http://192.168.4.1'>";
        webpage += "<html><body>Success! But wait while i reboot! You will be redirected</body></html>";
        server.send(200, "text/html", webpage);
      }
      else
      {
        String webpage;
        webpage += "<META HTTP-EQUIV='Refresh' CONTENT='10; URL=http://192.168.4.1'>";
        webpage += "<html><body>NO Success!!! But wait while i reboot! You will be redirected</body></html>";
        server.send(200, "text/html", webpage);
      }
    }
  });
  /////////////////////////////Variablenmodifikationsseite
  server.on("/", root);
  server.on("/save", handleSave);
  /////////////////////////////Setupseite
  // server.on("/setup", setupDrone);
  // server.on("/saveSetup", saveSetup);
  /////////////////////////////Server Starten
  server.begin();
}

#endif //Server_h
