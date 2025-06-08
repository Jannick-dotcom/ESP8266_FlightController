#pragma once

#include "stdint.h"
#include "StallardOSPID.hpp"
#include <string>

typedef enum hardwareError
{
  HW_OK = 0,
  GYRO = 1,
  MEM = 2,
  RECEIVER = 4
}hardwareError;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Veränderbare Werte
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    float Frequenz = 100;                      //Sampling Rate Default: 250Hz

    double degpersec = 80.0;                    //Maximale Rotationsgeschwindigkeit im Rate Modus [°/s]
    double pid_max = 200.0;                     //Max Output of PID-Controller

    double pid_p_gain_roll = 5.0;               //Gain setting for the roll P-controller
    double pid_i_gain_roll = 0.50;               //Gain setting for the roll I-controller
    double pid_d_gain_roll = 0.0;               //Gain setting for the roll D-controller

    double pid_p_gain_pitch = 5.0;              //Gain setting for the pitch P-controller.
    double pid_i_gain_pitch = 0.50;              //Gain setting for the pitch I-controller.
    double pid_d_gain_pitch = 0.0;              //Gain setting for the pitch D-controller.

    double pid_p_gain_yaw = 1.0;                //Gain setting for the pitch P-controller.
    double pid_i_gain_yaw = 0.0;                //Gain setting for the pitch I-controller.
    double pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.

    const char *ssid = "Drohne";
    const char *pass = "Passwort";
    uint8_t controlMode = 0; // 0->Radio; 1->Web
    std::string terminalOutput;

    /////////////////////////////////////////////////////////////////////////////////////////////////////

    //Invertvariables
    int8_t invX = -1;
    int8_t invY = 1;
    int8_t invZ = 1;

    uint8_t anzMotoren = 4;

    //Pins der Motoren
    uint8_t vl = 12;
    uint8_t vr = 2;
    uint8_t hl = 15;
    uint8_t hr = 0;

    uint8_t debugging = 0; //0-> kein Debugging, 1-> Debugging

    int16_t ax, ay, az;
    int16_t gx, gy, g_z;
    int16_t temperature;
    double anglePitch, angleRoll;
    double gyroX, gyroY, gyroZ;
    double accX, accY, accZ;
    double accelPitch=0;
    double accelRoll=0;
    double yawImu = 0;

    // float pid_error_temp;
    double pid_roll_setpoint = 0, gyro_roll_input, pid_output_roll;
    double pid_pitch_setpoint = 0, gyro_pitch_input, pid_output_pitch;
    double pid_yaw_setpoint = 0, gyro_yaw_input, pid_output_yaw;
    StallardosPID pitch(pid_p_gain_pitch, pid_i_gain_pitch, pid_d_gain_pitch);
    StallardosPID roll(pid_p_gain_roll, pid_i_gain_roll, pid_d_gain_roll);
    StallardosPID yaw(pid_p_gain_yaw, pid_i_gain_yaw, pid_d_gain_yaw);

    uint16_t Received[10];  //Empfangene Werte
    double Throttle, Pitch, Roll, Yaw; //Werte von Fernbedienung
    uint16_t Mode, Arming;  //Steuerungswerte von Fernbedienung
    uint16_t minPulse = 1040; //Minimaler wert der zu den ESC´s gesendet wird wenn die Motoren laufen sollen
    uint16_t esc[4];    //Ausgangswerte zu Motoren
    double gxC, gyC, gzC; //Kalibrierwerte Gyro
    double axC, ayC, azC;  //Kalibrierwerte Beschleunigungssensor

    uint64_t nextloop = 0;
    hardwareError HardwareIssues = HW_OK; //0->OK, 1->Gyro, 2->Spiffs, 3->Receiver...

    uint32_t durchlaufT; //us per durchlauf
    uint16_t timeNeeded;
