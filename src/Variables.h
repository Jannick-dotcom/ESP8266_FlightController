#ifndef Variables_h
#define Variables_h

#include "stdint.h"
#include "StallardOSPID.hpp"

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
struct vals {
    float Frequenz = 250;                      //Sampling Rate Default: 250Hz

    float degpersec = 80.0;                    //Maximale Rotationsgeschwindigkeit im Rate Modus [°/s]
    float pid_max = 200.0;                     //Max Output of PID-Controller

    float pid_p_gain_roll = 0.0;               //Gain setting for the roll P-controller
    float pid_i_gain_roll = 0.0;               //Gain setting for the roll I-controller
    float pid_d_gain_roll = 0.0;               //Gain setting for the roll D-controller

    float pid_p_gain_pitch = 0.0;              //Gain setting for the pitch P-controller.
    float pid_i_gain_pitch = 0.0;              //Gain setting for the pitch I-controller.
    float pid_d_gain_pitch = 0.0;              //Gain setting for the pitch D-controller.

    float pid_p_gain_yaw = 0.0;                //Gain setting for the pitch P-controller.
    float pid_i_gain_yaw = 0.0;                //Gain setting for the pitch I-controller.
    float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.

    const char *ssid = "Drohne";
    const char *pass = "Passwort";
    uint8_t controlMode = 0; // 0->Radio; 1->Web

    /////////////////////////////////////////////////////////////////////////////////////////////////////

    //Invertvariables
    int8_t invX = -1;
    int8_t invY = 1;
    int8_t invZ = -1;

    uint8_t anzMotoren = 4;

    //Pins der Motoren
    uint8_t vl = 12;
    uint8_t vr = 2;
    uint8_t hl = 15;
    uint8_t hr = 0;

    uint8_t debugging = 0; //0-> kein Debugging, 1-> Debugging

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temperature;
    float anglePitch, angleRoll;
    float gyroX, gyroY, gyroZ;
    float accX, accY, accZ;
    float acc_total_vector;

    // float pid_error_temp;
    float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
    float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
    float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
    StallardosPID pitch;
    StallardosPID roll;
    StallardosPID yaw;

    uint16_t Received[10];  //Empfangene Werte
    float Throttle, Pitch, Roll, Yaw, Mode, Arming;  //Steuerungswerte von Fernbedienung
    uint16_t minPulse = 1020; //Minimaler wert der zu den ESC´s gesendet wird wenn die Motoren laufen sollen
    uint16_t esc[4];    //Ausgangswerte zu Motoren
    float gxC, gyC, gzC; //Kalibrierwerte Gyro
    float axC, ayC, azC;  //Kalibrierwerte Beschleunigungssensor

    uint32_t nextloop = 0;
    hardwareError HardwareIssues = HW_OK; //0->OK, 1->Gyro, 2->Spiffs, 3->Receiver...

    uint16_t durchlaufT; //us per durchlauf
    uint16_t timeNeeded;
};

#endif
