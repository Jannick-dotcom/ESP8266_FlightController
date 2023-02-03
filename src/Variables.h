#ifndef Variables_h
#define Variables_h

#include "stdint.h"

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Veränderbare Werte
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct vals {
    double Frequenz = 250;                      //Sampling Rate Default: 250Hz

    double degpersec = 80.0;                    //Maximale Rotationsgeschwindigkeit im Rate Modus [°/s]
    double pid_max = 200.0;                     //Max Output of PID-Controller

    double pid_p_gain_roll = 0.0;               //Gain setting for the roll P-controller
    double pid_i_gain_roll = 0.0;               //Gain setting for the roll I-controller
    double pid_d_gain_roll = 0.0;               //Gain setting for the roll D-controller

    double pid_p_gain_pitch = 0.0;              //Gain setting for the pitch P-controller.
    double pid_i_gain_pitch = 0.0;              //Gain setting for the pitch I-controller.
    double pid_d_gain_pitch = 0.0;              //Gain setting for the pitch D-controller.

    double pid_p_gain_yaw = 0.0;                //Gain setting for the pitch P-controller.
    double pid_i_gain_yaw = 0.0;                //Gain setting for the pitch I-controller.
    double pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.

    const char *ssid = "Drohne";
    const char *pass = "";

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
    //Pin des Kamera tilt Servos
    uint8_t camServo = 16;




    uint8_t debugging = 1; //0-> kein Debugging, 1-> Debugging

    int16_t ax, ay, az;
    int16_t gx, gy, gz;
    int16_t temperature;
    double anglePitch, angleRoll;
    double gyroX, gyroY, gyroZ;
    double accX, accY, accZ;
    double acc_total_vector;

    double pid_error_temp;
    double pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
    double pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
    double pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;

    uint16_t Received[10];  //Empfangene Werte
    uint16_t Throttle, Pitch, Roll, Yaw, Mode, Campitch, Arming;  //Steuerungswerte von Fernbedienung
    uint16_t minPulse = 1040; //Minimaler wert der zu den ESC´s gesendet wird wenn die Motoren laufen sollen
    uint16_t esc[4];    //Ausgangswerte zu Motoren
    double gxC, gyC, gzC; //Kalibrierwerte Gyro
    double axC, ayC, azC;  //Kalibrierwerte Beschleunigungssensor

    uint32_t nextloop = 0;
    uint8_t HardwareIssues = 0; //0->OK, 1->Gyro, 2->Spiffs, 3->Receiver...

    uint16_t durchlaufT;
};

#endif
