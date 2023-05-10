#ifndef STALLARDOSPID_H_
#define STALLARDOSPID_H_

class StallardosPID
{
    double pid_max = 200.0; //Max Output of PID-Controller

    double pid_p_gain = 0.0; //Gain setting for the P-controller
    double pid_i_gain = 0.0; //Gain setting for the I-controller
    double pid_d_gain = 0.0; //Gain setting for the D-controller
    double pid_i_mem, pid_setpoint, pid_input, pid_output, pid_last_d_error;

public:
    StallardosPID();
    StallardosPID(double p_gain, double i_gain, double d_gain);
    double calculate_pid(double setpoint, double input);
};

#endif