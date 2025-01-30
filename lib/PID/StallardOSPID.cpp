#include "StallardOSPID.hpp"

/**
 * create a pid controller.
 *
 */
StallardosPID::StallardosPID()
{
    this->pid_i_mem = 0;
    this->pid_last_d_error = 0;
    this->pid_p_gain = 0;
    this->pid_i_gain = 0;
    this->pid_d_gain = 0;
    this->pid_output = 0;
}

/**
 * create a pid controller.
 *
 * @param p_gain  gain of the p part
 * @param i_gain  gain of the i part
 * @param d_gain  gain of the d part
 */
StallardosPID::StallardosPID(double p_gain, double i_gain, double d_gain)
{
    this->pid_i_mem = 0;
    this->pid_last_d_error = 0;
    this->pid_p_gain = p_gain;
    this->pid_i_gain = i_gain;
    this->pid_d_gain = d_gain;
    this->pid_output = 0;
}


/**
 * calculate the output.
 *
 * @param setpoint setpoint if the controller
 * @param input input of the pid controller
 * @return output of the pid controller
 */
double StallardosPID::calculate_pid(double setpoint, double input)
{
    
    double pid_error_temp;
    pid_error_temp = setpoint - input;
    pid_i_mem += pid_i_gain * pid_error_temp;
    if (pid_i_mem > pid_max)
        pid_i_mem = pid_max;
    else if (pid_i_mem < pid_max * -1)
        pid_i_mem = pid_max * -1;

    pid_output = pid_p_gain * pid_error_temp + pid_i_mem + pid_d_gain * (pid_error_temp - pid_last_d_error);
    if (pid_output > pid_max)
        pid_output = pid_max;
    else if (pid_output < pid_max * -1)
        pid_output = pid_max * -1;

    pid_last_d_error = pid_error_temp;
    
    return pid_output;
}

void StallardosPID::reset()
{
    this->pid_i_mem = 0;
    this->pid_last_d_error = 0;
}
