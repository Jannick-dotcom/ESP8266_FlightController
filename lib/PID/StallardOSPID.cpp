#include "StallardOSPID.hpp"
#include "droneUtility.hpp"
/**
 * create a pid controller.
 *
 */
StallardosPID::StallardosPID()
{
    this->pid_i_mem = 0;
    this->pid_last_error = 0;
    this->pid_p_gain = 0;
    this->pid_i_gain = 0;
    this->pid_d_gain = 0;
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
    this->pid_last_error = 0;
    this->pid_p_gain = p_gain;
    this->pid_i_gain = i_gain;
    this->pid_d_gain = d_gain;
}


/**
 * calculate the output.
 *
 * @param setpoint setpoint if the controller
 * @param input input of the pid controller
 * @return output of the pid controller
 */
double StallardosPID::calculate_pid(double setpoint, double input, double dT)
{
    double pid_error = setpoint - input;

    //P
    double P = pid_p_gain * pid_error;

    //I
    double I = pid_i_mem + pid_i_gain * pid_error * dT;
    pid_i_mem = I;
    constrainValue(pid_i_mem, pid_max, -pid_max);
    //D
    double D = (pid_error - pid_last_error) * pid_d_gain / dT;
    pid_last_error = pid_error;

    //Combine
    double pid_output = P + I + D;

    //Limit
    constrainValue(pid_output, pid_max, -pid_max);

    return pid_output;
}

void StallardosPID::reset()
{
    this->pid_i_mem = 0;
    this->pid_last_error = 0;
}
