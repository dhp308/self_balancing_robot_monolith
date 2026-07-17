#include "pid.h"

void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}

float PID_Calculate(PID_Controller *pid, float setpoint, float measured, float dt)
{
    float error = setpoint - measured;

    // 비례항
    float P = pid->Kp * error;

    // 적분항
    pid->integral += error * dt;
    float I = pid->Ki * pid->integral;

    // 미분항
    float D = pid->Kd * (error - pid->prev_error) / dt;
    pid->prev_error = error;

    // 출력 계산
    pid->output = P + I + D;

    // 출력 제한
    if (pid->output > PID_OUTPUT_MAX)
        pid->output = PID_OUTPUT_MAX;
    else if (pid->output < PID_OUTPUT_MIN)
        pid->output = PID_OUTPUT_MIN;

    return pid->output;
}

void PID_Reset(PID_Controller *pid)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->output = 0.0f;
}

void PID_SetGains(PID_Controller *pid, float Kp, float Ki, float Kd)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    PID_Reset(pid);
}
