#ifndef PID_H
#define PID_H

// PID 파라미터 구조체
typedef struct {
    float Kp;
    float Ki;
    float Kd;
    float integral;
    float prev_error;
    float output;
} PID_Controller;

// PID 초기값 (MATLAB 시뮬레이션 기반)
#define PID_KP_INIT  30.0f
#define PID_KI_INIT  0.0f
#define PID_KD_INIT  5.0f

// 출력 제한 (PWM 최대값)
#define PID_OUTPUT_MAX   1000.0f
#define PID_OUTPUT_MIN  -1000.0f

// 함수 선언
void PID_Init(PID_Controller *pid, float Kp, float Ki, float Kd);
float PID_Calculate(PID_Controller *pid, float setpoint, float measured, float dt);
void PID_Reset(PID_Controller *pid);
void PID_SetGains(PID_Controller *pid, float Kp, float Ki, float Kd);

#endif /* PID_H */
