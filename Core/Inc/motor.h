#ifndef MOTOR_H
#define MOTOR_H

#include "main.h"

// TB6612FNG GPIO 핀 정의
#define AIN1_PIN    GPIO_PIN_1
#define AIN2_PIN    GPIO_PIN_2
#define BIN1_PIN    GPIO_PIN_4
#define BIN2_PIN    GPIO_PIN_5
#define STBY_PIN    GPIO_PIN_6
#define MOTOR_GPIO  GPIOA

// PWM 최대값
#define PWM_MAX     1000

// 모터 방향
typedef enum {
    MOTOR_FORWARD,
    MOTOR_BACKWARD,
    MOTOR_STOP
} Motor_Direction;

// 함수 선언
void Motor_Init(TIM_HandleTypeDef *htim);
void Motor_SetSpeed(TIM_HandleTypeDef *htim, int16_t left_speed, int16_t right_speed);
void Motor_Stop(TIM_HandleTypeDef *htim);

#endif /* MOTOR_H */
