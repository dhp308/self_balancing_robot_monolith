#include "motor.h"

void Motor_Init(TIM_HandleTypeDef *htim)
{
    // STBY 핀 HIGH → 드라이버 활성화
    HAL_GPIO_WritePin(MOTOR_GPIO, STBY_PIN, GPIO_PIN_SET);

    // PWM 시작
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(htim, TIM_CHANNEL_2);

    // 초기 정지
    Motor_Stop(htim);
}

void Motor_SetSpeed(TIM_HandleTypeDef *htim, int16_t left_speed, int16_t right_speed)
{
    // 좌측 모터
    if (left_speed >= 0) {
        HAL_GPIO_WritePin(MOTOR_GPIO, AIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_GPIO, AIN2_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(MOTOR_GPIO, AIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_GPIO, AIN2_PIN, GPIO_PIN_SET);
        left_speed = -left_speed;
    }

    // 우측 모터
    if (right_speed >= 0) {
        HAL_GPIO_WritePin(MOTOR_GPIO, BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_GPIO, BIN2_PIN, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(MOTOR_GPIO, BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_GPIO, BIN2_PIN, GPIO_PIN_SET);
        right_speed = -right_speed;
    }

    // PWM 출력
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, left_speed);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, right_speed);
}

void Motor_Stop(TIM_HandleTypeDef *htim)
{
    HAL_GPIO_WritePin(MOTOR_GPIO, AIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO, AIN2_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO, BIN1_PIN, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(MOTOR_GPIO, BIN2_PIN, GPIO_PIN_RESET);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_1, 0);
    __HAL_TIM_SET_COMPARE(htim, TIM_CHANNEL_2, 0);
}

