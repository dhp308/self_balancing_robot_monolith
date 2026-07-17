#include "mpu6050.h"
#include <math.h>

// 상보필터 계수
#define ALPHA 0.98f

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c)
{
    uint8_t data;

    // 슬립 모드 해제
    data = 0x00;
    if (HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_PWR_MGMT_1,
                          1, &data, 1, 100) != HAL_OK)
        return HAL_ERROR;

    // 자이로 설정 (±250°/s)
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_GYRO_CONFIG,
                      1, &data, 1, 100);

    // 가속도 설정 (±2g)
    data = 0x00;
    HAL_I2C_Mem_Write(hi2c, MPU6050_ADDR, MPU6050_ACCEL_CONFIG,
                      1, &data, 1, 100);

    return HAL_OK;
}

HAL_StatusTypeDef MPU6050_ReadData(I2C_HandleTypeDef *hi2c, MPU6050_Data *data)
{
    uint8_t buf[14];
    int16_t accel_x, accel_z, gyro_y;
    static float angle = 0.0f;

    // 가속도 + 자이로 14바이트 읽기
    if (HAL_I2C_Mem_Read(hi2c, MPU6050_ADDR, MPU6050_ACCEL_XOUT,
                         1, buf, 14, 100) != HAL_OK)
        return HAL_ERROR;

    // 원시 데이터 변환
    accel_x = (int16_t)(buf[0] << 8 | buf[1]);
    accel_z = (int16_t)(buf[4] << 8 | buf[5]);
    gyro_y  = (int16_t)(buf[8] << 8 | buf[9]);

    // 가속도로 각도 계산
    float accel_angle = atan2f((float)accel_x, (float)accel_z) * 180.0f / M_PI;

    // 자이로 각속도 (°/초)
    float gyro_rate = (float)gyro_y / 131.0f;

    // 상보필터 (dt = 0.01s = 10ms)
    angle = ALPHA * (angle + gyro_rate * 0.01f) + (1.0f - ALPHA) * accel_angle;

    data->angle = angle;
    data->gyro_rate = gyro_rate;

    return HAL_OK;
}
