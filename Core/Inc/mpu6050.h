#ifndef MPU6050_H
#define MPU6050_H

#include "main.h"

// MPU-6050 I2C 주소
#define MPU6050_ADDR    0x68 << 1

// 레지스터 주소
#define MPU6050_PWR_MGMT_1  0x6B
#define MPU6050_ACCEL_XOUT  0x3B
#define MPU6050_GYRO_XOUT   0x43
#define MPU6050_CONFIG      0x1A
#define MPU6050_GYRO_CONFIG 0x1B
#define MPU6050_ACCEL_CONFIG 0x1C

// 데이터 구조체
typedef struct {
    float angle;      // 현재 각도 (도)
    float gyro_rate;  // 각속도 (도/초)
    int16_t raw_accel_x;  // 진단용
    int16_t raw_accel_z;  // 진단용
    int16_t raw_gyro_y;   // 진단용
} MPU6050_Data;

// 함수 선언
HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *hi2c);
HAL_StatusTypeDef MPU6050_ReadData(I2C_HandleTypeDef *hi2c, MPU6050_Data *data);

#endif /* MPU6050_H */
