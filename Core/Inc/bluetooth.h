#ifndef BLUETOOTH_H
#define BLUETOOTH_H

#include "main.h"

// 블루투스 버퍼 크기
#define BT_BUFFER_SIZE  64

// 수신 데이터 구조체
typedef struct {
    char buffer[BT_BUFFER_SIZE];
    uint8_t received;
    float Kp;
    float Ki;
    float Kd;
} BT_Data;

// 함수 선언
void BT_Init(UART_HandleTypeDef *huart, BT_Data *bt);
void BT_ParseCommand(BT_Data *bt);
void BT_SendData(UART_HandleTypeDef *huart, float angle, float output);

#endif /* BLUETOOTH_H */
