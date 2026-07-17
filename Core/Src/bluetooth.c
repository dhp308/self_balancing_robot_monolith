#include "bluetooth.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

void BT_Init(UART_HandleTypeDef *huart, BT_Data *bt)
{
    bt->received = 0;
    bt->Kp = 30.0f;
    bt->Ki = 0.0f;
    bt->Kd = 5.0f;
    memset(bt->buffer, 0, BT_BUFFER_SIZE);

    // UART 수신 인터럽트 시작
    HAL_UART_Receive_IT(huart, (uint8_t*)bt->buffer, BT_BUFFER_SIZE);
}

void BT_ParseCommand(BT_Data *bt)
{
    if (!bt->received) return;

    // "KP:30.0" 형식 파싱
    if (strncmp(bt->buffer, "KP:", 3) == 0)
        bt->Kp = atof(bt->buffer + 3);
    else if (strncmp(bt->buffer, "KI:", 3) == 0)
        bt->Ki = atof(bt->buffer + 3);
    else if (strncmp(bt->buffer, "KD:", 3) == 0)
        bt->Kd = atof(bt->buffer + 3);

    bt->received = 0;
    memset(bt->buffer, 0, BT_BUFFER_SIZE);
}

void BT_SendData(UART_HandleTypeDef *huart, float angle, float output)
{
    char buf[64];
    int angle_int = (int)(angle * 100);
    int output_int = (int)(output * 100);
    snprintf(buf, sizeof(buf), "A:%d.%02d,O:%d.%02d\r\n",
             angle_int/100, abs(angle_int%100),
             output_int/100, abs(output_int%100));
    HAL_UART_Transmit(huart, (uint8_t*)buf, strlen(buf), 100);
}

