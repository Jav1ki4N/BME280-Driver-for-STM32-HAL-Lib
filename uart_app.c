#include "uart_app.h"

uint8_t recv_data = 0;

/* UART Settings */
void UART_App_Init()
{
    HAL_UART_Receive_IT(&huart1, &recv_data, 1);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART1)
    {
        HAL_UART_Receive_IT(&huart1, &recv_data, 1);
    }
}
