#include "linker.h"

/* Initialize all perpherals */
/* Call necessary functions  */
void BSP_Init()
{
    /* Hardware */
    LCD_Init();
    //DHT22_Init();

    /* System */
    Delay_Init();
    UART_App_Init();
    HAL_UART_Transmit(&huart1, (uint8_t*)"System Init OK\r\n", 16, 500); 

    /* necesary functions */

}
