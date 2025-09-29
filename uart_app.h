#ifndef _UART_APP_H_
#define _UART_APP_H_

#include "linker.h"

/* Call in Bsp_Init()  */
void UART_App_Init(void);

extern uint16_t center_x;
extern uint16_t center_y;

#endif
