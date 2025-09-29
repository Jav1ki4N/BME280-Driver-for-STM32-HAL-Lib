#ifndef LINKER_H
#define LINKER_H

#include <main.h>
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include "gpio.h"
#include "lcd2004.h"
#include "display.h"
#include "dht22.h"
#include "delay.h"
#include "tim.h"
#include "i2c.h"
#include "usart.h"
#include "bme280.h"
#include "uart_app.h"


void BSP_Init(void);

#endif
