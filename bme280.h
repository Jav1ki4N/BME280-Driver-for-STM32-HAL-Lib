/*********************************************/
/**   BME280 Driver using I2C and Hal Lib   **/
/**           i4N, Sep.30th,2025            **/
/*********************************************/

/* Datasheet: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bme280-ds002.pdf */

#ifndef _BME280_H_
#define _BME280_H_

#include "linker.h"

/*********************/
/** Pin Assignments **/

#define BME280_I2C                &hi2c2
#define BME280_I2C_PORT           GPIOB
#define BME280_I2C_SCL            GPIO_PIN_10 // I2C2
#define BME280_I2C_SDA            GPIO_PIN_11 // I2C2

/*************************/
/**  ADDRs and Commands **/

#define BME280_I2C_ADDR          (0x76<<1) // if SDO is connected to GND (default)
//#define BME280_I2C_ADDR_ALT    (0x77<<1) // if SDO is connected to VCC
#define BME280_CHIP_ID            0xD0     // ID register

#define BME280_PRESSURE_MSB       0xF7     // Pressure    MSB,    pressure is 20 bits, and we start from here (BURST READ)
//#define BME280_TEMPERATURE_MSB  0xFA     // Temperature MSB,    temperature is 20 bits
//#define BME280_HUMIDITY_MSB     0xFD     // Humidity    MSB,    humidity is 16 bits

#define BME280_Calibration_REG1   0x88     // where the calibration data starts
#define BME280_Calibration_REG2   0xA1     // single byte of dig_H1, separated from other humidity calibration data
#define BME280_Calibration_REG3   0xE1     // where the rest of humidity calibration data starts

#define BME280_RESET              0xE0     // soft reset register
#define BME280_CTRL_HUMI          0xF2     // humidity control register
#define BME280_CTRL_MEA           0xF4     // measurement control register
#define BME280_CONFIG             0xF5     // configuration register

/*****************/
/**  debugging  **/

#define IS_UART                   1        // set to 1 to enable uart debugging info
#define UART_DEBUG                &huart1  // default debug uart

/********************/
/** data structure **/

typedef struct
{
  /* measurements */
  float pressure;     
  float temperature;  
  float humidity;

  /* raw data */
  int32_t temp_raw,
          press_raw,
          humi_raw;

  /* temperature calibration parameters */
  uint16_t dig_T1;
  int16_t  dig_T2, dig_T3;

  /*  pressure calibration parameters */
  uint16_t dig_P1;
  int16_t  dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

  /*  humidity calibration parameters */
  uint8_t  dig_H1,dig_H3;
  int16_t  dig_H2,dig_H4, dig_H5;
  int8_t   dig_H6;

}bme280_data_t;

extern bme280_data_t BME280;

/* APIs */
void BME280_DataProc(bme280_data_t *device); // Call in main loop
                                             // e.g. BME280_DataProc(&BME280);    

#endif
