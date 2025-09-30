#include "bme280.h"

bme280_data_t BME280; // create a BME280 device entity 
                      // extern in bme280.h

/************************************************************************************/

/*
    * @brief  Read calibration(Trimming) data from sensor
    * @retval stautus - 0: success, other: fail
*/
uint8_t BME280_GetCalibrationData(bme280_data_t *device)
{
    
    uint8_t calibration_buf[32] = {0}; 
    
    if(HAL_I2C_Mem_Read(BME280_I2C, 
                        BME280_I2C_ADDR, 
                        BME280_Calibration_REG1, //0x88
                        1, 
                        calibration_buf,          
                        24,                       
                        500)!=HAL_OK) {
        #if IS_UART
            HAL_UART_Transmit(UART_DEBUG, (uint8_t*)"Error: Cannot read calibration REG1\r\n", 40, 500);
        #endif
        return 1;       // error handler1
    }
    
    if(HAL_I2C_Mem_Read(BME280_I2C, 
                        BME280_I2C_ADDR, 
                        BME280_Calibration_REG2, //0xA1
                        1, 
                        &calibration_buf[24],          
                        1,                        
                        500)!=HAL_OK) {
        #if IS_UART
            HAL_UART_Transmit(UART_DEBUG, (uint8_t*)"Error: Cannot read calibration REG2\r\n", 40, 500);
        #endif                    
        return 2;      // error handler2
    }
    if(HAL_I2C_Mem_Read(BME280_I2C, 
                        BME280_I2C_ADDR, 
                        BME280_Calibration_REG3, //0xE1
                        1, 
                        &calibration_buf[25],          
                        7,                         
                        500)!=HAL_OK) {
        #if IS_UART
            HAL_UART_Transmit(UART_DEBUG, (uint8_t*)"Error: Cannot read calibration REG3\r\n", 40, 500);
        #endif
        return 3;     // error handler3
    } 

    /* assign calibration data */
    device->dig_T1 = (uint16_t)((calibration_buf[1]  << 8)|(calibration_buf[0]));  //0x88
    device->dig_T2 = (int16_t) ((calibration_buf[3]  << 8)|(calibration_buf[2]));
    device->dig_T3 = (int16_t) ((calibration_buf[5]  << 8)|(calibration_buf[4]));

    device->dig_P1 = (uint16_t)((calibration_buf[7]  << 8)|(calibration_buf[6]));
    device->dig_P2 = (int16_t) ((calibration_buf[9]  << 8)|(calibration_buf[8]));
    device->dig_P3 = (int16_t) ((calibration_buf[11] << 8)|(calibration_buf[10]));
    device->dig_P4 = (int16_t) ((calibration_buf[13] << 8)|(calibration_buf[12]));
    device->dig_P5 = (int16_t) ((calibration_buf[15] << 8)|(calibration_buf[14]));
    device->dig_P6 = (int16_t) ((calibration_buf[17] << 8)|(calibration_buf[16]));
    device->dig_P7 = (int16_t) ((calibration_buf[19] << 8)|(calibration_buf[18]));
    device->dig_P8 = (int16_t) ((calibration_buf[21] << 8)|(calibration_buf[20]));
    device->dig_P9 = (int16_t) ((calibration_buf[23] << 8)|(calibration_buf[22]));
    
    device->dig_H1 = (uint8_t)  (calibration_buf[24]); 
    device->dig_H2 = (int16_t) ((calibration_buf[26] << 8)|(calibration_buf[25])); //0xE1
    device->dig_H3 = (uint8_t)  (calibration_buf[27]);
    device->dig_H4 = (int16_t) ((calibration_buf[28] << 4)|(calibration_buf[29] & 0x0F));
    device->dig_H5 = (int16_t) ((calibration_buf[30] << 4)|(calibration_buf[29] >>   4));
    device->dig_H6 = (int8_t)   (calibration_buf[31]);
    
    #if IS_UART
        HAL_UART_Transmit(UART_DEBUG, (uint8_t*)"Calibration data read\r\n", 23, 500);
    #endif

    return 0; //succeed
}

/*
 * @brief  BME280 Configuration
 * humi_osrs:  humidity    oversampling rate - 0,1,2,4,8,16 (x0,x1,x2,x4,x8,x16)
 * temp_osrs:  temperature oversampling rate - 0,1,2,4,8,16 (x0,x1,x2,x4,x8,x16)
 * press_osrs: pressure    oversampling rate - 0,1,2,4,8,16 (x0,x1,x2,x4,x8,x16)
 * mode: S,F,N (sleep, forced, normal)
 * standby: 0.5,62.5,125,250,500,1000,10,20 in ms, *1000 as parameter ( VERY IMPORTANT ) 
 * filter: 0,2,4,8,16
*/

void BME280_Config(uint8_t humi_osrs, uint8_t temp_osrs, uint8_t press_osrs, char mode, uint32_t standby, uint8_t filter)
{
    uint8_t reset   = 0xB6; // always do soft reset, change if necessary
    uint8_t osrs_h,         // humidity    oversampling rate
            osrs_t,         // temperature oversampling rate
            osrs_p;         // pressure    oversampling rate

    uint8_t mode_byte,
            standby_byte,
            filter_byte;


    switch(humi_osrs)
    {
        case 0:  osrs_h = 0x0;break;     // skipped
        case 1:  osrs_h = 0x1;break;     // x1
        case 2:  osrs_h = 0x2;break;     // x2
        case 4:  osrs_h = 0x3;break;     // x4
        case 8:  osrs_h = 0x4;break;     // x8
        case 16: osrs_h = 0x5;break;     // x16
        default: osrs_h = 0x1;break;     // default x1
    }

    uint8_t ctrl_hum = osrs_h; // ctrl_hum register value

    switch(temp_osrs)
    {
        case 0:  osrs_t = 0x0;break;     // skipped
        case 1:  osrs_t = 0x1;break;     // x1
        case 2:  osrs_t = 0x2;break;     // x2
        case 4:  osrs_t = 0x3;break;     // x4
        case 8:  osrs_t = 0x4;break;     // x8
        case 16: osrs_t = 0x5;break;     // x16
        default: osrs_t = 0x1;break;     // default x1
    }

    switch(press_osrs)
    {
        case 0:  osrs_p = 0x0;break;     // skipped
        case 1:  osrs_p = 0x1;break;     // x1
        case 2:  osrs_p = 0x2;break;
        case 4:  osrs_p = 0x3;break;     // x4
        case 8:  osrs_p = 0x4;break;     // x8
        case 16: osrs_p = 0x5;break;     // x16
        default: osrs_p = 0x1;break;     // default x1
    }

    switch(mode)
    {
        case 'S':  mode_byte = 0x0;break;      // sleep
        case 'F':  mode_byte = 0x1;break;      // forced
        case 'N':  mode_byte = 0x3;break;      // normal
        default:   mode_byte = 0x3;break;      // default normal
    }

    uint8_t ctrl_mea = (osrs_t<<5)|(osrs_p<<2)|mode_byte; // ctrl_meas register value
    
    switch(standby)
    {
        case 500:    standby_byte = 0x0; break;
        case 6250:   standby_byte = 0x1; break;
        case 12500:  standby_byte = 0x2; break;
        case 25000:  standby_byte = 0x3; break;
        case 50000:  standby_byte = 0x4; break;
        case 100000: standby_byte = 0x5; break;
        case 1000:   standby_byte = 0x6; break;
        case 2000:   standby_byte = 0x7; break;
        default:   standby_byte = 0x5; break;
    }

    switch(filter)
    {
        case 0:  filter_byte = 0x0; break;
        case 2:  filter_byte = 0x1; break;
        case 4:  filter_byte = 0x2; break;
        case 8:  filter_byte = 0x3; break;
        case 16: filter_byte = 0x4; break;
        default: filter_byte = 0x0; break;
    }

    uint8_t ctrl_cfg = (standby_byte<<5)|(filter_byte<<2)|0x0; // config register value


    HAL_I2C_Mem_Write(
        BME280_I2C, 
        BME280_I2C_ADDR, 
        BME280_RESET,           // 0xE0
        1, 
        &reset,
        1,                  
        500);

    HAL_Delay(100); // wait for reset to complete


    HAL_I2C_Mem_Write(
        BME280_I2C, 
        BME280_I2C_ADDR, 
        BME280_CTRL_HUMI, 
        1, 
        &ctrl_hum, 
        1, 
        500);

    HAL_I2C_Mem_Write(
        BME280_I2C, 
        BME280_I2C_ADDR, 
        BME280_CTRL_MEA, 
        1, 
        &ctrl_mea, 
        1, 
        500);

    HAL_I2C_Mem_Write(
        BME280_I2C, 
        BME280_I2C_ADDR, 
        BME280_CONFIG, 
        1, 
        &ctrl_cfg, 
        1, 
        500);
        
    #if IS_UART
        HAL_UART_Transmit(UART_DEBUG, (uint8_t*)"Device Configured\r\n", 19, 500);
    #endif

}

uint8_t BME280_Init(bme280_data_t *device){
    
    /* read chip id register to confirm the communication   */                               
    /* return 0x60 if succeeded                             */
  
    uint8_t id = 0;
    uint8_t init_flag = 0;
    

    HAL_I2C_Mem_Read(BME280_I2C, 
                     BME280_I2C_ADDR,   //i2c  device   addr
                     BME280_CHIP_ID,    //ID   register addr
                     1,                 //addr size
                     &id,               //id   buffer
                     1,                 //data size(id)
                     500);              //timeout

    if(id == 0x60){ // succeed in getting the chip id

        #if IS_UART

                HAL_UART_Transmit(UART_DEBUG, (uint8_t*)"device Chip ID: 0x60\r\n", 22, 500);
                HAL_UART_Transmit(UART_DEBUG, (uint8_t*)"Reading device Data...\r\n", 25, 500);
                HAL_UART_Transmit(UART_DEBUG, (uint8_t*)"======================\r\n", 25, 500);
            
        #endif

        /* get calibrbation data */
        
        BME280_GetCalibrationData(device);
        BME280_Config(4, 4, 4, 'N', 100000, 2); 
        init_flag = 1;

    }
    else
    {
        #if IS_UART
            HAL_UART_Transmit(UART_DEBUG, (uint8_t*)"Error: Cannot find device\r\n", 28, 500);
        #endif
    }
    return init_flag;  
}
void BME280_GetRawData(bme280_data_t *device)
{
    uint8_t buf[8] = {0}; // buffer for raw data read from sensor

    /* ready to read datas from sensor                 */
    /* from 0xF7 - OxFE burst read 8 bytes             */

    HAL_I2C_Mem_Read(BME280_I2C, 
                    BME280_I2C_ADDR, 
                    BME280_PRESSURE_MSB, 
                    1, 
                    buf,                   // 8 byte buffer for raw data
                    8,                     // pressure, temperature and humidity is 8 byte in total
                    500);

    /* raw data structure                                                       */
    /* for pressure and temperature:                                            */
    /* their raw data consist of 3 bytes: MSB,LSB and XLSB                      */
    /* XLSB only has 4 highest bits that are vaild                              */
    /* in order to create a 20 bit byte it needs to be shifted right by 4 bits  */

    /* humidity does not have XLSB                                              */

    device->press_raw  = (buf[0]<<12)|(buf[1]<<4)|(buf[2]>>4); //20 bits, 3bytes, 4bits discarded
	device->temp_raw   = (buf[3]<<12)|(buf[4]<<4)|(buf[5]>>4); //20 bits, 3bytes, 4bits discarded
	device->humi_raw   = (buf[6]<<8) |(buf[7]);                //16 bits, 2bytes
}

/*
 * @brief  The compensation algorithm of Temperature, Pressure and Humidity
 *         Don't modify it unless you know what you are doing
 *         Look up BME280 Datasheet for more information
*/ 

int32_t BME280_Compensation(bme280_data_t *device)
{
    int32_t tvar1,
            tvar2,
            t_fine,
            adc_T = device->temp_raw,
            temp_out;
    
    int64_t pvar1,
            pvar2,
            p,
            press_out;
    int32_t adc_P = device->press_raw;
            

    int32_t hvar,
            adc_H = device->humi_raw,
            humi_out;

    /* Temperature compensation */

    tvar1 = ((((adc_T>>3)  - ((int32_t)device->dig_T1<<1))) * ((int32_t)device->dig_T2)) >> 11;
    tvar2 = (((((adc_T>>4) - ((int32_t)device->dig_T1)) * ((adc_T>>4) - ((int32_t)device->dig_T1))) >> 12) * ((int32_t)device->dig_T3)) >> 14;
    t_fine = tvar1 + tvar2;
    temp_out = ((t_fine * 5 + 128) >> 8); 
    device->temperature = (temp_out / 100.0f);

    /* Pressure compensation */

    pvar1 = (int64_t)t_fine - 128000;
    pvar2 = pvar1 * pvar1 * (int64_t)device->dig_P6;
    pvar2 = pvar2 + ((pvar1 * (int64_t)device->dig_P5) << 17);
    pvar2 = pvar2 + (((int64_t)device->dig_P4) << 35);
    pvar1 = ((pvar1 * pvar1 * (int64_t)device->dig_P3) >> 8) +
            ((pvar1 * (int64_t)device->dig_P2) << 12);
    pvar1 = (((((int64_t)1) << 47) + pvar1)) * ((int64_t)device->dig_P1) >> 33;

    if (pvar1 == 0) return -1;  
    
    else
    {
        p  = 1048576 - adc_P;
        p  = (((p << 31) - pvar2) * 3125) / pvar1;
        pvar1 = (((int64_t)device->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        pvar2 = (((int64_t)device->dig_P8) * p) >> 19;
        p = ((p + pvar1 + pvar2) >> 8) + (((int64_t)device->dig_P7) << 4);

        press_out = (uint32_t)p;  
        device->pressure = (press_out / 256.0f / 100.0f) ; // to hPa
    }

    /* Humidity compensation */
    hvar = (t_fine - ((int32_t)76800));
	hvar = (((((adc_H << 14) - (((int32_t)device->dig_H4) << 20) - (((int32_t)device->dig_H5) *hvar)) + ((int32_t)16384)) >> 15) 
         * (((((((hvar * ((int32_t)device->dig_H6)) >> 10) * (((hvar * ((int32_t)device->dig_H3)) >> 11) 
         + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)device->dig_H2) + 8192) >> 14));
	hvar = (hvar - (((((hvar >> 15) * (hvar >> 15)) >> 7) * ((int32_t)device->dig_H1)) >> 4));
	hvar = (hvar < 0 ? 0 : hvar);
	hvar = (hvar > 419430400 ? 419430400 : hvar);

    humi_out = (uint32_t)(hvar>>12);
    device->humidity = (humi_out/1024.0f);

    device->altitude = (1-(float)pow((((device->pressure*100.0) / STD_PRESSURE)),(1.0/5.255f))) * 44330.0f;
    
    return 0;
}

void BME280_DataProc(bme280_data_t *device)
{
    static uint8_t isInit = 0;
    if(!isInit){
        if(BME280_Init(device))isInit = 1;

        else return;
    }
    /* Get raw data */
    BME280_GetRawData(device);

    /* Data compensation */
    BME280_Compensation(device);

    #if IS_UART

        char debug_buf[70];
        static uint8_t isFirst = 1;
        if(!isFirst)
        {
            
            sprintf(debug_buf, "T: %.2f C, P: %.2f hPa, H: %.2f %%, A: %.2f m\r\n", device->temperature, device->pressure, device->humidity,device->altitude);
            HAL_UART_Transmit(UART_DEBUG, (uint8_t*)debug_buf, strlen(debug_buf), 500);
        }
        isFirst = 0;

    #endif
}
