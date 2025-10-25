
#include "BME280.h"
#include <cmath>

using namespace BME280;
using namespace GENERAL;

bool INIT::Init(CONFIG::MODE mode  ,CONFIG::OSRS osrs_t,CONFIG::OSRS    osrs_p,
                CONFIG::OSRS osrs_h,CONFIG::OSRS filter,CONFIG::STANDBY standby)
{

	if     (state == STATE::DONE)return true;
	else if(state == STATE::ID)
	{
		if (REGISTER::Read(REGISTER::ID_REG,&chip_id,1)!=HAL_OK)return DEBUGGING::Print("INIT:I2C ERROR\r\n")    ,false;
		if (chip_id != REGISTER::ID)                            return DEBUGGING::Print("INIT:WRONG CHIP ID\r\n"),false;
		state = STATE::CALIB;
	}
	else if(state == STATE::CALIB) state = (CALIB:: Get_Data())?(STATE::CONFIG):(STATE::CALIB);
	else if(state == STATE::CONFIG)state = (CONFIG::Setting(mode,osrs_t,osrs_p,osrs_h,filter,standby)
										                ?(STATE::DONE):(STATE::CONFIG));
	return (state == STATE::DONE);
}

bool CALIB::Get_Data()
{
	if   (state == STATE::DONE)return true;
	else if (state == STATE::CALIB1)
	{
		if (REGISTER::Read(REGISTER::CALIB_REG1,buffer,24)!=HAL_OK)return DEBUGGING::Print("CA1:I2C ERROR\r\n"),false;
		state = STATE::CALIB2;
	}
	else if(state == STATE::CALIB2)
	{
		if (REGISTER::Read(REGISTER::CALIB_REG2,buffer+24,1)!=HAL_OK)return DEBUGGING::Print("CA2:I2C ERROR\r\n"),false;
		state = STATE::CALIB3;
	}
	else if(state == STATE::CALIB3)
	{
		if (REGISTER::Read(REGISTER::CALIB_REG3,buffer+25,7)!=HAL_OK)return DEBUGGING::Print("CA3:I2C ERROR\r\n"),false;
		state = STATE::WRITE;
	}
	else if(state == STATE::WRITE)
	{
		dig_T1 = static_cast<uint16_t>((buffer[1]  << 8)|(buffer[0]));
		dig_T2 = static_cast<int16_t> ((buffer[3]  << 8)|(buffer[2]));
		dig_T3 = static_cast<int16_t> ((buffer[5]  << 8)|(buffer[4]));

		dig_P1 = static_cast<uint16_t>((buffer[7]  << 8)|(buffer[6]));
		dig_P2 = static_cast<int16_t> ((buffer[9]  << 8)|(buffer[8]));
		dig_P3 = static_cast<int16_t> ((buffer[11] << 8)|(buffer[10]));
		dig_P4 = static_cast<int16_t> ((buffer[13] << 8)|(buffer[12]));
		dig_P5 = static_cast<int16_t> ((buffer[15] << 8)|(buffer[14]));
		dig_P6 = static_cast<int16_t> ((buffer[17] << 8)|(buffer[16]));
		dig_P7 = static_cast<int16_t> ((buffer[19] << 8)|(buffer[18]));
		dig_P8 = static_cast<int16_t> ((buffer[21] << 8)|(buffer[20]));
		dig_P9 = static_cast<int16_t> ((buffer[23] << 8)|(buffer[22]));

		dig_H1 = buffer[24];
		dig_H2 = static_cast<int16_t> ((buffer[26] << 8)|(buffer[25])); //0xE1
		dig_H3 = buffer[27];
		dig_H4 = static_cast<int16_t> ((buffer[28] << 4)|(buffer[29] & 0x0F));
		dig_H5 = static_cast<int16_t> ((buffer[30] << 4)|(buffer[29] >>   4));
		dig_H6 = static_cast<int8_t>   (buffer[31]);

		state = STATE::DONE;
		return true;
	}
	return false;
}

bool CONFIG::Setting(MODE mode  ,OSRS osrs_t,OSRS osrs_p,
					 OSRS osrs_h,OSRS filter,STANDBY standby)
{
	const auto mode_val= (mode == MODE::NORMAL)?static_cast<uint8_t>(0x3):static_cast<uint8_t>(mode),
	           osrs_t_val = static_cast<uint8_t>(osrs_t),
		       osrs_p_val = static_cast<uint8_t>(osrs_p),
		       osrs_h_val = static_cast<uint8_t>(osrs_h),
		       filter_val = static_cast<uint8_t>(filter),
		       standby_val= static_cast<uint8_t>(standby);

	if   (state == STATE::DONE)return true;
	else if (state == STATE::RESET)
	{
		/*? Just reset it, whatever */
		if (REGISTER::Write(REGISTER::RESET_REG,REGISTER::RESET)!=HAL_OK)
			return DEBUGGING::Print("CFGRESET:I2C ERROR\r\n"),false;
		state = STATE::CTRL_HUM;
	}
	else if (state == STATE::CTRL_HUM)
	{
		if (REGISTER::Write(REGISTER::CTRL_HUM_REG,osrs_h_val)!=HAL_OK)
			return DEBUGGING::Print("CFGHUM:I2C ERROR\r\n"),false;
		state = STATE::CTRL_MEAS;
	}
	else if (state == STATE::CTRL_MEAS)
	{
		if (REGISTER::Write(REGISTER::CTRL_MEA_REG,
		                   (osrs_t_val<<5|osrs_p_val<<2|mode_val))!=HAL_OK)
			return DEBUGGING::Print("CFGMEA:I2C ERROR\r\n"),false;
		state = STATE::REG;
	}
	else if(state == STATE::REG)
	{
		if (REGISTER::Write(REGISTER::CONFIG_REG,
		                   (standby_val<<5|filter_val<<2|0x0))!=HAL_OK)
			return DEBUGGING::Print("CFGREG:I2C ERROR\r\n"),false;
		state = STATE::DONE;
		return true;
	}
	return false;
}

bool Proc::Compensation()
{
	if (REGISTER::Read(REGISTER::DATA_REG,buffer,8)!=HAL_OK)return DEBUGGING::Print("PROC:I2C ERROR\r\n"),false;

	pres_raw = (buffer[0]<<12)|(buffer[1]<<4)|(buffer[2]>>4);
	temp_raw = (buffer[3]<<12)|(buffer[4]<<4)|(buffer[5]>>4);
	hum_raw  = (buffer[6]<<8) |(buffer[7]);

	adc_H = hum_raw;
	adc_P = pres_raw;
	adc_T = temp_raw;

	tvar1 = (((( adc_T>>3)  - ((int32_t)CALIB::dig_T1<<1))) * ((int32_t)CALIB::dig_T2)) >> 11;
    tvar2 = (((((adc_T>>4)  - ((int32_t)CALIB::dig_T1)) * ((adc_T>>4) - ((int32_t)CALIB::dig_T1))) >> 12) * ((int32_t)CALIB::dig_T3)) >> 14;
    t_fine = tvar1 + tvar2;
    temp_out = ((t_fine * 5 + 128) >> 8);
    temperature = (temp_out / 100.0f);

    /* Pressure compensation */

    pvar1 = (int64_t)t_fine - 128000;
    pvar2 = pvar1 * pvar1 * (int64_t)CALIB::dig_P6;
    pvar2 = pvar2 + ((pvar1 * (int64_t)CALIB::dig_P5) << 17);
    pvar2 = pvar2 + (((int64_t)CALIB::dig_P4) << 35);
    pvar1 = ((pvar1 * pvar1 * (int64_t)CALIB::dig_P3) >> 8)+((pvar1 * (int64_t)CALIB::dig_P2) << 12);
    pvar1 = (((((int64_t)1) << 47) + pvar1)) * ((int64_t)CALIB::dig_P1) >> 33;

    if (pvar1 == 0) return false;

    else
    {
        p_fine  = 1048576 - adc_P;
        p_fine  = (((p_fine << 31) - pvar2) * 3125) / pvar1;
        pvar1 = (((int64_t)CALIB::dig_P9) * (p_fine >> 13) * (p_fine >> 13)) >> 25;
        pvar2 = (((int64_t)CALIB::dig_P8) * p_fine) >> 19;
        p_fine = ((p_fine + pvar1 + pvar2) >> 8) + (((int64_t)CALIB::dig_P7) << 4);

        pres_out = (uint32_t)p_fine;
        pressure = (pres_out / 256.0f / 100.0f) ; // to hPa
    }

    /* Humidity compensation */
    hvar = (t_fine - ((int32_t)76800));
	hvar = (((((adc_H << 14) - (((int32_t)CALIB::dig_H4) << 20) - (((int32_t)CALIB::dig_H5) *hvar)) + ((int32_t)16384)) >> 15)
         * (((((((hvar * ((int32_t)CALIB::dig_H6)) >> 10) * (((hvar * ((int32_t)CALIB::dig_H3)) >> 11)
         + ((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)CALIB::dig_H2) + 8192) >> 14));
	hvar = (hvar - (((((hvar >> 15) * (hvar >> 15)) >> 7) * ((int32_t)CALIB::dig_H1)) >> 4));
	hvar = (hvar < 0 ? 0 : hvar);
	hvar = (hvar > 419430400 ? 419430400 : hvar);

    hum_out  = (uint32_t)(hvar>>12);
    humidity = (hum_out/1024.0f);

    altitude = (1-(float)pow((((pressure*100.0) / STD_PRESSURE)),(1.0/5.255f))) * 44330.0f;

	DEBUGGING::Print("T: %.2f C P: %.2f hPa H: %.2f %% Alt: %.2f m\r\n",
					 temperature,pressure,humidity,altitude);

    return true;
}

