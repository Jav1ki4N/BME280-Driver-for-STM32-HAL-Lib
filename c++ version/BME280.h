
/* *Temperature, Humidity & Pressure Sensor */
/* *Botsch BME280						    */
/* *Simplified Ver. By @i4N,2025			*/

#pragma once
#include <cstdarg>   // for va_list, if needed
#include <utility>
#include <cstdint>
#include <cstdio>    // for std::snprintf
#include <cstring>
#include "i2c.h"
#include "usart.h"

/*$---------------*/
/*$ BME280 SENSOR */
/*$---------------*/
namespace BME280
{
	/*$-----------------*/
	/*$ GENERAL PURPOSE */
	namespace GENERAL
	{
		namespace REGISTER{ inline constexpr uint8_t DEVICE_ADDR = 0x76 << 1;} //? meant to be in Line 61 but
		/*$ PORTS AND PINS */												   //? It's used in Line 31
		namespace GPIO
		{
			inline auto HI2CX = &hi2c1;
			inline auto PORT    = GPIOB;
			inline auto PIN_SCL = GPIO_PIN_6;
			inline auto PIN_SDA = GPIO_PIN_7;
			inline auto ADDR = REGISTER::DEVICE_ADDR;                    //? Device address
		}

		/*$ DEBUGGING WITH UART */
		namespace DEBUGGING
		{
			inline           bool     isDebug = true;
			inline           auto     HUARTX = &huart1;
			inline constexpr uint16_t MAX_LENGTH        {64};
			inline           char     buffer[MAX_LENGTH]{  };
			inline constexpr uint32_t TIMEOUT = 1000;

			/*f Function: DEBUGGING::Print      */
			/*? Print formatted string via UART */

			template<typename... Args>
			bool Print( const char* format, Args... args)
			{
				const int length = std::snprintf(buffer, MAX_LENGTH, format, +args...);
				if (length <= 0 || !isDebug) return false;

				auto length_to_send = (length >= MAX_LENGTH) ?
				                              MAX_LENGTH - 1 : static_cast<size_t>(length);

				HAL_UART_Transmit(HUARTX,
							      reinterpret_cast<uint8_t*>(buffer),
							      static_cast<uint16_t>(length_to_send),
							      TIMEOUT);
				return true;
			}
		}

		/*$ Registers & fixed data */
		namespace REGISTER
		{
			/* Device */
			//inline constexpr uint8_t DEVICE_ADDR   = 0x76 << 1;
			/* ? ↑ See Line 17 */
			inline constexpr uint8_t DEVICE_ADDR_ALT = 0x77 << 1; //? If SDO is grounded manually
			/* ID */
			inline constexpr uint8_t ID_REG          = 0XD0;
			inline constexpr uint8_t ID              = 0X60;
			/* CALIBRATION */
			inline constexpr uint8_t CALIB_REG1      = 0x88;
			inline constexpr uint8_t CALIB_REG2      = 0xA1;
			inline constexpr uint8_t CALIB_REG3      = 0xE1;
			/* CONFIGURATION */
			inline constexpr uint8_t RESET_REG       = 0xE0;
			inline constexpr uint8_t RESET           = 0xB6;
			inline constexpr uint8_t CTRL_HUM_REG    = 0xF2;
			inline constexpr uint8_t CTRL_MEA_REG    = 0xF4;
			inline constexpr uint8_t CONFIG_REG      = 0xF5;
			/* DATA */
			inline constexpr uint8_t DATA_REG = 0xF7;

			/*f Function: REGISTER::Read */
			/*? from reg read read_size data and storage them in pData  */
			/*? return I2C state for debugging							*/

			inline auto Read(uint16_t reg,uint8_t* pData,
							 uint16_t read_size)
			{
				return HAL_I2C_Mem_Read(GPIO::HI2CX,GPIO::ADDR, //? which is REGISTER::DEVICE_ADDR
									    reg,1,pData,read_size,DEBUGGING::TIMEOUT);
			}

			/*f Function: REGISTER::Write */
			/*? write */

			inline auto Write(uint8_t reg,uint8_t pData)
			{
				return HAL_I2C_Mem_Write(GPIO::HI2CX,GPIO::ADDR,
				                         reg,1,&pData,1,DEBUGGING::TIMEOUT); //? 1 byte of reg address to write in
			}													     	     //? 1 byte of data size to write
		}
	}

	namespace CALIB
	{
		enum class STATE : uint8_t
		{CALIB1,CALIB2,CALIB3,WRITE,DONE};
		inline auto state = STATE::CALIB1;

		inline uint8_t buffer[32]{0};

		/* temperature calibration parameters */
		inline uint16_t dig_T1{0};
		inline int16_t  dig_T2{0}, dig_T3{0};

		/*  pressure calibration parameters */
		inline uint16_t dig_P1{0};
		inline int16_t  dig_P2{0}, dig_P3{0}, dig_P4{0}, dig_P5{0},
		                dig_P6{0}, dig_P7{0}, dig_P8{0}, dig_P9{0};

		/*  humidity calibration parameters */
		inline uint8_t  dig_H1{0},dig_H3{0};
		inline int16_t  dig_H2{0},dig_H4{0}, dig_H5{0};
		inline int8_t   dig_H6{0};

		bool Get_Data();
	}

	namespace CONFIG
	{
		enum class STATE : uint8_t
		{RESET,CTRL_HUM,CTRL_MEAS,REG,DONE};
		inline auto state = STATE::RESET;

		enum class MODE  : uint8_t
		{SLEEP,FORCED,NORMAL};

		enum class OSRS : uint8_t
		{x0,x1,x2,x4,x8,x16};

		enum class STANDBY : uint8_t
		{ms0_5,ms62_5,ms125,ms250,ms500,ms1000,ms10,ms20};

		bool Setting(MODE mode  ,OSRS osrs_t,OSRS osrs_p,
				     OSRS osrs_h,OSRS filter,STANDBY standby);
	}

	namespace INIT
	{
		enum class STATE : uint8_t
		{ID,CALIB,CONFIG,DONE};

		inline auto state = INIT::STATE::ID; //* default started from Getting ID

		inline uint8_t chip_id{0};

		bool Init(CONFIG::MODE    mode    = CONFIG::MODE::NORMAL,
				  CONFIG::OSRS    osrs_t  = CONFIG::OSRS::x4,
			      CONFIG::OSRS    osrs_p  = CONFIG::OSRS::x4,
				  CONFIG::OSRS    osrs_h  = CONFIG::OSRS::x4,
				  CONFIG::OSRS    filter  = CONFIG::OSRS::x2,
				  CONFIG::STANDBY standby = CONFIG::STANDBY::ms1000); //? default using these parameters

	}

	namespace Proc
	{
		inline int32_t temp_raw   {0},pres_raw{0},hum_raw {0};
		inline float   temperature{0.0f},pressure{0.0f},humidity{0.0f},altitude{0.0f};
		inline uint8_t buffer[8]{};

		inline int32_t tvar1{},tvar2{},t_fine{},adc_T{}   ,temp_out{};
		inline int64_t pvar1{},pvar2{},p_fine{},pres_out{};
		inline int32_t adc_P{};
		inline int32_t hvar{} ,adc_H{},hum_out{};

		static constexpr float STD_PRESSURE = 101325.0f;

		bool Compensation();
	}
}

/*$ BME280 WIRING 			*/
/*? VCC/GND - 3V3/GND  		*/
/*? SCL - PB6 				*/
/*? SDA - PB7 				*/
/*? CSB - VCC 				*/
/*? SDO - VOID OR GROUNDED  */





