/*
 * ________________________________________________________________________________________________________
 * Copyright (c) 2020 GebraBit Inc. All rights reserved.
 *
 * This software, related documentation and any modifications thereto (collectively “Software”) is subject
 * to GebraBit and its licensors' intellectual property rights under U.S. and international copyright
 * and other intellectual property rights laws. 
 *
 * GebraBit and its licensors retain all intellectual property and proprietary rights in and to the Software
 * and any use, reproduction, disclosure or distribution of the Software without an express license agreement
 * from GebraBit is strictly prohibited.
 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT 
 * NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT IN  
 * NO EVENT SHALL GebraBit BE LIABLE FOR ANY DIRECT, SPECIAL, INDIRECT, INCIDENTAL, OR CONSEQUENTIAL DAMAGES, 
 * OR ANY DAMAGES WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN ACTION OF CONTRACT,
 * NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF OR IN CONNECTION WITH THE USE OR PERFORMANCE
 * OF THE SOFTWARE.
 * @Author       	: Mehrdad Zeinali
 * ________________________________________________________________________________________________________
 */
#ifndef	_DPS310__H_
#define	_DPS310__H_
#include "main.h"
#include "stdint.h"
#include "stdlib.h"
#include "stdio.h"
#include "spi.h"
#include "string.h"
#include "math.h"

/************************************************
 *              USER REGISTER MAP               *
 ***********************************************/ 
#define DPS310_PSR_B2                  			(0x00)
#define DPS310_PSR_B1                       (0x01)
#define DPS310_PSR_B0                       (0x02)
#define DPS310_TMP_B2                       (0x03)
#define DPS310_TMP_B1                       (0x04)
#define DPS310_TMP_B0                				(0x05)
#define DPS310_PRS_CFG                			(0x06)
#define DPS310_TMP_CFG                			(0x07)
#define DPS310_MEAS_CFG             				(0x08)
#define DPS310_CFG_REG             					(0x09)
#define DPS310_INT_STS             					(0x0A)
#define DPS310_FIFO_STS                   	(0x0B)
#define DPS310_RESET                   			(0x0C)
#define DPS310_PRODUCT_ID                   (0x0D)
#define DPS310_COEF                         (0x10)//0x10 TO 0x21
#define DPS310_COEF_SRCE                    (0x28)
/*----------------------------------------------*
 *           USER REGISTER MAP End              *
 *----------------------------------------------*/ 

/************************************************
 *         MSB Bit Start Location Begin         *
 ***********************************************/ 
#define START_MSB_BIT_AT_0                    0
#define START_MSB_BIT_AT_1                    1
#define START_MSB_BIT_AT_2                    2
#define START_MSB_BIT_AT_3                    3
#define START_MSB_BIT_AT_4                    4
#define START_MSB_BIT_AT_5                    5
#define START_MSB_BIT_AT_6                    6
#define START_MSB_BIT_AT_7                    7
/*----------------------------------------------*
 *        MSB Bit Start Location End            *
 *----------------------------------------------*/ 
/************************************************
 *          Bit Field Length Begin              *
 ***********************************************/ 
#define BIT_LENGTH_1                          1
#define BIT_LENGTH_2                          2
#define BIT_LENGTH_3                          3
#define BIT_LENGTH_4                          4
#define BIT_LENGTH_5                          5
#define BIT_LENGTH_6                          6
#define BIT_LENGTH_7                          7
#define BIT_LENGTH_8                          8
/*----------------------------------------------*
 *          Bit Field Length End                *
 *----------------------------------------------*/
 /************************************************
 *          Register Values Begin                *
 ***********************************************/ 
#define DPS310_SOFT_RESET_CMD                 9
#define DPS310_FIFO_FLUSH_CMD                 11
#define COEFFICIENT_DATA_BUFFER_SIZE          18
#define REGISTER_RAW_DATA_BUFFER_SIZE         6
#define PRESSURE_REGISTER_QTY                 3
#define TEMPERATURE_REGISTER_QTY              3
#define FIFO_DATA_BUFFER_SIZE         				96
#define BYTE_QTY_IN_ONE_PACKET        				3
#define PRESSURE_DATA_BUFFER_SIZE         	  17
#define TEMPERATURE_DATA_BUFFER_SIZE         	17
#define SEA_LEVEL_PRESSURE										101325
/*----------------------------------------------*
 *           Register Values End                *
 *----------------------------------------------*/
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */

/******************************************************
 *          Values For FIFO Status  Register          *
 ******************************************************/ 
typedef enum FIFO_Status
{  
 FIFO_EMPTY    = 0x01 ,
 FIFO_FULL     = 0x02    						
}DPS310_FIFO_Status;
/******************************************************
*         Values For CFG_REG [6:4]  Register          *
 ******************************************************/ 
typedef enum Interrupt
{
 PRESS_INTERRUPT     						= 1 ,                  
 TEMP_INTERRUPT   							= 2 ,	
 PRESS_TEMP_INTERRUPT 					= 3 ,	
 FIFO_FULL_INTERRUPT  					= 4 ,									
 PRESS_FIFO_FULL_INTERRUPT 			= 5 ,                  
 TEMP_FIFO_FULL_INTERRUPT       = 6 ,
 PRESS_TEMP_FIFO_FULL_INTERRUPT = 7
} DPS310_Interrupt;
/**************************************************
 *          Values For INT_STS  Register          *
 **************************************************/ 
typedef enum Interrupt_Status
{  
 PRESSURE_MESUREMENT_INTERRUPT      = 0x01 ,                   
 TEMPERATURE_MESUREMENT_INTERRUPT   = 0x02 ,                
 FIFO_IS_FULL_INTERRUPT             = 0x04                      
}DPS310_Interrupt_Status;
/**************************************************
 *     Values For Disable And Enable Functions    *
 **************************************************/ 
typedef enum Ability 
{  
	Disable = 0     ,                      
	Enable     
}DPS310_Ability;        
/**************************************************
 *     Values For MEAS_CTRL In MEAS_CFG Register  *
 **************************************************/ 
typedef enum Measurement_Mode
{
	STANDBY  																	 = 0 ,        							
	COMMAND_PRESSURE 												   = 1 , 
  COMMAND_TEMPERATURE 											 = 2 ,	
	CONTINUOUS_BACKGROUND_PRESSURE						 = 5 ,
	CONTINUOUS_BACKGROUND_TEMPERATURE          = 6 ,
	CONTINUOUS_BACKGROUND_PRESSURE_TEMPERATURE = 7
} DPS310_Measurement_Mode;
/**************************************************************
 *         						Values For  Oversampling     				   *
 **************************************************************/ 
typedef enum Oversampling
{
	  SINGLE     = 0 ,										
	 _2_TIMES    = 1 , 										
	 _4_TIMES    = 2 ,										 
	 _8_TIMES    = 3 ,                   
	 _16_TIMES   = 4 ,											
	 _32_TIMES   = 5 ,		                    
	 _64_TIMES   = 6 ,
	 _128_TIMES  = 7 ,
} DPS310_Oversampling;
/**************************************************************
 *        					Values For Measurement Rate 				      *
 **************************************************************/ 
typedef enum Measurement_Rate
{
	 _1_PER_SECOND         = 0 ,                  
	 _2_PER_SECOND         = 1 ,									
	 _4_PER_SECOND         = 2 ,									
	 _8_PER_SECOND         = 3 ,
	 _16_PER_SECOND        = 4 ,                  
	 _32_PER_SECOND        = 5 ,									
	 _64_PER_SECOND        = 6 ,									
	 _128_PER_SECOND       = 7 
} DPS310_Measurement_Rate;
/**************************************************************
 *        					Values For Scale Factor 						      *
 **************************************************************/ 
typedef enum Scale_Factor
{
	 SF_524288_KP_KT       = 0 ,                  
	 SF_1572864_KP_KT      = 1 ,									
	 SF_3670016_KP_KT      = 2 ,									
	 SF_7864320_KP_KT      = 3 ,
	 SF_253952_KP_KT       = 4 ,                  
	 SF_516096_KP_KT       = 5 ,									
	 SF_1040384_KP_KT      = 6 ,									
	 SF_2088960_KP_KT      = 7 
} DPS310_Scale_Factor;

/******************************************************
*         Values For  CFG_REG[3:2]  Register          *
 ******************************************************/ 
typedef enum Bit_Shift
{ 
  NO_SHIFT = 0 ,	                                 
	SHIFT_BIT                     								      															
}DPS310_Bit_Shift;
/*************************************************
 *         Values For Data Preparation           *
 **************************************************/ 
typedef enum Preparation
{  
	IS_NOT_Ready = 0 ,                      
	IS_Ready     = 1 ,
  TEMPERATURE_PRESSURE_IS_READY	= 3
}DPS310_Preparation;
/*************************************************
 *       Values For Coefficient Status           *
 **************************************************/ 
typedef enum Coefficient_Status
{  
	COEFFICIENT_ARE_NOT_AVAILABLE = 0 ,                      
	COEFFICIENT_ARE_AVAILABLE     
}DPS310_Coefficient_Status;
/*************************************************
 *    Values For Sensor Initialization           *
 **************************************************/ 
typedef enum Sensor_Initialization 
{  
	INITIALIZATION_NOT_COMPLETE = 0     ,                      
	INITIALIZATION_COMPLETE     
}DPS310_Sensor_Initialization;
/*************************************************
 *           Values For Reset Process             *
 **************************************************/ 
typedef enum 
{  
	DONE     = 0     ,                      
	FAILED   = 1    
}DPS310_Reset_Status;
/**************************************************
 *       Values For Disable And Enable FIFO       *
 **************************************************/ 
typedef enum FIFO_Ability
{  
	FIFO_DISABLE = 0     ,                      
	FIFO_ENABLE     
} DPS310_FIFO_Ability;

/**************************************************
 * Values For Methode of getting data from sensor *
 **************************************************/ 
typedef enum Get_DATA
{  
	FROM_REGISTER = 0     ,                      
	FROM_FIFO     
} DPS310_Get_DATA; 
/*******************************************************
 *  		      Values For Temperature Sensor  			  	 *
 *******************************************************/ 
typedef enum Temperature_Sensor
{  
	INTERNAL_SENSOR = 0 ,                      
	EXTERNAL_SENSOR     
} DPS310_Temperature_Sensor; 
 /*************************************************
 *  Defining DPS310 Register & Data As Struct   *
 **************************************************/
typedef	struct DPS310
{
	  uint8_t                       	Register_Cache;
	  DPS310_Get_DATA             		GET_DATA;
	  DPS310_Reset_Status         		RESET;
	  uint8_t                       	PRODUCT_ID;
		uint8_t                       	REVISION_ID;
		DPS310_Sensor_Initialization    SENSOR_INITIALIZATION;
	  DPS310_Coefficient_Status       COEFFICIENT_STATUS;
		DPS310_Temperature_Sensor			  TEMPERATURE_COEFFICIENT;
	  DPS310_Temperature_Sensor       TEMPERATURE;
		DPS310_Ability 									TEMPERATURE_CORRECTION;
		DPS310_Measurement_Mode					MEASUREMENT_MODE;
	  DPS310_Measurement_Rate					PRESSURE_RATE;
   	DPS310_Oversampling             PRESSURE_OVERSAMPLING;
		DPS310_Scale_Factor             PRESSURE_SCALE_FACTOR;
		uint64_t												PRESSURE_SF_VALUE;
	  DPS310_Bit_Shift								PRESSURE_BITSHIFT;
		DPS310_Measurement_Rate					TEMPERATURE_RATE;
   	DPS310_Oversampling             TEMPERATURE_OVERSAMPLING;
		DPS310_Scale_Factor             TEMPERATURE_SCALE_FACTOR;
		uint64_t												TEMPERATURE_SF_VALUE;
		DPS310_Bit_Shift								TEMPERATURE_BITSHIFT;
		DPS310_Interrupt								INTERRUPT;
  	DPS310_Interrupt_Status					INTERRUPT_STATUS;
	  DPS310_Preparation							TEMPERATURE_DATA;
	  DPS310_Preparation							PRESSURE_DATA;
	  DPS310_Preparation							TEMP_PRESS_DATA;
	  DPS310_Ability									FIFO;
	  uint8_t													PACKET_QTY_IN_FULL_FIFO ;
		DPS310_FIFO_Status							FIFO_STATUS;
		uint8_t 												CALIBRATION_COEFFICIENT_DATA[COEFFICIENT_DATA_BUFFER_SIZE];
		int32_t                         C0;
		int32_t                         C1;
		int32_t                         C00;
		int32_t                         C10;
		int32_t                         C01;
		int32_t                         C11;
	  int32_t                         C20;
		int32_t                         C21;
		int32_t                         C30;
		uint8_t 												REGISTER_RAW_DATA_BUFFER[REGISTER_RAW_DATA_BUFFER_SIZE];
		int32_t 												REGISTER_RAW_TEMPERATURE;
	  int32_t 											  REGISTER_RAW_PRESSURE;
		float 													ALTITUDE;
		float 													COMPENSATED_PRESSURE;
		float 													COMPENSATED_TEMPERATURE;
		uint8_t 												FIFO_DATA[FIFO_DATA_BUFFER_SIZE];
		float												    COMPENSATED_FIFO_TEMPERATURE[TEMPERATURE_DATA_BUFFER_SIZE];
		float												    COMPENSATED_FIFO_PRESSURE[PRESSURE_DATA_BUFFER_SIZE];
		float												    FIFO_ALTITUDE[PRESSURE_DATA_BUFFER_SIZE];
}GebraBit_DPS310;
/*
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
:::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::::: */
/********************************************************
 *Declare Read&Write DPS310 Register Values Functions *
 ********************************************************/
extern	uint8_t	GB_DPS310_Read_Reg_Data ( uint8_t regAddr,uint8_t* data);
extern	uint8_t GB_DPS310_Read_Reg_Bits (uint8_t regAddr,uint8_t start_bit, uint8_t len, uint8_t* data);
extern	uint8_t GB_DPS310_Burst_Read(uint8_t regAddr,uint8_t *data, uint16_t byteQuantity);
extern	uint8_t GB_DPS310_Write_Reg_Data(uint8_t regAddr, uint8_t data);
extern	uint8_t	GB_DPS310_Write_Reg_Bits(uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t data);
extern	uint8_t GB_DPS310_Burst_Write		( uint8_t regAddr,uint8_t *data, 	uint16_t byteQuantity);
/********************************************************
 *       Declare DPS310 Configuration Functions       *
 ********************************************************/
extern void GB_DPS310_Pressure_Measurement_Rate(GebraBit_DPS310 * DPS310 , DPS310_Measurement_Rate rate );
extern void GB_DPS310_Pressure_OverSampling(GebraBit_DPS310 * DPS310 , DPS310_Oversampling oversmp );
extern void GB_DPS310_Temperature_Measurement_Rate(GebraBit_DPS310 * DPS310 , DPS310_Measurement_Rate rate );
extern void GB_DPS310_Temperature_OverSampling(GebraBit_DPS310 * DPS310 , DPS310_Oversampling oversmp );
extern void GB_DPS310_Temperature(GebraBit_DPS310 * DPS310 , DPS310_Temperature_Sensor tmp );
extern void GB_DPS310_Check_Coefficient(GebraBit_DPS310 * DPS310);
extern void GB_DPS310_Sensor_Initialization(GebraBit_DPS310 * DPS310);
extern void GB_DPS310_Check_Temperature_Data(GebraBit_DPS310 * DPS310);
extern void GB_DPS310_Check_Pressure_Data(GebraBit_DPS310 * DPS310);
extern void GB_DPS310_Check_Temperature_Pressure_Data(GebraBit_DPS310 * DPS310);
extern void GB_DPS310_Measurement_Mode(GebraBit_DPS310 * DPS310 , DPS310_Measurement_Mode meas);
extern void GB_DPS310_Temperature_Result_BitShift(GebraBit_DPS310 * DPS310 , DPS310_Bit_Shift shift);
extern void GB_DPS310_Pressure_Result_BitShift(GebraBit_DPS310 * DPS310 , DPS310_Bit_Shift shift);
extern void GB_DPS310_Interrupt(GebraBit_DPS310 * DPS310 , DPS310_Interrupt intrupt);
extern void GB_DPS310_Check_Interrupt_Status(GebraBit_DPS310 * DPS310 );
extern void GB_DPS310_Soft_Reset(GebraBit_DPS310 * DPS310 );
extern void GB_DPS310_Product_ID(GebraBit_DPS310 * DPS310 );
extern void GB_DPS310_Revision_ID(GebraBit_DPS310 * DPS310 );
extern void GB_DPS310_Check_Temperature_Coefficient_Source(GebraBit_DPS310 * DPS310 );
/********************************************************
 *          Declare DPS310 FIFO Functions             *
 ********************************************************/
extern void GB_DPS310_FIFO(GebraBit_DPS310 * DPS310 , DPS310_Ability fifo);
extern void GB_DPS310_FIFO_Configuration ( GebraBit_DPS310 * DPS310 , DPS310_FIFO_Ability fifo  );
extern void GB_DPS310_Check_FIFO_Status(GebraBit_DPS310 * DPS310 );
extern void GB_DPS310_FIFO_Flush(GebraBit_DPS310 * DPS310 );
extern void GB_DPS310_Read_FIFO(GebraBit_DPS310 * DPS310);
extern void GB_DPS310_FIFO_Data_Partition_Pressure_Temperature(GebraBit_DPS310 * DPS310);
/********************************************************
 *          Declare DPS310 DATA Functions             *
 ********************************************************/
extern void GB_DPS310_Twos_Complement_Converter(int32_t *value, uint8_t length);
extern void GB_DPS310_Calculate_Calibration_Coefficients(GebraBit_DPS310 * DPS310);
extern void GB_DPS310_Get_Register_Raw_Pressure_Temperature(GebraBit_DPS310 * DPS310 );
extern void GB_DPS310_Calculate_Compensated_Pressure(GebraBit_DPS310 * DPS310);
extern void GB_DPS310_Calculate_Compensated_Temperature(GebraBit_DPS310 * DPS310)	;
extern void GB_DPS310_Get_Data(GebraBit_DPS310 * DPS310 , DPS310_Get_DATA get_data) ;
extern void GB_DPS310_Temperature_Correction(GebraBit_DPS310 * DPS310);
extern void GB_DPS310_Altitude(GebraBit_DPS310 * DPS310);
///********************************************************
// *          Declare DPS310 HIGH LEVEL Functions       *
// ********************************************************/
extern void GB_DPS310_Initialize( GebraBit_DPS310 * DPS310 );
extern void GB_DPS310_Configuration(GebraBit_DPS310 * DPS310, DPS310_FIFO_Ability fifo);

#endif
