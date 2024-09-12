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
 
#include "GebraBit_DPS310.h"
#include <math.h>
extern SPI_HandleTypeDef hspi1;
	
/*=========================================================================================================================================
 * @brief     Read data from spacial register.
 * @param     regAddr Register Address of DPS310
 * @param     data    Pointer to Variable that register value is saved .
 * @return    stat    Return status
 ========================================================================================================================================*/
uint8_t	GB_DPS310_Read_Reg_Data ( uint8_t regAddr, uint8_t* data)
{	
	uint8_t txBuf[2] = {regAddr|0x80 , 0xFF}; //Read operation: set the 8th-bit to 1.
	uint8_t rxBuf[2];
	HAL_StatusTypeDef stat = HAL_ERROR ;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	stat = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	if (stat == HAL_OK)
	{
		*data = rxBuf[1];
	}
	return stat;
}
/*========================================================================================================================================= 
 * @brief     Read data from spacial bits of a register.
 * @param     regAddr     Register Address of DPS310 .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to read(1 to 8) 
 * @param     data        Pointer to Variable that register Bits value is saved .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_DPS310_Read_Reg_Bits (uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t* data)
{
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;
	
	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}

	if (GB_DPS310_Read_Reg_Data( regAddr, &tempData) == HAL_OK)
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1); //formula for making a broom of 1&0 for gathering desired bits
		tempData &= mask; // zero all non-important bits in data
		tempData >>= (start_bit - len + 1); //shift data to zero position
		*data = tempData;
		status = HAL_OK;
	}
	else
	{
		status = HAL_ERROR;
		*data = 0;
	}
	return status;
}
/*========================================================================================================================================= 
 * @brief     Read multiple data from first spacial register address.
 * @param     regAddr First Register Address of DPS310 that reading multiple data start from this address
 * @param     data    Pointer to Variable that multiple data is saved .
 * @param     byteQuantity Quantity of data that we want to read .
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_DPS310_Burst_Read(uint8_t regAddr,  uint8_t *data, uint16_t byteQuantity)
{
	uint8_t *pTxBuf;
	uint8_t *pRxBuf;
	uint8_t status = HAL_ERROR;
	pTxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1)); // reason of "+1" is for register address that comes in first byte
	pRxBuf = ( uint8_t * )malloc(sizeof(uint8_t) * (byteQuantity + 1));
	memset(pTxBuf, 0, (byteQuantity + 1)*sizeof(uint8_t));

	pTxBuf[0] = regAddr | 0x80; //Read operation: set the 8th-bit to 1.

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, pTxBuf, pRxBuf, byteQuantity+1, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	if (status == HAL_OK)
	{
		memcpy(data, &pRxBuf[1], byteQuantity*sizeof(uint8_t)); //here we dont have "+1" beacause we don't need first byte that was register data , we just need DATA itself
	}
	free(pTxBuf);
	free(pRxBuf);
	return status;
}
/*=========================================================================================================================================
 * @brief     Write data to spacial register.
 * @param     regAddr Register Address of DPS310
 * @param     data    Value that will be writen to register .
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_DPS310_Write_Reg_Data(uint8_t regAddr, uint8_t data)
{
	uint8_t txBuf[2] = {regAddr|0x00 , data}; //Write operation: set the 8th-bit to 0
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
	while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	
	return status;	
}
/*=========================================================================================================================================
 * @brief     Write data to spacial bits of a register.
 * @param     regAddr     Register Address of DPS310 .
 * @param     start_bit   Start Bit location .(0 to 7)
 * @param     len         Quantity of Bits want to write(1 to 8) 
 * @param     data        Value that will be writen to register bits .
 * @return    status      Return status
 ========================================================================================================================================*/
uint8_t GB_DPS310_Write_Reg_Bits(uint8_t regAddr, uint8_t start_bit, uint8_t len, uint8_t data)
{
	uint8_t txBuf[2];
	uint8_t rxBuf[2];
	uint8_t status = HAL_ERROR;
	uint8_t tempData = 0;
	if (len>8 || start_bit>7)
	{
		return HAL_ERROR;
	}
	if (GB_DPS310_Read_Reg_Data( regAddr,  &tempData) == HAL_OK)	
	{
		uint8_t mask = ((1 << len) - 1) << (start_bit - len + 1);
		data <<= (start_bit - len + 1); // shift data into correct position
		data &= mask; // zero all non-important bits in data
		tempData &= ~(mask); // zero all important bits in existing byte
		tempData |= data; // combine data with existing byte

		txBuf[0] = regAddr;
		txBuf[1] = tempData;
	
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
		status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, 2, HAL_MAX_DELAY));
		while(HAL_SPI_GetState(&hspi1) != HAL_SPI_STATE_READY);
		HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);
	}
	return status;
}
/*========================================================================================================================================= 
 * @brief     Write value to Multiple register address.
 * @param     regAddr First Register Address of DPS310 that writing multiple data start from this address
 * @param     data    Pointer to Variable that multiple data are writen from .
 * @param     byteQuantity Quantity of data that we want to write .
 * @return    status    Return status
 ========================================================================================================================================*/
uint8_t GB_DPS310_Burst_Write		( uint8_t regAddr, uint8_t *data, 	uint16_t byteQuantity)
{
	uint8_t txBuf[byteQuantity + 1]; // +1 is for register address that is 1 byte
	uint8_t rxBuf[byteQuantity + 1];
	uint8_t status = HAL_ERROR;
	txBuf[0] = regAddr | 0x00; //Write operation: set the 8th-bit to 0.
	memcpy(txBuf+1, data, byteQuantity); // +1 is for set the address of data from [1]th position of array

	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_RESET);
	status = (HAL_SPI_TransmitReceive(&hspi1, txBuf, rxBuf, byteQuantity+1, HAL_MAX_DELAY));
	HAL_GPIO_WritePin(SPI_CS_GPIO_Port, SPI_CS_Pin, GPIO_PIN_SET);

	return status;
}
/*=========================================================================================================================================
 * @brief     Set Pressure Measurement Rate 
 * @param     rate   Values According to DPS310_Measurement_Rate Enum 
 * @param     DPS310   DPS310 struct  PRESSURE_RATE variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Pressure_Measurement_Rate(GebraBit_DPS310 * DPS310 , DPS310_Measurement_Rate rate )
{
 if (DPS310->MEASUREMENT_MODE >= CONTINUOUS_BACKGROUND_PRESSURE )
 {
  GB_DPS310_Write_Reg_Bits ( DPS310_PRS_CFG,  START_MSB_BIT_AT_6,  BIT_LENGTH_3,rate );
	DPS310->PRESSURE_RATE = rate ;
 }
}
/*=========================================================================================================================================
 * @brief     Set Pressure OverSampling
 * @param     oversmp   Values According to DPS310_Oversampling Enum 
 * @param     DPS310   DPS310 struct  PRESSURE_SCALE_FACTOR And PRESSURE_OVERSAMPLING variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Pressure_OverSampling(GebraBit_DPS310 * DPS310 , DPS310_Oversampling oversmp )
{
	DPS310->PRESSURE_OVERSAMPLING = oversmp ;
  GB_DPS310_Write_Reg_Bits ( DPS310_PRS_CFG,  START_MSB_BIT_AT_3,  BIT_LENGTH_4,oversmp );
	if (DPS310->PRESSURE_OVERSAMPLING >= _16_TIMES )
	{
	  GB_DPS310_Pressure_Result_BitShift(DPS310 ,  SHIFT_BIT); 
	}	
	switch(DPS310->PRESSURE_OVERSAMPLING )
	 {
	  case _1_PER_SECOND:
		DPS310->PRESSURE_SCALE_FACTOR = SF_524288_KP_KT ;
		DPS310->PRESSURE_SF_VALUE     = 524288 ;
    break;
		case _2_PER_SECOND:
		DPS310->PRESSURE_SCALE_FACTOR = SF_1572864_KP_KT ;
		DPS310->PRESSURE_SF_VALUE = 1572864 ;
    break;	
		case _4_PER_SECOND:
		DPS310->PRESSURE_SCALE_FACTOR = SF_3670016_KP_KT ;
		DPS310->PRESSURE_SF_VALUE = 3670016 ;
    break;	
		case _8_PER_SECOND:
		DPS310->PRESSURE_SCALE_FACTOR = SF_7864320_KP_KT ;
		DPS310->PRESSURE_SF_VALUE = 7864320 ;
    break;	
	  case _16_PER_SECOND:
		DPS310->PRESSURE_SCALE_FACTOR = SF_253952_KP_KT ;
		DPS310->PRESSURE_SF_VALUE = 253952 ;
    break;
		case _32_PER_SECOND:
		DPS310->PRESSURE_SCALE_FACTOR = SF_516096_KP_KT ;
		DPS310->PRESSURE_SF_VALUE = 516096 ;
    break;	
		case _64_PER_SECOND:
		DPS310->PRESSURE_SCALE_FACTOR = SF_1040384_KP_KT ;
		DPS310->PRESSURE_SF_VALUE = 1040384 ;
    break;	
		case _128_PER_SECOND:
		DPS310->PRESSURE_SCALE_FACTOR = SF_2088960_KP_KT ;
		DPS310->PRESSURE_SF_VALUE = 2088960 ;
    break;			
	 }
}
/*=========================================================================================================================================
 * @brief     Set Temperature Measurement Rate 
 * @param     rate   Values According to DPS310_Measurement_Rate Enum 
 * @param     DPS310   DPS310 struct  TEMPERATURE_RATE variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Temperature_Measurement_Rate(GebraBit_DPS310 * DPS310 , DPS310_Measurement_Rate rate )
{
 if (DPS310->MEASUREMENT_MODE >= CONTINUOUS_BACKGROUND_PRESSURE )
 {
  GB_DPS310_Write_Reg_Bits ( DPS310_TMP_CFG,  START_MSB_BIT_AT_6,  BIT_LENGTH_3,rate );
	DPS310->TEMPERATURE_RATE = rate ;
 }
}
/*=========================================================================================================================================
 * @brief     Set Temperature OverSampling
 * @param     oversmp   Values According to DPS310_Oversampling Enum 
 * @param     DPS310   DPS310 struct  TEMPERATURE_SCALE_FACTOR And TEMPERATURE_OVERSAMPLING variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Temperature_OverSampling(GebraBit_DPS310 * DPS310 , DPS310_Oversampling oversmp )
{
  GB_DPS310_Write_Reg_Bits ( DPS310_TMP_CFG,  START_MSB_BIT_AT_3,  BIT_LENGTH_4,oversmp );
	DPS310->TEMPERATURE_OVERSAMPLING = oversmp ;
	if (DPS310->TEMPERATURE_OVERSAMPLING >= _16_TIMES )
	{
	  GB_DPS310_Temperature_Result_BitShift(DPS310 ,  SHIFT_BIT); 
	}
	switch(DPS310->TEMPERATURE_OVERSAMPLING )
	 {
	  case _1_PER_SECOND:
		DPS310->TEMPERATURE_SCALE_FACTOR = SF_524288_KP_KT ;
		DPS310->TEMPERATURE_SF_VALUE     = 524288 ;
    break;
		case _2_PER_SECOND:
		DPS310->TEMPERATURE_SCALE_FACTOR = SF_1572864_KP_KT ;
		DPS310->TEMPERATURE_SF_VALUE = 1572864 ;
    break;	
		case _4_PER_SECOND:
		DPS310->TEMPERATURE_SCALE_FACTOR = SF_3670016_KP_KT ;
		DPS310->TEMPERATURE_SF_VALUE = 3670016 ;
    break;	
		case _8_PER_SECOND:
		DPS310->TEMPERATURE_SCALE_FACTOR = SF_7864320_KP_KT ;
		DPS310->TEMPERATURE_SF_VALUE = 7864320 ;
    break;	
	  case _16_PER_SECOND:
		DPS310->TEMPERATURE_SCALE_FACTOR = SF_253952_KP_KT ;
		DPS310->TEMPERATURE_SF_VALUE = 253952 ;
    break;
		case _32_PER_SECOND:
		DPS310->TEMPERATURE_SCALE_FACTOR = SF_516096_KP_KT ;
		DPS310->TEMPERATURE_SF_VALUE = 516096 ;
    break;	
		case _64_PER_SECOND:
		DPS310->TEMPERATURE_SCALE_FACTOR = SF_1040384_KP_KT ;
		DPS310->TEMPERATURE_SF_VALUE = 1040384 ;
    break;	
		case _128_PER_SECOND:
		DPS310->TEMPERATURE_SCALE_FACTOR = SF_2088960_KP_KT ;
		DPS310->TEMPERATURE_SF_VALUE = 2088960 ;
    break;			
	 }
}
/*=========================================================================================================================================
 * @brief     Temperature Selection
 * @param     tmp      Values According to DPS310_Temperature_Sensor Enum 
 * @param     DPS310   DPS310 struct  TEMPERATURE_RATE variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Temperature(GebraBit_DPS310 * DPS310 , DPS310_Temperature_Sensor tmp )
{
  GB_DPS310_Write_Reg_Bits ( DPS310_TMP_CFG,  START_MSB_BIT_AT_7,  BIT_LENGTH_1,tmp );
	DPS310->TEMPERATURE = tmp ;
}
/*=========================================================================================================================================
 * @brief     Check if Coefficients are ready 
 * @param     DPS310   DPS310 struct  COEFFICIENT_STATUS variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Check_Coefficient(GebraBit_DPS310 * DPS310)
{
 GB_DPS310_Read_Reg_Bits ( DPS310_MEAS_CFG,  START_MSB_BIT_AT_7,  BIT_LENGTH_1,&DPS310->COEFFICIENT_STATUS );
}
/*=========================================================================================================================================
 * @brief     Check if Sensor is Initialized 
 * @param     DPS310   DPS310 struct  SENSOR_INITIALIZATION variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Sensor_Initialization(GebraBit_DPS310 * DPS310)
{
 GB_DPS310_Read_Reg_Bits ( DPS310_MEAS_CFG,  START_MSB_BIT_AT_6,  BIT_LENGTH_1,&DPS310->SENSOR_INITIALIZATION );
}
/*=========================================================================================================================================
 * @brief     Check if Temperature Data is ready
 * @param     DPS310   DPS310 struct  TEMPERATURE_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Check_Temperature_Data(GebraBit_DPS310 * DPS310)
{
 GB_DPS310_Read_Reg_Bits ( DPS310_MEAS_CFG,  START_MSB_BIT_AT_5,  BIT_LENGTH_1,&DPS310->TEMPERATURE_DATA );
}
/*=========================================================================================================================================
 * @brief     Check if Pressure Data is ready
 * @param     DPS310   DPS310 struct  PRESSURE_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Check_Pressure_Data(GebraBit_DPS310 * DPS310)
{
 GB_DPS310_Read_Reg_Bits ( DPS310_MEAS_CFG,  START_MSB_BIT_AT_4,  BIT_LENGTH_1,&DPS310->PRESSURE_DATA );
}
/*=========================================================================================================================================
 * @brief     Check if Temperature&Pressure Data is ready
 * @param     DPS310   DPS310 struct  TEMP_PRESS_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Check_Temperature_Pressure_Data(GebraBit_DPS310 * DPS310)
{
 GB_DPS310_Read_Reg_Bits ( DPS310_MEAS_CFG,  START_MSB_BIT_AT_5,  BIT_LENGTH_2,&DPS310->TEMP_PRESS_DATA );
}
/*
M403Z 
*/
/*=========================================================================================================================================
 * @brief     Set Measurement Mode
 * @param     meas      Values According to DPS310_Measurement_Mode Enum 
 * @param     DPS310    DPS310 struct  MEASUREMENT_MODE variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_DPS310_Measurement_Mode(GebraBit_DPS310 * DPS310 , DPS310_Measurement_Mode meas)
{
 GB_DPS310_Write_Reg_Bits ( DPS310_MEAS_CFG,  START_MSB_BIT_AT_2,  BIT_LENGTH_3,meas );
 DPS310->MEASUREMENT_MODE = meas ;	
}
/*=========================================================================================================================================
 * @brief     Set Temperature Result BitShift
 * @param     shift      Values According to DPS310_Bit_Shift Enum 
 * @param     DPS310    DPS310 struct  TEMPERATURE_BITSHIFT variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_DPS310_Temperature_Result_BitShift(GebraBit_DPS310 * DPS310 , DPS310_Bit_Shift shift)
{
 GB_DPS310_Write_Reg_Bits ( DPS310_CFG_REG,  START_MSB_BIT_AT_3,  BIT_LENGTH_1,shift );
 DPS310->TEMPERATURE_BITSHIFT = shift ;	
}
/*=========================================================================================================================================
 * @brief     Set Pressure Result BitShift
 * @param     shift      Values According to DPS310_Bit_Shift Enum 
 * @param     DPS310    DPS310 struct  PRESSURE_BITSHIFT variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_DPS310_Pressure_Result_BitShift(GebraBit_DPS310 * DPS310 , DPS310_Bit_Shift shift)
{
 GB_DPS310_Write_Reg_Bits ( DPS310_CFG_REG,  START_MSB_BIT_AT_2,  BIT_LENGTH_1,shift );
 DPS310->PRESSURE_BITSHIFT = shift ;	
}
/*=========================================================================================================================================
 * @brief     Set DPS310 FIFO
 * @param     fifo      Values According to DPS310_Ability Enum 
 * @param     DPS310    DPS310 struct  FIFO variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_DPS310_FIFO(GebraBit_DPS310 * DPS310 , DPS310_Ability fifo)
{
 GB_DPS310_Write_Reg_Bits ( DPS310_CFG_REG,  START_MSB_BIT_AT_1,  BIT_LENGTH_1,fifo );
 DPS310->FIFO = fifo ;	
}
/*=========================================================================================================================================
 * @brief     Read DPS310 FIFO
 * @param     DPS310    DPS310 struct  FIFO_DATA variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_DPS310_Read_FIFO(GebraBit_DPS310 * DPS310  )  
{
  GB_DPS310_Burst_Read( DPS310_PSR_B2,DPS310->FIFO_DATA, FIFO_DATA_BUFFER_SIZE);
}
/*=========================================================================================================================================
 * @brief     Set DPS310 Interrupt
 * @param     intrupt      Values According to DPS310_Interrupt Enum 
 * @param     DPS310    DPS310 struct  INTERRUPT variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_DPS310_Interrupt(GebraBit_DPS310 * DPS310 , DPS310_Interrupt intrupt)
{
 GB_DPS310_Write_Reg_Bits ( DPS310_CFG_REG,  START_MSB_BIT_AT_6,  BIT_LENGTH_3,intrupt );
 DPS310->INTERRUPT = intrupt;
}
/*=========================================================================================================================================
 * @brief     Check Interrupt Status
 * @param     DPS310   DPS310 struct  INTERRUPT_STATUS variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Check_Interrupt_Status(GebraBit_DPS310 * DPS310 )
{
 GB_DPS310_Read_Reg_Bits ( DPS310_INT_STS,  START_MSB_BIT_AT_2,  BIT_LENGTH_3,&DPS310->INTERRUPT_STATUS );
}
/*=========================================================================================================================================
 * @brief     Check FIFO Status
 * @param     DPS310   DPS310 struct  FIFO_STATUS variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Check_FIFO_Status(GebraBit_DPS310 * DPS310 )
{
 GB_DPS310_Read_Reg_Bits ( DPS310_FIFO_STS,  START_MSB_BIT_AT_1,  BIT_LENGTH_2,&DPS310->FIFO_STATUS );
}
/*=========================================================================================================================================
 * @brief     Empty FIFO 
 * @param     DPS310   DPS310 struct  
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_FIFO_Flush(GebraBit_DPS310 * DPS310 )
{
 GB_DPS310_Write_Reg_Bits ( DPS310_RESET,  START_MSB_BIT_AT_7,  BIT_LENGTH_1,DPS310_FIFO_FLUSH_CMD );
 HAL_Delay(10);	
}
/*=========================================================================================================================================
 * @brief     Reset FIFO 
 * @param     DPS310   DPS310 struct  
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Soft_Reset(GebraBit_DPS310 * DPS310 )
{
 GB_DPS310_Write_Reg_Bits ( DPS310_RESET,  START_MSB_BIT_AT_3,  BIT_LENGTH_4,DPS310_SOFT_RESET_CMD );
 HAL_Delay(100);
}
/*=========================================================================================================================================
 * @brief     Get Product ID
 * @param     DPS310   DPS310 struct  PRODUCT_ID variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Product_ID(GebraBit_DPS310 * DPS310 )
{
 GB_DPS310_Read_Reg_Bits ( DPS310_PRODUCT_ID,  START_MSB_BIT_AT_3,  BIT_LENGTH_4,&DPS310->PRODUCT_ID );
}
/*=========================================================================================================================================
 * @brief     Get Revision ID
 * @param     DPS310   DPS310 struct  REVISION_ID variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Revision_ID(GebraBit_DPS310 * DPS310 )
{
 GB_DPS310_Read_Reg_Bits ( DPS310_PRODUCT_ID,  START_MSB_BIT_AT_7,  BIT_LENGTH_4,&DPS310->REVISION_ID );
}
/*=========================================================================================================================================
 * @brief     Check Temperature Coefficient Source
 * @param     DPS310   DPS310 struct  TEMPERATURE_COEFFICIENT variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Check_Temperature_Coefficient_Source(GebraBit_DPS310 * DPS310  )
{
	uint8_t tmp;
	GB_DPS310_Read_Reg_Data ( DPS310_COEF_SRCE,&tmp);
	DPS310->TEMPERATURE_COEFFICIENT = (DPS310_Temperature_Sensor)((tmp & 0x80)>>7) ;
}
/*=========================================================================================================================================
 * @brief     Config DPS310 FIFO
 * @param     DPS310   DPS310 struct 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_FIFO_Configuration ( GebraBit_DPS310 * DPS310 , DPS310_FIFO_Ability fifo  )
{
	//DPS310->FIFO_PACKET_QTY = FIFO_DATA_BUFFER_SIZE / BYTE_QTY_IN_ONE_PACKET ;  
	if( fifo==FIFO_ENABLE )  
	{
		DPS310->PACKET_QTY_IN_FULL_FIFO = FIFO_DATA_BUFFER_SIZE / BYTE_QTY_IN_ONE_PACKET ;  
		DPS310->FIFO = Enable  ;
		GB_DPS310_FIFO_Flush( DPS310 );
		GB_DPS310_FIFO(  DPS310 , Enable );
	}
	else if ( fifo == FIFO_DISABLE )
	{
		DPS310->FIFO = Disable  ;
		GB_DPS310_FIFO( DPS310 , Disable );//--****************************---Disable
		GB_DPS310_FIFO_Flush( DPS310 );
	}
}
/*=========================================================================================================================================
 * @brief     Initialize DPS310
 * @param     DPS310   DPS310 struct 
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Initialize( GebraBit_DPS310 * DPS310 )
{
	GB_DPS310_Soft_Reset ( DPS310 );
	do 
	 {
    GB_DPS310_Sensor_Initialization(DPS310);
		HAL_Delay(15);///page 10 datasheet
		if ( DPS310->SENSOR_INITIALIZATION == INITIALIZATION_COMPLETE )
			break;
	 }while(1);
	GB_DPS310_Product_ID ( DPS310 );
	GB_DPS310_Revision_ID( DPS310 );	
  GB_DPS310_Temperature_Correction( DPS310 );	 
	GB_DPS310_Check_Temperature_Coefficient_Source( DPS310 );
	GB_DPS310_Temperature(DPS310 ,  DPS310->TEMPERATURE_COEFFICIENT ); 
	GB_DPS310_Calculate_Calibration_Coefficients( DPS310 ); 
	//GB_DPS310_Pressure_Result_BitShift(DPS310 ,  SHIFT_BIT);
	GB_DPS310_Measurement_Mode(DPS310 , CONTINUOUS_BACKGROUND_PRESSURE_TEMPERATURE);
	GB_DPS310_FIFO_Configuration (DPS310 , FIFO_DISABLE  );
	 
}	
/*=========================================================================================================================================
 * @brief     Config DPS310 
 * @param     fifo   Values According to DPS310_FIFO_Ability 
 * @param     DPS310   DPS310 struct 
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_DPS310_Configuration(GebraBit_DPS310 * DPS310, DPS310_FIFO_Ability fifo) 
{
 GB_DPS310_Pressure_Measurement_Rate(DPS310 , _4_PER_SECOND );
 GB_DPS310_Pressure_OverSampling(DPS310 ,  _64_TIMES );
 GB_DPS310_Temperature_Measurement_Rate(DPS310 , _4_PER_SECOND );
 GB_DPS310_Temperature_OverSampling(DPS310 , SINGLE );
 GB_DPS310_FIFO_Configuration (DPS310 , fifo  );
}
/*=========================================================================================================================================
 * @brief     Converts value Twos_Complement to real value
 * @param     value    points to data to be convert 
 * @param     length   length of value in Bits 
 * @return    Nothing
 ========================================================================================================================================*/
void GB_DPS310_Twos_Complement_Converter(int32_t *value, uint8_t length)
{
	if (*value & ((uint32_t)1 << (length - 1)))
	{
		*value -= (uint32_t)1 << length;
	}
}
/*=========================================================================================================================================
 * @brief     Config DPS310 
 * @param     DPS310   DPS310 struct TEMPERATURE_CORRECTION variable 
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_DPS310_Temperature_Correction(GebraBit_DPS310 * DPS310)
{
	uint8_t temp;
  GB_DPS310_Read_Reg_Data ( 0x32,&temp);
	/*
	 * If bit 1 is set then the device is okay, and TEMPERATURE CORRECTION does not
	 * need to be applied
	 */
	if (temp & 0x02)
		DPS310->TEMPERATURE_CORRECTION = Disable ; 
  else
	{
		DPS310->TEMPERATURE_CORRECTION = Enable ;
    GB_DPS310_Write_Reg_Data( 0x0e, 0xA5);
	  GB_DPS310_Write_Reg_Data( 0x0f, 0x96);
	  GB_DPS310_Write_Reg_Data( 0x62, 0x02);
	  GB_DPS310_Write_Reg_Data( 0x0e, 0x00);
    GB_DPS310_Write_Reg_Data( 0x0f, 0x00);
	}
}
/*=========================================================================================================================================
 * @brief     Calculate Calibration Coefficients
 * @param     DPS310   DPS310 struct C0 ...to...C30
 * @return    Nothing
 ========================================================================================================================================*/ 	
void GB_DPS310_Calculate_Calibration_Coefficients(GebraBit_DPS310 * DPS310)
{
 do 
	 {
    GB_DPS310_Check_Coefficient(DPS310);
		HAL_Delay(45);///page 10 datasheet
		if ( DPS310->COEFFICIENT_STATUS == COEFFICIENT_ARE_AVAILABLE )
			break;
	 }while(1);
 GB_DPS310_Burst_Read( DPS310_COEF,  DPS310->CALIBRATION_COEFFICIENT_DATA ,  COEFFICIENT_DATA_BUFFER_SIZE);
 DPS310->C0 = ((uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[0] << 4) | (((uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[1] >> 4) & 0x0F);
 GB_DPS310_Twos_Complement_Converter(&DPS310->C0, 12);
 DPS310->C1 = (((uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[1] & 0x0F) << 8) | (uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[2];
 GB_DPS310_Twos_Complement_Converter(&DPS310->C1, 12);
 DPS310->C00 = ((uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[3] << 12) | ((uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[4] << 4) | (((uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[5] >> 4) & 0x0F);
 GB_DPS310_Twos_Complement_Converter(&DPS310->C00, 20);
 DPS310->C10 = (((uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[5] & 0x0F) << 16) | ((uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[6] << 8) | (uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[7];
 GB_DPS310_Twos_Complement_Converter(&DPS310->C10, 20);
 DPS310->C01 = ((uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[8] << 8) | (uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[9];
 GB_DPS310_Twos_Complement_Converter(&DPS310->C01, 16);
 DPS310->C11 = ((uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[10] << 8) | (uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[11];
 GB_DPS310_Twos_Complement_Converter(&DPS310->C11, 16);
 DPS310->C20 = ((uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[12] << 8) | (uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[13];
 GB_DPS310_Twos_Complement_Converter(&DPS310->C20, 16);
 DPS310->C21 = ((uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[14] << 8) | (uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[15];
 GB_DPS310_Twos_Complement_Converter(&DPS310->C21, 16);
 DPS310->C30 = ((uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[16] << 8) | (uint32_t)DPS310->CALIBRATION_COEFFICIENT_DATA[17];
 GB_DPS310_Twos_Complement_Converter(&DPS310->C30, 16);
}
/*=========================================================================================================================================
 * @brief     Get Raw Pressure And Temperature From Register
 * @param     DPS310   DPS310 struct REGISTER_RAW_PRESSURE and REGISTER_RAW_TEMPERATURE Variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Get_Register_Raw_Pressure_Temperature(GebraBit_DPS310 * DPS310 )
{
 GB_DPS310_Check_Temperature_Pressure_Data(DPS310);
 if(DPS310->TEMP_PRESS_DATA == TEMPERATURE_PRESSURE_IS_READY)
	{
   GB_DPS310_Burst_Read( DPS310_PSR_B2,  DPS310->REGISTER_RAW_DATA_BUFFER ,  REGISTER_RAW_DATA_BUFFER_SIZE);
   DPS310->REGISTER_RAW_PRESSURE    = ((uint32_t)DPS310->REGISTER_RAW_DATA_BUFFER[0]<<16)  | ((uint32_t)DPS310->REGISTER_RAW_DATA_BUFFER[1]<<8) | ((uint32_t)DPS310->REGISTER_RAW_DATA_BUFFER[2])  ;
   DPS310->REGISTER_RAW_TEMPERATURE = ((uint32_t)DPS310->REGISTER_RAW_DATA_BUFFER[3]<<16)  | ((uint32_t)DPS310->REGISTER_RAW_DATA_BUFFER[4]<<8) | ((uint32_t)DPS310->REGISTER_RAW_DATA_BUFFER[5])  ;
   GB_DPS310_Twos_Complement_Converter(&DPS310->REGISTER_RAW_PRESSURE   ,24);
   GB_DPS310_Twos_Complement_Converter(&DPS310->REGISTER_RAW_TEMPERATURE,24);
	}
}
/*=========================================================================================================================================
 * @brief     Calculate Compensated Temperature
 * @param     DPS310   DPS310 struct COMPENSATED_TEMPERATURE Variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Calculate_Compensated_Temperature(GebraBit_DPS310 * DPS310)	
{
 float   scaled_temperature;
 scaled_temperature = (float)DPS310->REGISTER_RAW_TEMPERATURE/(float)DPS310->TEMPERATURE_SF_VALUE ;
 DPS310->COMPENSATED_TEMPERATURE = (float)(DPS310->C0/2U)+((float)DPS310->C1 * scaled_temperature );
}
/*=========================================================================================================================================
 * @brief     Calculate Compensated Pressure
 * @param     DPS310   DPS310 struct COMPENSATED_PRESSURE Variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Calculate_Compensated_Pressure(GebraBit_DPS310 * DPS310)
{
	float scaled_pressure , scaled_temperature;
  scaled_pressure    = (float)DPS310->REGISTER_RAW_PRESSURE/(float)DPS310->PRESSURE_SF_VALUE ;
	scaled_temperature = (float)DPS310->REGISTER_RAW_TEMPERATURE/(float)DPS310->TEMPERATURE_SF_VALUE ;
	DPS310->COMPENSATED_PRESSURE    = DPS310->C00 +( scaled_pressure * (DPS310->C10 + (scaled_pressure *(DPS310->C20+ (scaled_pressure * DPS310->C30))))) + (scaled_temperature *DPS310->C01) + (scaled_temperature *scaled_pressure *(DPS310->C11+(scaled_pressure*DPS310->C21))) ;	
  DPS310->COMPENSATED_PRESSURE /=100;// Pa to mbar
}
/*=========================================================================================================================================
 * @brief     Devide Data from FIFO to Pressure And Temperature
 * @param     DPS310   DPS310 struct COMPENSATED_FIFO_TEMPERATURE and COMPENSATED_FIFO_PRESSURE Variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_FIFO_Data_Partition_Pressure_Temperature(GebraBit_DPS310 * DPS310)
{
 	uint16_t i,offset=0;
	uint16_t j = 0;
	int32_t fifo_temp =0;
	int32_t last_raw_temp =0;
	 for ( i = 0 ; ((i+j) < DPS310->PACKET_QTY_IN_FULL_FIFO)&&(offset<FIFO_DATA_BUFFER_SIZE-2) ; )	
		{
		 fifo_temp = ((uint32_t)DPS310->FIFO_DATA[offset]<<16)  | ((uint32_t)DPS310->FIFO_DATA[offset+1]<<8) | ((uint32_t)DPS310->FIFO_DATA[offset+2]);
	   if( DPS310->FIFO_DATA[offset+2] & 0x01)
			{
			 if((uint32_t)last_raw_temp)
			 { 
				 GB_DPS310_Twos_Complement_Converter(&fifo_temp   ,24);
				 DPS310->COMPENSATED_FIFO_PRESSURE[i] = (DPS310->C00 +( ((float)fifo_temp/(float)DPS310->PRESSURE_SF_VALUE) * (DPS310->C10 + (((float)fifo_temp/(float)DPS310->PRESSURE_SF_VALUE) *(DPS310->C20+ (((float)fifo_temp/(float)DPS310->PRESSURE_SF_VALUE) * DPS310->C30))))) + (((float)last_raw_temp/(float)DPS310->TEMPERATURE_SF_VALUE) *DPS310->C01) + (((float)last_raw_temp/(float)DPS310->TEMPERATURE_SF_VALUE) *((float)fifo_temp/(float)DPS310->PRESSURE_SF_VALUE) *(DPS310->C11+(((float)fifo_temp/(float)DPS310->PRESSURE_SF_VALUE)*DPS310->C21))))/100 ;	
			   DPS310->FIFO_ALTITUDE[j] =((1 - pow((DPS310->COMPENSATED_FIFO_PRESSURE[i]*100) / SEA_LEVEL_PRESSURE, 1/5.257)) / 0.0000225577);
				 i++;
			 } 
			}
		 else
		  {
				GB_DPS310_Twos_Complement_Converter(&fifo_temp   ,24);
				last_raw_temp = fifo_temp ;
				DPS310->COMPENSATED_FIFO_TEMPERATURE[j] = (float)(DPS310->C0/2U)+((float)DPS310->C1 * ((float)fifo_temp/(float)DPS310->TEMPERATURE_SF_VALUE) );
				j++;
			}
				offset += 3; 
		} 
}
/*=========================================================================================================================================
 * @brief     Determine Grtting Data From FIFO Or Register
 * @param     get_data   According to DPS310_Get_DATA Enum 
 * @param     DPS310    DPS310 struct GET_DATA  Variable
 * @return    Nothing
 ========================================================================================================================================*/
void GB_DPS310_Get_Data(GebraBit_DPS310 * DPS310 , DPS310_Get_DATA get_data)
{
 if( (get_data == FROM_REGISTER)&&(DPS310->FIFO == Disable) )
 {
	  DPS310->GET_DATA = FROM_REGISTER ;
	  GB_DPS310_Get_Register_Raw_Pressure_Temperature(DPS310);
	  GB_DPS310_Calculate_Compensated_Temperature(DPS310)	;
	  GB_DPS310_Calculate_Compensated_Pressure(DPS310); 
	  GB_DPS310_Altitude(DPS310);
 } 
 else if ((get_data == FROM_FIFO)&&(DPS310->FIFO == Enable)) 
 {
   GB_DPS310_Check_FIFO_Status(DPS310);
	 if(DPS310->FIFO_STATUS == FIFO_FULL)
	 {
		 DPS310->GET_DATA = FROM_FIFO ;
		 GB_DPS310_Read_FIFO(DPS310);
		 GB_DPS310_FIFO_Data_Partition_Pressure_Temperature(DPS310);
		 GB_DPS310_FIFO_Flush(DPS310);
	 }
 }	 
}
/*=========================================================================================================================================
 * @brief     Convert Pressuer To Altitude
 * @param     DPS310   DPS310 struct ALTITUDE Variable
 * @return    Nothing
 ========================================================================================================================================*/ 
void GB_DPS310_Altitude(GebraBit_DPS310 * DPS310)
{
    double altitude;

    DPS310->ALTITUDE = ((1 - pow((DPS310->COMPENSATED_PRESSURE*100) / SEA_LEVEL_PRESSURE, 1/5.257)) / 0.0000225577);
}

/*----------------------------------------------------------------------------------------------------------------------------------------*
 *                                                                      End                                                               *
 *----------------------------------------------------------------------------------------------------------------------------------------*/



