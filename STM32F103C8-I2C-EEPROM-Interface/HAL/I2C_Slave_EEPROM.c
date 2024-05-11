/*
 * I2C_Slave_EEPROM.c
 *
 *  Created on: May 4, 2024
 *  Author: ABDULLAH KARKOUR
 */


#include "I2C_Slave_EEPROM.h"

void EEPROM_Init()
{
	// Init I2C
	I2C_Device_CFG_t   I2C1_cfg ;

	I2C1_cfg.Device_mode = master ;
	I2C1_cfg.I2C_MOde = I2C_Mode_I2C_Mode;
	I2C1_cfg.General_Call_Address_Detection = I2C_GC_Enable;
	I2C1_cfg.I2C_ACK_Control = I2C_ACK_Control_Enable ;
	I2C1_cfg.I2C_CLK_Speed = I2C_CLK_Speed_SM_100K ;
	I2C1_cfg.I2C_IRQ = I2C_IRQ_NONE ;
	I2C1_cfg.I2C_Stretch_Mode = I2C_Stretch_Mode_Enable;

	MCAL_I2C_Init(I2C1, &I2C1_cfg) ;
	MCAL_I2C_GPIO_Set_Pins(I2C1);

}
void EEPROM_Write_Nbytes(uint32_t memory_address , uint8_t* data , uint8_t data_length)
{
	uint8_t Buffer[250] , i ;
	Buffer[0] = (uint8_t)(memory_address >>8);
	Buffer[1] =(uint8_t) memory_address ;

	for (i=2 ; i<data_length+2 ; i++)
	{
		Buffer[i] = data[i-2];
	}

	MCAL_I2C_Master_Tx(I2C1, EEPROM_Slave_Address, Buffer, data_length+2, with_Stop, Start);
}
void EEPROM_Read_bytes(uint32_t memory_address , uint8_t* dataOut , uint8_t data_length)
{
	uint8_t Buffer[2] ;
	Buffer[0] = (uint8_t)(memory_address >>8);
	Buffer[1] = (uint8_t) memory_address ;

	MCAL_I2C_Master_Tx(I2C1, EEPROM_Slave_Address, Buffer, 2, without_Stop, Start);
	MCAL_I2C_Master_Rx(I2C1, EEPROM_Slave_Address, dataOut, data_length, with_Stop, Reapeted_Start);
}
