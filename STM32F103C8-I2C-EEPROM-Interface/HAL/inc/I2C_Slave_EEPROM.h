/*
 * I2C_Slave_EEPROM.h
 *
 *  Created on: May 4, 2024
 *  Author: ABDULLAH KARKOUR
 */

#ifndef INC_I2C_SLAVE_EEPROM_H_
#define INC_I2C_SLAVE_EEPROM_H_

#include "Stm32F103C8_I2C_Driver.h"

/*
 * EEPROM is i2c Slave
 * idel mode : device in high impedance and waits for data
 * Master Transmitter Mode : the Device Transmite data to slave reciever
 * Master Reciever Mode   : the device Recieve data from slave transmitter
 **/

#define EEPROM_Slave_Address			0x2A

void EEPROM_Init();
void EEPROM_Write_Nbytes(uint32_t memory_address , uint8_t* data , uint8_t data_length);
void EEPROM_Read_bytes(uint32_t memory_address , uint8_t* dataOut , uint8_t data_length);


#endif /* INC_I2C_SLAVE_EEPROM_H_ */
