/**
 ******************************************************************************
 * @file           : main.c
 * @author         : ABDULLAH KARKOUR
 * @brief          : STM32F103C8 interface with I2C Slave EEPROM in Master Mode
 ******************************************************************************
 */

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
  #warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif

#include "Stm32_F103X6.h"
#include "Stm32F103C8_I2C_Driver.h"
#include "I2C_Slave_EEPROM.h"

void Clock_init()
{
	RCC_GPIOB_CLK_EN();
}
int main(void)
{
	Clock_init();

	// Test Case 1
	uint8_t ch1[] = { 0x1, 0x2 ,0x3,0x4,0x5,0x6,0x7};
	uint8_t ch2[7] = {0};

	EEPROM_Init();
	EEPROM_Write_Nbytes(0xAF, ch1, 7);
	EEPROM_Read_bytes(0xAF, ch2, 7);

	// Test case 2
	ch1[0] = 0xA ;
	ch1[1] = 0xB ;
	ch1[2] = 0xC ;
	ch1[3] = 0xD ;

	EEPROM_Write_Nbytes(0xFFF, ch1, 4);
	EEPROM_Read_bytes(0xFFF, ch2, 4);

    /* Loop forever */
	for(;;);
}
