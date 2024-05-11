/**
 ******************************************************************************
 * @file           : Stm32F103C8_I2C_Driver.c
 * @author         : ABDULLAH KARKOUR
 * @brief          : STM32F103C8 I2C Driver
 ******************************************************************************
*/

/*
* ===============================================
* Includes
* ===============================================
*/
#include "Stm32F103C8_I2C_Driver.h"

/*
* ===============================================
* 			Generic Variables
* ===============================================
*/
I2C_Device_CFG_t G_I2C_Cfg[2];
/*
* ===============================================
* 			Generic Macros
* ===============================================
*/

#define I2C1_index		0
#define I2C2_index		1



/*
* ===============================================
* APIs Supported by "MCAL I2C DRIVER"
* ===============================================
*/

/**================================================================
 * @Fn			-MCAL_I2C_Init
 * @brief 		-Initializes the I2C according to the specified parameters in I2C_Init_Cfg
 * @param [in] 	-I2Cx: where x can be (1,2 depending on I2C used) to select the I2C peripheral
 * @param [in] 	-I2C_Init_Cfg: pointer to a I2C_Device_CFG_t structure that contains
 *         		 the configuration information for the specified I2C.
 * @retval 		-none
 * Note			-none
 */
void MCAL_I2C_Init(I2C_TypeDef* I2Cx , I2C_Device_CFG_t* I2C_Init_Cfg)
{
	// Enable RCC Clock
	if(I2Cx == I2C1)
	{
		RCC_I2C1_CLK_EN();
		G_I2C_Cfg[I2C1_index] = *I2C_Init_Cfg ;
	}
	else if (I2Cx == I2C2)
	{
		RCC_I2C2_CLK_EN();
		G_I2C_Cfg[I2C2_index] = *I2C_Init_Cfg ;
	}

	// Disable The I2C Peripheral I2C_CR1 Bit 0 PE: Peripheral enable
	I2Cx->I2C_CR1 &= ~(I2C_CR1_PE);

	// check the mode of I2C Bus
	if(I2C_Init_Cfg->I2C_MOde == I2C_Mode_I2C_Mode)
	{
		/********************* INIT Timing **************************/
		// Program the peripheral input clock in I2C_CR2 Register Bits 5:0 FREQ[5:0]: Peripheral clock frequency
		I2Cx->I2C_CR2 &= ~(I2C_CR2_FREQ_Msk);
		// Get the Pclk1
		I2Cx->I2C_CR2 |= ((MCAL_RCC_GET_PCLK1_Freq()/1000000) & 0x001F);

		// Configure the clock control registers

		/* Configure the Speed in standard Mode */
		if((I2C_Init_Cfg->I2C_CLK_Speed == I2C_CLK_Speed_SM_100K)||(I2C_Init_Cfg->I2C_CLK_Speed == I2C_CLK_Speed_SM_50K))
		{
			 // Configure standard Mode
			I2Cx->I2C_CCR &= ~(1<<15) ;

			/* calculate the Speed in standard Mode */
			// Tclk/2 = CCR * Tpclk1
			// CCR = (Tclk / 2* Tpclk1)
			// CCR =( Fpclk1 / 2* Fclk)
			I2Cx->I2C_CCR |= (MCAL_RCC_GET_PCLK1_Freq() / (I2C_Init_Cfg->I2C_CLK_Speed <<1));

			/* Configure I2C Rise Time  TRISE register (I2C_TRISE) */
//			For instance: in Sm mode, the maximum allowed SCL rise time is 1000 ns.
//			If, in the I2C_CR2 register, the value of FREQ[5:0] bits is equal to 0x08 and TPCLK1 = 125 ns
//			therefore the TRISE[5:0] bits must be programmed with 09h.
//			(1000 ns / 125 ns = 8 + 1)
			I2Cx->I2C_TRISE = (I2Cx->I2C_TRISE & (~(0x001F)));
			I2Cx->I2C_TRISE |= (((MCAL_RCC_GET_PCLK1_Freq()/1000000)+1) & 0x001F) ;
		}
		else
		{
			// Fast Mode Not supported yet
		}

		/* Configure I2C_CR1 Register */

		I2Cx->I2C_CR1 |= ((I2C_Init_Cfg->General_Call_Address_Detection) | (I2C_Init_Cfg->I2C_ACK_Control) | (I2C_Init_Cfg->I2C_Stretch_Mode) | (I2C_Init_Cfg->I2C_MOde));

		/* Configure I2C OAR1 & OAR2 Registers */

		if(I2C_Init_Cfg->I2C_Slave_Adress.Enable_Dual_Address == 1)
		{
			I2Cx->I2C_OAR2 |= I2C_OAR2_ENDUAL ;
			I2Cx->I2C_OAR2 |= ((I2C_Init_Cfg->I2C_Slave_Adress.Secondary_Slave_Address)<<I2C_OAR2_ADD2_Pos) ;
		}

		I2Cx->I2C_OAR2 |= ((I2C_Init_Cfg->I2C_Slave_Adress.Primary_Slave_Address)<<1) ;

		I2Cx->I2C_OAR2 |= I2C_Init_Cfg->I2C_Slave_Adress.I2C_Slave_Addressing_mode ;

	}
	else
	{
		// SMBus Not Supported
	}


	/* interrupt Enable (Slave Mode) */
	if(I2C_Init_Cfg->I2C_IRQ != I2C_IRQ_NONE)
	{
		I2Cx->I2C_CR2 |= I2C_Init_Cfg->I2C_IRQ ;

		// Enable NVIC
		if(I2Cx == I2C1)
		{
			NVIC_IRQ31_I2C1_EV_ENABLE;
			NVIC_IRQ32_I2C1_ER_ENABLE;
		}
		else if (I2Cx == I2C2)
		{
			NVIC_IRQ33_I2C2_EV_ENABLE;
			NVIC_IRQ34_I2C2_ER_ENABLE;
		}
	}
	// Enable The I2C Peripheral I2C_CR1 Bit 0 PE: Peripheral enable
	I2Cx->I2C_CR1 |= (I2C_CR1_PE);

}

/**================================================================
 * @Fn			-MCAL_I2C_DeInit
 * @brief 		-reset all the I2C Registers
 * @param [in] 	-I2Cx: where x can be (1 or 2 depending on i2c used) to select the I2C peripheral
 * @retval 		-none
 * Note			-none
 */
void MCAL_I2C_DeInit(I2C_TypeDef* I2Cx)
{

	if(I2Cx == I2C1)
	{
		// Disable RCC Clock
		RCC_I2C1_RESET();

		// Disable NVIC
		NVIC_IRQ33_I2C2_EV_DISABLE;
		NVIC_IRQ34_I2C2_ER_DISABLE;
	}
	else if (I2Cx == I2C2)
	{
		// Disable RCC Clock
		RCC_I2C2_RESET();

		// Disable NVIC
		NVIC_IRQ31_I2C1_EV_DISABLE;
		NVIC_IRQ32_I2C1_ER_DISABLE;
	}
}

/**================================================================
 * @Fn				-MCAL_I2C_GPIO_Set_Pins
 * @brief 			- intialize GPIO Pins ,according the recomended GPIO configurations for STM32F103xx peripherals
 * @param [in] 		-I2Cx: where x can be (1 or 2 depending on i2c used) to select the I2C peripheral
 * @retval 			-none
 * Note				-Should enable the corresponding ALT  & GPIO  in RCC clock Also called after MCAL_I2C_Init()
 */
void MCAL_I2C_GPIO_Set_Pins(I2C_TypeDef* I2Cx)
{
	GPIO_PinConfig_t Pin_Cfg ;
	if(I2Cx == I2C1)
	{
		// PB6 : I2C1_SCL  >> Alternate function open drain
		// PB7 : I2C1_SDA  >> Alternate function open drain

		Pin_Cfg.GPIO_PinNumber 	  = GPIO_PIN_6 ;
		Pin_Cfg.GPIO_Mode     	  = GPIO_MODE_OUTPUT_AF_OD ;
		Pin_Cfg.GPIO_Output_Speed = GPIO_SPEED_10M ;
		MCAL_GPIO_Init(GPIOB, &Pin_Cfg);

		Pin_Cfg.GPIO_PinNumber 	  = GPIO_PIN_7 ;
		Pin_Cfg.GPIO_Mode     	  = GPIO_MODE_OUTPUT_AF_OD ;
		Pin_Cfg.GPIO_Output_Speed = GPIO_SPEED_10M ;
		MCAL_GPIO_Init(GPIOB, &Pin_Cfg);

	}
	else 	if(I2Cx == I2C2)
	{
		// PB10 : I2C1_SCL  >> Alternate function open drain
		// PB11 : I2C1_SDA  >> Alternate function open drain

		Pin_Cfg.GPIO_PinNumber 	  = GPIO_PIN_10 ;
		Pin_Cfg.GPIO_Mode     	  = GPIO_MODE_OUTPUT_AF_OD ;
		Pin_Cfg.GPIO_Output_Speed = GPIO_SPEED_10M ;
		MCAL_GPIO_Init(GPIOB, &Pin_Cfg);

		Pin_Cfg.GPIO_PinNumber 	  = GPIO_PIN_11 ;
		Pin_Cfg.GPIO_Mode     	  = GPIO_MODE_OUTPUT_AF_OD ;
		Pin_Cfg.GPIO_Output_Speed = GPIO_SPEED_10M ;
		MCAL_GPIO_Init(GPIOB, &Pin_Cfg);

	}
}
/**================================================================
 * @Fn			-MCAL_I2C_Master_Tx
 * @brief 		-Send data With I2C peripheral in master mode
 * @param [in] 	-I2Cx : where x can be (1 or 2 depending on i2c used) to select the I2C peripheral .
 * @param [in]  -Slave_address
 * @param [in]  -data
 * @param [in]  -data_length
 * @param [in]  -stop : Specifies if Stop Condition is used or not .
 * @param [in]  -start: Specifies if device using start codition first time or reapeted .
 * @retval 		-none
 * Note			-none
 */
void MCAL_I2C_Master_Tx(I2C_TypeDef* I2Cx , uint16_t Slave_address , uint8_t* data , uint32_t data_length , Stop_Condition_t stop , Start_Condition_t start)
{
	/* Set the start bit in I2C_CR1 Register to generate start Condition */
	I2C_Generate_Start(I2Cx, Enable, start);

	/* Check if start Condition is generated or not .
	 * SB=1 , cleared by reading SR1 register followed by writing DR register with Address .
	 * */
	while(!I2C_Get_Flag_Status(I2Cx, I2C_Flag_SB));

	/* Send Slave Address */
	I2C_Send_Address(I2Cx, Slave_address, I2C_Transmitter_mode);

	/* wait till the address has been sent
	 * ADDR=1, cleared by reading SR1 register followed by reading SR2.
	 */
	while(!(I2C_Get_Flag_Status(I2Cx ,I2C_Flag_ADDR)));

	/* Check Event that Master transmite byte
	 * TRA , MSL , BUSY , TXE
	 */
	I2C_Get_Event_Status(I2Cx , I2C_Event_Master_Byte_Transmitting);

	/* Send Data */
	uint32_t i ;
	for(i=0 ; i<data_length ; i++)
	{
		/* Write Data in DR register */
		I2Cx->I2C_DR = data[i];

		/*When the acknowledge pulse is received, the TxE bit is set by hardware
		 * TxE=1, shift register not empty, d. ata register empty, cleared by writing DR register
		 */
		while(!I2C_Get_Flag_Status(I2Cx, I2C_Flag_TxE));
	}

	/* Stop Condition */
	if(stop == with_Stop)
	{
		/* Generate Stop Condition*/
		I2C_Generate_Stop(I2Cx, Enable);
	}
}
/**================================================================
 * @Fn			-MCAL_I2C_Master_Rx
 * @brief 		-Recieve data With I2C peripheral in master mode
 * @param [in] 	-I2Cx : where x can be (1 or 2 depending on i2c used) to select the I2C peripheral .
 * @param [in]  -Slave_address
 * @param [in]  -data
 * @param [in]  -data_length
 * @param [in]  -stop : Specifies if Stop Condition is used or not .
 * @param [in]  -start: Specifies if device using start codition first time or reapeted .
 * @retval 		-none
 * Note			-none
 */
void MCAL_I2C_Master_Rx(I2C_TypeDef* I2Cx , uint16_t Slave_address , uint8_t* data , uint32_t data_length , Stop_Condition_t stop , Start_Condition_t start)
{
	/* Set the start bit in I2C_CR1 Register to generate start Condition */
	I2C_Generate_Start(I2Cx, Enable, start);

	/* Check if start Condition is generated or not .
	 * SB=1 , cleared by reading SR1 register followed by writing DR register with Address .
	 * */
	while(!I2C_Get_Flag_Status(I2Cx, I2C_Flag_SB));

	/* Send Slave Address */
	I2C_Send_Address(I2Cx, Slave_address, I2C_Receiver_mode);

	/* wait till the address has been sent
	 * ADDR=1, cleared by reading SR1 register followed by reading SR2.
	 */
	while(!(I2C_Get_Flag_Status(I2Cx ,I2C_Flag_ADDR)));

	/*Enable ACK*/
	I2C_ACK_Control(I2Cx, Enable);

	if(data_length)
	{
		uint32_t i ;
		for(i=0 ; i < data_length ; i++)
		{
//			 After each byte the interface generates in
//			sequence:
//			1. An acknowledge pulse if the ACK bit is set
//			2. The RxNE bit is set

			/* Wait RXNE Flag to Set */
			while(!(I2C_Get_Flag_Status(I2Cx, I2C_Flag_RxNE)));

			data[i] = I2Cx->I2C_DR ;
		}

//		To generate the nonacknowledge pulse after the last received data byte, the ACK bit
//		must be cleared just after reading the second last data byte (after second last RxNE event)
		I2C_ACK_Control(I2Cx, Disable);
	}


	/* Stop Condition */
	if(stop == with_Stop)
	{
		/* Generate Stop Condition*/
		I2C_Generate_Stop(I2Cx, Enable);
	}

	/* Return the ACK To the Global Configuration */
	uint8_t index = (I2Cx == I2C1)? 0 : 1 ;
	if(G_I2C_Cfg[index].I2C_ACK_Control == I2C_ACK_Control_Enable)
	{
		// Enable ACK Control
		I2Cx->I2C_CR1 |= I2C_CR1_ACK ;
	}

}
/**================================================================
 * @Fn			-MCAL_I2C_Slave_Tx
 * @brief 		-Send data With I2C peripheral in slave mode
 * @param [in] 	-I2Cx : where x can be (1 or 2 depending on i2c used) to select the I2C peripheral .
 * @param [in]  -data
 * @retval 		-none
 * Note			-none
 */
void MCAL_I2C_Slave_Tx(I2C_TypeDef* I2Cx , uint8_t* data)
{
	/* Write data in DR register */
	I2Cx->I2C_DR = *data ;
}
/**================================================================
 * @Fn			-MCAL_I2C_Slave_Rx
 * @brief 		-Recieve data With I2C peripheral in slave mode
 * @param [in] 	-I2Cx : where x can be (1 or 2 depending on i2c used) to select the I2C peripheral .
 * @param [in]  -data
 * @retval 		-none
 * Note			-none
 */
void MCAL_I2C_Slave_Rx(I2C_TypeDef* I2Cx , uint8_t* data)
{
	/* Read data From DR register */
	*data = I2Cx->I2C_DR ;
}
/*************************
 *	Generic APIS
 *************************/
void I2C_Generate_Start(I2C_TypeDef* I2Cx , fanctional_State_t S_state  , Start_Condition_t start )
{
	if(start != Reapeted_Start)
		// wait if Bus is Busy
		while(I2C_Get_Flag_Status(I2Cx , I2C_Flag_Busy));

	//	Bit 8 START: Start generation
	//	This bit is set and cleared by software and cleared by hardware when start is sent or PE=0.
	//	In Master Mode:
	//	0: No Start generation
	//	1: Repeated start generation
	//	In Slave mode:
	//	0: No Start generation
	//	1: Start generation when the bus is free
	if(S_state != Disable)
	{
		// generate the start condition
		I2Cx->I2C_CR1 |= I2C_CR1_START ;
	}
	else
	{
		// Disable the start condition generation
		I2Cx->I2C_CR1 &= ~(I2C_CR1_START) ;
	}

}

void I2C_Generate_Stop(I2C_TypeDef* I2Cx , fanctional_State_t P_state)
{
	if(P_state != Disable)
	{
		// generate the Stop condition
		I2Cx->I2C_CR1 |= I2C_CR1_STOP ;
	}
	else
	{
		// Disable the Stop condition generation
		I2Cx->I2C_CR1 &= ~(I2C_CR1_STOP) ;
	}
}

void I2C_ACK_Control(I2C_TypeDef* I2Cx , fanctional_State_t ACK_state)
{
	if(ACK_state != Disable)
	{
		// Enable ACK Control
		I2Cx->I2C_CR1 |= I2C_CR1_ACK ;
	}
	else
	{
		// Disable  ACK Control
		I2Cx->I2C_CR1 &= ~(I2C_CR1_ACK) ;
	}
}
Flag_Status_t I2C_Get_Flag_Status(I2C_TypeDef* I2Cx  , Status_flag_t flag)
{
	Flag_Status_t status = Reset ;


	if(flag < I2C_Flag_MSL )
	{
		status = ((I2Cx->I2C_SR1)&(flag))? Set : Reset ;
	}
	else
	{
		status = ((I2Cx->I2C_SR2)&(flag>>15))? Set : Reset ;
	}

	/* in case of ADDR Flag must Read SR2 After SR1
	 * ADDR=1, cleared by reading SR1 register followed by reading SR2.
	 * */
//	if(flag == I2C_Flag_ADDR)
//	{
//		uint32_t dummy_read ;
//		dummy_read = I2Cx->I2C_SR2 ;
//	}
	return status ;
}
Flag_Status_t I2C_Get_Event_Status(I2C_TypeDef* I2Cx , Event_type_t Event)
{
	Flag_Status_t Event_Status ;
	switch(Event)
	{
	case I2C_Event_Master_Byte_Transmitting :
		/* Check Event that Master transmite byte
		 * TRA , MSL , BUSY , TXE
		 */
		Event_Status = ((I2C_Get_Flag_Status(I2Cx, I2C_Flag_TRA)) & (I2C_Get_Flag_Status(I2Cx, I2C_Flag_TxE)) & (I2C_Get_Flag_Status(I2Cx, I2C_Flag_Busy)) & I2C_Get_Flag_Status(I2Cx, I2C_Flag_MSL))? Set : Reset;
		break;
	}
	return Event_Status ;
}
// Support 7Bit_address only
void I2C_Send_Address(I2C_TypeDef* I2Cx , uint16_t address ,I2C_Direction_t Direction_mode)
{
	// shift address by 1 to leave LSB for Direction
	address = (address<<1);

//	The master can decide to enter Transmitter or Receiver mode depending on the LSB of the
//	slave address sent.
//	In 7-bit addressing mode,
//	– To enter Transmitter mode, a master sends the slave address with LSB reset.
//	– To enter Receiver mode, a master sends the slave address with LSB set.

	if(Direction_mode == I2C_Transmitter_mode)
	{
		/* Reset LSB to enter Transmitter mode */
		address &= ~(1<<0);
	}
	else
	{
		/* Set LSB to enter Receiver mode */
		address |= (1<<0);
	}

	/* Write address in DR register */
	I2Cx->I2C_DR = address;
}

/*************************
 *	 ISR
 *************************/
void I2C1_EV_IRQHandler (void)
{
	uint32_t dummy_Read ;

	if(G_I2C_Cfg[I2C1_index].Device_mode == slave)
	{
		/* handle interrupt generated by Stop Event*/
		if(I2C_Get_Flag_Status(I2C1, I2C_Flag_STOPF))
		{
			/* ADDR=1, cleared by reading SR1 register followed by reading SR2*/
			dummy_Read = I2C1->I2C_SR1 ;
			dummy_Read = I2C1->I2C_SR2 ;
			G_I2C_Cfg[I2C1_index].P_Slave_Event_CallBack(I2C_EV_ADDR_MATCHED);
		}

		/* handle interrupt generated by Address matched Event*/
		if(I2C_Get_Flag_Status(I2C1, I2C_Flag_ADDR))
		{
			/* (STOPF == 1) {READ SR1; WRITE CR1} */
			dummy_Read = I2C1->I2C_SR1 ;
			I2C1->I2C_CR1 = 0x0000 ;
			G_I2C_Cfg[I2C1_index].P_Slave_Event_CallBack(I2C_EV_STOP);
		}

		/* handle interrupt generated by TXE Event*/
		if(I2C_Get_Flag_Status(I2C1, I2C_Flag_TxE))
		{
			// The APP should use (MCAL_I2C_Slave_Tx) to send data
			if(I2C_Get_Flag_Status(I2C1, I2C_Flag_TRA))
			G_I2C_Cfg[I2C1_index].P_Slave_Event_CallBack(I2C_EV_DATA_REQ);
		}

		/* handle interrupt generated by RXNE Event*/
		if(I2C_Get_Flag_Status(I2C1, I2C_Flag_RxNE))
		{
			// The APP should use (MCAL_I2C_Slave_Rx) to Recieve data
			if(!I2C_Get_Flag_Status(I2C1, I2C_Flag_TRA))
			G_I2C_Cfg[I2C1_index].P_Slave_Event_CallBack(I2C_EV_DATA_RCV);
		}

	}
	else
	{
		// the master mode not supported
	}

}
void I2C1_ER_IRQHandler (void)
{

}
void I2C2_EV_IRQHandler (void)
{
	uint32_t dummy_Read ;

	if(G_I2C_Cfg[I2C2_index].Device_mode == slave)
	{
		/* handle interrupt generated by Stop Event*/
		if(I2C_Get_Flag_Status(I2C2, I2C_Flag_STOPF))
		{
			/* ADDR=1, cleared by reading SR1 register followed by reading SR2*/
			dummy_Read = I2C2->I2C_SR1 ;
			dummy_Read = I2C2->I2C_SR2 ;
			G_I2C_Cfg[I2C2_index].P_Slave_Event_CallBack(I2C_EV_ADDR_MATCHED);
		}

		/* handle interrupt generated by Address matched Event*/
		if(I2C_Get_Flag_Status(I2C2, I2C_Flag_ADDR))
		{
			/* (STOPF == 1) {READ SR1; WRITE CR1} */
			dummy_Read = I2C2->I2C_SR1 ;
			I2C2->I2C_CR1 = 0x0000 ;
			G_I2C_Cfg[I2C2_index].P_Slave_Event_CallBack(I2C_EV_STOP);
		}

		/* handle interrupt generated by TXE Event*/
		if(I2C_Get_Flag_Status(I2C2, I2C_Flag_TxE))
		{
			// The APP should use (MCAL_I2C_Slave_Tx) to send data
			if(I2C_Get_Flag_Status(I2C2, I2C_Flag_TRA))
			G_I2C_Cfg[I2C2_index].P_Slave_Event_CallBack(I2C_EV_DATA_REQ);
		}

		/* handle interrupt generated by RXNE Event*/
		if(I2C_Get_Flag_Status(I2C2, I2C_Flag_RxNE))
		{
			// The APP should use (MCAL_I2C_Slave_Rx) to Recieve data
			if(!I2C_Get_Flag_Status(I2C2, I2C_Flag_TRA))
			G_I2C_Cfg[I2C2_index].P_Slave_Event_CallBack(I2C_EV_DATA_RCV);
		}

	}
	else
	{
		// the master mode not supported
	}

}
void I2C2_ER_IRQHandler (void)
{

}



