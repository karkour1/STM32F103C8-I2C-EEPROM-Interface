/*
 * Stm32F103C8_I2C_Driver.h
 *
 *  Created on: May 2, 2024
 *  Author: ABDULLAH KARKOUR
 */
#ifndef INC_STM32F103C8_I2C_DRIVER_H_
#define INC_STM32F103C8_I2C_DRIVER_H_

/*
* ===============================================
* 					Includes
* ===============================================
*/
#include "Stm32_F103X6.h"
#include "Stm32_F103C6_gpio_driver.h"
#include "Stm32_F103C6_EXTI_driver.h"
#include "Stm32_F103C6_RCC_driver.h"

/*
* ===============================================
* 	User type definitions (structures)
* ===============================================
*/
typedef struct{

	uint16_t	Enable_Dual_Address 	   ; // Enable = 1 , Disable = 0
	uint16_t	Primary_Slave_Address  	   ;
	uint16_t	Secondary_Slave_Address    ;
	uint16_t	I2C_Slave_Addressing_mode  ; // @ref I2C_Slave_Addressing_mode
}I2C_Slave_Device_Addressing_mode;

typedef enum {
	master,
	slave
}Device_mode_t;
typedef enum{

	I2C_EV_STOP,
	I2C_ERROR_AF,
	I2C_EV_ADDR_MATCHED,
	I2C_EV_DATA_REQ,		// the APP Layer should send data
	I2C_EV_DATA_RCV			// the APP layer should Recieve data


}Slave_State;

typedef enum{
	with_Stop,
	without_Stop
}Stop_Condition_t;

typedef enum {
	Start,
	Reapeted_Start
}Start_Condition_t;

typedef enum {
	Disable ,
	Enable
}fanctional_State_t ;

typedef enum{
	Reset,
	Set
}Flag_Status_t;

typedef enum {
	I2C_Transmitter_mode ,
	I2C_Receiver_mode
}I2C_Direction_t;
typedef enum{
	I2C_Flag_SB   	= I2C_SR1_SB    	,
	I2C_Flag_ADDR 	= I2C_SR1_ADDR  	, // ADDR: Address sent (master mode)/matched (slave mode)
	I2C_Flag_BTF    = I2C_SR1_BTF    	,
	I2C_Flag_ADD10  = I2C_SR1_ADD10    	,
	I2C_Flag_STOPF  = I2C_SR1_STOPF    	,
	I2C_Flag_RxNE   = I2C_SR1_RXNE    	,
	I2C_Flag_TxE    = I2C_SR1_TXE    	,
	I2C_Flag_BERR   = I2C_SR1_BERR    	,
	I2C_Flag_ARLO   = I2C_SR1_ARLO    	,
	I2C_Flag_AF     = I2C_SR1_AF    	,
	I2C_Flag_OVR    = I2C_SR1_OVR   	,
	I2C_Flag_PECERR = I2C_SR1_PECERR   	,
	I2C_Flag_MSL  	= I2C_SR2_MSL<<15	,
	I2C_Flag_Busy 	= I2C_SR2_BUSY<<15  ,
	I2C_Flag_TRA 	= I2C_SR2_TRA<<15   ,
	I2C_Flag_GENCALL= I2C_SR2_GENCALL<<15,
	I2C_Flag_DUALF	= I2C_SR2_DUALF<<15

}Status_flag_t;

typedef enum {
	I2C_Event_Master_Byte_Transmitting

}Event_type_t;
typedef struct {

	uint32_t		I2C_CLK_Speed;								//  I2C supports the standard mode (Sm, up to 100 kHz) and Fm mode (Fm, up to 400 kHz).
																//  This Prameter must be set From @ref I2C_CLK_Speed_define

	uint32_t		I2C_Stretch_Mode;							//  Enable or Disable Stretching in cLock .
																//  This Prameter must be set From @ref I2C_Stretch_Mode_define

	uint32_t		I2C_MOde;		    						//  Specifies the mode of I2C Bus (I2C mode, SMBus mode)
																//  This Prameter must be set From @ref I2C_Mode_define

	uint32_t		I2C_ACK_Control;							//  Specifies the ACK Mode (Returned or not)
																//  This Prameter must be set From @ref I2C_ACK_Control_define

	uint32_t		I2C_IRQ;									//  Specifies Interrupt is Enable or Disable ,
																//  in slave mode you have to enable interrupt to avoid stuck in polling till master send without doing any thing .
																//  This Prameter must be set From @ref I2C_IRQ_define

	uint32_t		General_Call_Address_Detection;		    	//  Specifies General call enable
																//  This Prameter must be set From @ref I2C_GC_define

	I2C_Slave_Device_Addressing_mode		I2C_Slave_Adress;

	Device_mode_t							Device_mode ;   // master or slave

	void (*P_Slave_Event_CallBack)(Slave_State State);

}I2C_Device_CFG_t;

/*
* ===============================================
* 	Macros Configuration References
* ===============================================
*/

//@ref I2C_CLK_Speed_define
//Supports different communication speeds:
//– Standard Speed (up to 100 kHz)
//– Fast Speed (up to 400 kHz)

#define I2C_CLK_Speed_SM_50K			(50000U)
#define I2C_CLK_Speed_SM_100K			(100000U)
#define I2C_CLK_Speed_FM_200K			(200000U) // Fast Mode Not supported yet
#define I2C_CLK_Speed_FM_400K			(400000U) // Fast Mode Not supported yet

//@ref I2C_Stretch_Mode_define
#define I2C_Stretch_Mode_Enable			(0x00000000)
#define I2C_Stretch_Mode_Disable		I2C_CR1_NOSTRETCH

// @ref I2C_Mode_define
#define I2C_Mode_I2C_Mode 				0
#define I2C_Mode_I2C_SMBus				I2C_CR1_SMBUS

//@ref I2C_Slave_Addressing_mode
#define I2C_Slave_Addressing_mode_7Bits		0
#define I2C_Slave_Addressing_mode_10Bits	(1<<15)

//@ref I2C_ACK_Control_define
#define I2C_ACK_Control_Enable			I2C_CR1_ACK
#define I2C_ACK_Control_Disable			(0x0000)

//@ref I2C_IRQ_define
#define I2C_IRQ_NONE					(0x00000000)
#define I2C_IRQ_EV_Enable				(I2C_CR2_ITEVTEN)
#define I2C_IRQ_EV_TxE_RxNE_Enable		(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN)
#define I2C_IRQ_ERR_Enable				(I2C_CR2_ITERREN)
#define I2C_IRQ_EV_ERR_Enable			(I2C_CR2_ITEVTEN | I2C_CR2_ITBUFEN | I2C_CR2_ITERREN)

//@ref I2C_GC_define
#define I2C_GC_Enable				I2C_CR1_ENGC
#define I2C_GC_Disable				0x00000000

/*
* ===============================================
* APIs Supported by "MCAL I2C DRIVER"
* ===============================================
*/

void MCAL_I2C_Init(I2C_TypeDef* I2Cx , I2C_Device_CFG_t* I2C_Init_Cfg);
void MCAL_I2C_DeInit(I2C_TypeDef* I2Cx);

void MCAL_I2C_GPIO_Set_Pins(I2C_TypeDef* I2Cx);

// Master with polling mechanism
void MCAL_I2C_Master_Tx(I2C_TypeDef* I2Cx , uint16_t Slave_address , uint8_t* data , uint32_t data_length , Stop_Condition_t stop , Start_Condition_t start);
void MCAL_I2C_Master_Rx(I2C_TypeDef* I2Cx , uint16_t Slave_address , uint8_t* data , uint32_t data_length , Stop_Condition_t stop , Start_Condition_t start);

// Slave With Interrupt
void MCAL_I2C_Slave_Tx(I2C_TypeDef* I2Cx , uint8_t* data);
void MCAL_I2C_Slave_Rx(I2C_TypeDef* I2Cx , uint8_t* data);

// Generic APIS
void I2C_Send_Address(I2C_TypeDef* I2Cx , uint16_t address ,I2C_Direction_t Direction_mode) ;
void I2C_Generate_Start(I2C_TypeDef* I2Cx , fanctional_State_t S_state , Start_Condition_t start);
void I2C_Generate_Stop(I2C_TypeDef* I2Cx , fanctional_State_t P_state);
void I2C_ACK_Control(I2C_TypeDef* I2Cx , fanctional_State_t ACK_state);
Flag_Status_t I2C_Get_Flag_Status(I2C_TypeDef* I2Cx  , Status_flag_t flag) ;
Flag_Status_t I2C_Get_Event_Status(I2C_TypeDef* I2Cx , Event_type_t Event);


#endif /* INC_STM32F103C8_I2C_DRIVER_H_ */
