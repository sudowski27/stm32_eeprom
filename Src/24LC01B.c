#include "stm32f4xx_ll_i2c.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_usart.h"
#include "stm32f4xx_ll_rcc.h"


void i2c_init() {
	// Initialize bus
	LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_I2C1);

	// Initialize GPIO's pins
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_7, LL_GPIO_MODE_ALTERNATE);
	LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_ALTERNATE);

	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_7, LL_GPIO_OUTPUT_OPENDRAIN);
	LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_OPENDRAIN);

	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_7, LL_GPIO_SPEED_FREQ_HIGH);
	LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_HIGH);

	LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_7,  LL_GPIO_AF_4);
	LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_8,  LL_GPIO_AF_4);

	// Configure I2C
	LL_I2C_Enable(I2C1);
	LL_I2C_SetOwnAddress1(I2C1, 0x00, LL_I2C_OWNADDRESS1_7BIT);
	LL_I2C_EnableClockStretching(I2C1);
}


void debug_diode_init() {
	// Function to check ACK or NACK
	LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
	LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
	LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
	LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_LOW);
}


void debug_diode_on() {
	LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5);
}


void debug_diode_off(){
	LL_GPIO_ResetOutputPin(GPIOA, LL_GPIO_PIN_5);
}


void i2c_master_transmit(uint8_t slave_address, uint8_t *buffer, uint16_t size) {
	// start condition
	LL_I2C_GenerateStartCondition(I2C1);

	// Indicate the status of Start Bit
	while(!LL_I2C_IsActiveFlag_SB(I2C1)) {} // when is True escape from this loop

	LL_I2C_TransmitData8(I2C1, slave_address); // send address of slave

	while(!LL_I2C_IsActiveFlag_ADDR(I2C1)) {} // indicate the status of address sent

	LL_I2C_ClearFlag_ADDR(I2C1); // clear address matched flag

	// send data
	while ( size--) {
		while(!LL_I2C_IsActiveFlag_TXE(I2C1)) {}  // wait until the data register is empty

		LL_I2C_TransmitData8(I2C1, *buffer); // send data

		buffer ++; // shift pointer to next data
	}
	// LL_I2C_NACK  Flag
	LL_I2C_GenerateStopCondition(I2C1); // generate a STOP condition after the current byte transfer
}


void master_transmit_24lc01b(uint8_t *buffer, uint16_t size) {
	extern uint8_t address_24lc01b;
	uint8_t address = address_24lc01b;  // 0x50

	i2c_master_transmit(address, buffer, size);
}
