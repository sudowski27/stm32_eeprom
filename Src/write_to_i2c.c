#include "stm32f4xx_hal.h"

extern I2C_HandleTypeDef hi2c1;
void write_byte_to_24LC01B(uint16_t DevAddress, uint8_t Data, uint16_t Size) {
	HAL_I2C_Master_Transmit(&hi2c1, DevAddress, Data, Size, HAL_MAX_DELAY);
}
