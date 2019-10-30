
/* Includes ------------------------------------------------------------------*/

//#include "I2C.h"
#include "I2C_ReadWrite.h"
#include "Hardware_configuration.h"
#include "stm32f0xx_hal.h"
//#include "stm32f0xx_hal_def.h" //for HAL_StatusTypeDef

//#include "stm32f0xx_hal_conf.h" //for stm32f0xx_hal_i2c.h
//#include "stm32f0xx_hal_i2c.h" //for HAL_I2C_Mem_Read

#define ADDRESS_SIZE  I2C_MEMADD_SIZE_8BIT
#define I2C_TIMEOUT 1000 //1sec

I2C_HandleTypeDef *hi2c[I2CBUS_MAX];



int I2cRead(uint8_t I2cBus, uint16_t I2cDeviceID_7bit, uint16_t Address, uint8_t *pDataR , uint16_t ByteCount)
{
    return  HAL_I2C_Mem_Read(hi2c[I2cBus], (I2cDeviceID_7bit << 1), Address, ADDRESS_SIZE, pDataR, ByteCount, I2C_TIMEOUT);
}


int I2cWrite(uint8_t I2cBus, uint16_t I2cDeviceID_7bit, uint16_t Address, uint8_t *pDataW, uint16_t Length)
{
    return  HAL_I2C_Mem_Write(hi2c[I2cBus], (I2cDeviceID_7bit << 1), Address, ADDRESS_SIZE, pDataW, Length, I2C_TIMEOUT);
}

