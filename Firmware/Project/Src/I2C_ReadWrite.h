
//#include "stm32f0xx_hal.h"
#include <stdint.h> //for uint8_t, uint16_t, uint32_t

#define STUSB4700_I2C_ADDR1_7BIT 0x28
#define STUSB4700_I2C_ADDR2_7BIT 0x29

#define STUSB4700_I2C_ADDR1_8BIT_WR 0x50 //including Write bit (R/W bit = '0')
#define STUSB4700_I2C_ADDR1_8BIT_RD 0x51 //including Read  bit (R/W bit = '1')
#define STUSB4700_I2C_ADDR2_8BIT_WR 0x52 //including Write bit (R/W bit = '0')
#define STUSB4700_I2C_ADDR2_8BIT_RD 0x53 //including Read  bit (R/W bit = '1')


int I2cRead(uint8_t Bus, uint16_t I2cDeviceID_7bit, uint16_t Address, uint8_t *pDataR , uint16_t ByteCount);
int I2cWrite(uint8_t Bus, uint16_t I2cDeviceID_7bit, uint16_t Address, uint8_t *pDataW, uint16_t Length);


#define I2C_OK 0
#define I2C_ERR 0x2C //0x2C = 44 for a general I2C error
#define I2C_ERR_BUSY 1
#define I2C_ERR_ACK 2
#define I2C_ERR_TIMEOUT 3
