/*
** ---------------------------------------------------------------------------
** <filename>.c
** Description: Firmware for G168 Command interpreter
** Version: 1.02
**
** Author: Gregory GOSCINIAK
** email: gregory.gosciniak@st.com
** Date: 2017-10-01
**
**
** THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY 
** KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
** IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
** PARTICULAR PURPOSE.
**
** Copyright (c) 2015-2018 STMicroelectronics. All rights reserved.
** ---------------------------------------------------------------------------
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_ll_bus.h"
//#include "stm32f0xx_ll_rcc.h"
//#include "stm32f0xx_ll_system.h"
//#include "stm32f0xx_ll_utils.h"
#include "stm32f0xx_ll_gpio.h"
//#include "stm32f0xx_ll_exti.h"
//#include "stm32f0xx_ll_usart.h"
//#include "stm32f0xx_ll_pwr.h"

/* Includes ------------------------------------------------------------------*/
//#include "main.h"
//#include "stm32f0xx_hal.h"

#include "Hardware_configuration.h" //for I2CBUS_MAX, USBPORT_MAX
#include "printf_debug.h"

int Gpio_Check_I2C1_external_pullup(void);
int Gpio_Check_I2C2_external_pullup(void);

//--------------------
//STM32F072 I2C pinout
//https://developer.mbed.org/platforms/ST-Nucleo-F072RB/

//I2C1_SCL = PB8
//I2C1_SDA = PB9

//I2C2_SCL = PB10
//I2C2_SDA = PB11
//--------------------
static char I2C_pullup_Status[I2CBUS_MAX] = {-1, -1};


int Gpio_Check_I2C_external_pullup(void)
{
    int Status;
    //DBGPRINTF("Testing External pull-up resistors for I2C bus...\r\n");
    
#if (I2CBUS_MAX >= 1)
    Status = Gpio_Check_I2C1_external_pullup();
    I2C_pullup_Status[0] = Status;
        
//    if(Status == 0) //OK
//    {
//        DBGPRINTF("I2C1: pull-up OK\r\n");
//    }
//    else
//    {
//        DBGPRINTF("I2C1: NO pull-up\r\n");
//    }
#endif
    
#if (I2CBUS_MAX >= 2)
    Status = Gpio_Check_I2C2_external_pullup();
    I2C_pullup_Status[1] = Status;
    
//    if(Status == 0) //OK
//    {
//        DBGPRINTF("I2C2: pull-up OK\r\n");
//    }
//    else
//    {
//        DBGPRINTF("I2C2: NO pull-up\r\n");
//    }
#endif
    
 
    return Status;
}

int Gpio_I2C_external_pullup_GetStatus(int I2cBus)
{
    int Status = 0xFF;
    
    
    if(I2cBus < I2CBUS_MAX)
    {
        Status = I2C_pullup_Status[I2cBus];
    }
    
    if(Status == 0) //OK
    {
        DBGPRINTF("I2cBus-%i: pull-up OK\r\n", I2cBus);
    }
    else
    {
        DBGPRINTF("I2cBus-%i: NO pull-up ! (error %i)\r\n", I2cBus, Status);
    }
    
    return Status;
}

int Gpio_Check_I2C1_external_pullup(void)
{
#ifdef I2C1
    
    uint32_t GpioPin;
    
    /* Enable the peripheral clock of GPIOB */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    
    
    /* Configure SCL Pin as :  */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_8, LL_GPIO_PULL_NO);
    
    GpioPin = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_8);
    if(GpioPin == 0)
    { 
        //no external pull-up
        return -1; //error
    }
    
    
    /* Configure SDA Pin as :  */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_9, LL_GPIO_PULL_NO);
    
    GpioPin = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_9);
    if(GpioPin == 0)
    { 
        //no external pull-up
        return -2; //error
    }
    
#endif
    return 0; //OK
}

int Gpio_Check_I2C2_external_pullup(void)
{
#ifdef I2C2
    
    uint32_t GpioPin;
    
    /* Enable the peripheral clock of GPIOB */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    
    
    /* Configure SCL Pin as :  */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_10, LL_GPIO_PULL_NO);
    
    GpioPin = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_10);
    if(GpioPin == 0)
    { 
        //no external pull-up
        return -1; //error
    }
    
    
    /* Configure SDA Pin as :  */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_INPUT);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_NO);
    
    GpioPin = LL_GPIO_IsInputPinSet(GPIOB, LL_GPIO_PIN_11);
    if(GpioPin == 0)
    { 
        //no external pull-up
        return -2; //error
    }
    
#endif
    
    return 0; //OK
}

void Configure_GPIO_for_I2C2_test1(void) //to realease potential previous I2C transaction (during debug tests)
{
#if 0 //from example
    /* Enable the LED2 Clock */
    LED2_GPIO_CLK_ENABLE();
    
    /* Configure IO in output push-pull mode to drive external LED2 */
    LL_GPIO_SetPinMode(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_MODE_OUTPUT);
    /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
    //LL_GPIO_SetPinOutputType(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_OUTPUT_PUSHPULL);
    /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
    //LL_GPIO_SetPinSpeed(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_SPEED_FREQ_LOW);
    /* Reset value is LL_GPIO_PULL_NO */
    //LL_GPIO_SetPinPull(LED2_GPIO_PORT, LED2_PIN, LL_GPIO_PULL_NO);
    
#endif
    
    
    /* Enable the peripheral clock of GPIOB */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    
    
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_10); //level high by default
    
    /* Configure SCL Pin as : Alternate function, High Speed, Open drain, Pull up */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_OUTPUT);
    //LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_10, LL_GPIO_AF_1);
    //LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_10, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_10, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_10, LL_GPIO_PULL_UP);
    
    /* Configure SDA Pin as : Alternate function, High Speed, Open drain, Pull up */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_INPUT);
    //LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_11, LL_GPIO_AF_1);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_11, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_11, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_UP);
    
    for(int i=0; i<10*2; i++)
    {
        volatile int dummy;
        
        LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_10);
        for(dummy=0; dummy<20; dummy++); //wait
    }
    
    //reset pin status to floating:
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_10, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_10, LL_GPIO_MODE_INPUT);
    
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_11, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_11, LL_GPIO_MODE_INPUT);
}



void Configure_GPIO_for_I2C1_test1(void) //to release potential previous I2C transaction
{
    
    
    /* Enable the peripheral clock of GPIOB */
    LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOB);
    
    
    LL_GPIO_SetOutputPin(GPIOB, LL_GPIO_PIN_8); //level high by default
    
    /* Configure SCL Pin as : Alternate function, High Speed, Open drain, Pull up */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_OUTPUT);
    //LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_8, LL_GPIO_AF_1);
    //LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_8, LL_GPIO_SPEED_FREQ_LOW);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_8, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_8, LL_GPIO_PULL_UP);
    
    /* Configure SDA Pin as : Alternate function, High Speed, Open drain, Pull up */
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_INPUT);
    //LL_GPIO_SetAFPin_0_7(GPIOB, LL_GPIO_PIN_9, LL_GPIO_AF_1);
    LL_GPIO_SetPinSpeed(GPIOB, LL_GPIO_PIN_9, LL_GPIO_SPEED_FREQ_HIGH);
    LL_GPIO_SetPinOutputType(GPIOB, LL_GPIO_PIN_9, LL_GPIO_OUTPUT_OPENDRAIN);
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_9, LL_GPIO_PULL_UP);
    
    for(int i=0; i<10*2; i++)
    {
        volatile int dummy;
        
        LL_GPIO_TogglePin(GPIOB, LL_GPIO_PIN_8);
        for(dummy=0; dummy<20; dummy++); //wait
    }
    
    //reset pin status to floating:
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_8, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_8, LL_GPIO_MODE_INPUT);
    
    LL_GPIO_SetPinPull(GPIOB, LL_GPIO_PIN_9, LL_GPIO_PULL_NO);
    LL_GPIO_SetPinMode(GPIOB, LL_GPIO_PIN_9, LL_GPIO_MODE_INPUT);
}

