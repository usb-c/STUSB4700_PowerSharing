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

int Gpio_Check_I2C_external_pullup(void);
int Gpio_I2C_external_pullup_GetStatus(int I2cBus);

int Check_I2C1_external_pullup(void);
int Check_I2C2_external_pullup(void);

void Configure_GPIO_for_I2C2_test1(void);
void Configure_GPIO_for_I2C1_test1(void);
