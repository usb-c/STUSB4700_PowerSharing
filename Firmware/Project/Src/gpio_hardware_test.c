/**
  ******************************************************************************
  * @file    Examples_LL/GPIO/GPIO_InfiniteLedToggling/Src/main.c
  * @author  MCD Application Team
  * @brief   This example describes how to configure and use GPIOs through
  *          the STM32F0xx  GPIO LL API.
  *          Peripheral initialization done using LL unitary services functions.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f0xx_ll_bus.h" //for LL_AHB1_GRP1_EnableClock
//#include "stm32f0xx_ll_rcc.h"
//#include "stm32f0xx_ll_system.h"
#include "stm32f0xx_ll_utils.h" //for LL_MAX_DELAY
#include "stm32f0xx_ll_gpio.h"


/** @addtogroup STM32F0xx_LL_Examples
  * @{
  */

/** @addtogroup GPIO_InfiniteLedToggling
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
#define LED2_PIN                           LL_GPIO_PIN_5
#define LED2_GPIO_PORT                     GPIOA
#define LED2_GPIO_CLK_ENABLE()             LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA)


/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
void     Configure_GPIO1(void);
void     Configure_GPIO2(void);
static void LL_mDelay(uint32_t Delay);

/* 
Pin information:
J5 pin 7 = CS  = STM32 pin 16 = STM32 PA6
J5 pin 3 = SCK = STM32 pin 15 = STM32 PA5
*/

/* Private functions ---------------------------------------------------------*/

/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int Gpio_basic_hardware_test_toggle(void)
{
  /* Configure the system clock to 48 MHz */
  //SystemClock_Config();
  
  /* -2- Configure IO in output push-pull mode to drive external LED */
  Configure_GPIO1();
  Configure_GPIO2();


  //toggling period signal_1 = 12ms => 83Hz
  //toggling period signal_2 = 6ms => 166Hz

  /* Toggle IO in an infinite loop */
  for(int i=0; i<10; i++)
  {
    LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_6); //unused pin SPI CS
    LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5); //unused pin SPI SCK
    
    /* Insert delay 250 ms */
    LL_mDelay(5);
  }
  
  LL_mDelay(10);

  for(int i=0; i<10; i++)
  {
    LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_6); //unused pin SPI CS
    LL_GPIO_TogglePin(GPIOA, LL_GPIO_PIN_5); //unused pin SPI SCK
    
    /* Insert delay 250 ms */
    LL_mDelay(2);
  }
  
  return 0;
}


/**
  * @brief  This function configures GPIO
  * @note   Peripheral configuration is minimal configuration from reset values.
  *         Thus, some useless LL unitary functions calls below are provided as
  *         commented examples - setting is default configuration from reset.
  * @param  None
  * @retval None
  */
void Configure_GPIO1(void)
{
  /* Enable the LED2 Clock */
  //LED2_GPIO_CLK_ENABLE();
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /* Configure IO in output push-pull mode to drive external LED2 */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_6, LL_GPIO_MODE_OUTPUT);
  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_6, LL_GPIO_OUTPUT_PUSHPULL);
  /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_6, LL_GPIO_SPEED_FREQ_LOW);
  /* Reset value is LL_GPIO_PULL_NO */
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_6, LL_GPIO_PULL_NO);
}

void Configure_GPIO2(void)
{
  /* Enable the LED2 Clock */
  //LED2_GPIO_CLK_ENABLE();
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);

  /* Configure IO in output push-pull mode to drive external LED2 */
  LL_GPIO_SetPinMode(GPIOA, LL_GPIO_PIN_5, LL_GPIO_MODE_OUTPUT);
  /* Reset value is LL_GPIO_OUTPUT_PUSHPULL */
  LL_GPIO_SetPinOutputType(GPIOA, LL_GPIO_PIN_5, LL_GPIO_OUTPUT_PUSHPULL);
  /* Reset value is LL_GPIO_SPEED_FREQ_LOW */
  LL_GPIO_SetPinSpeed(GPIOA, LL_GPIO_PIN_5, LL_GPIO_SPEED_FREQ_LOW);
  /* Reset value is LL_GPIO_PULL_NO */
  LL_GPIO_SetPinPull(GPIOA, LL_GPIO_PIN_5, LL_GPIO_PULL_NO);
}

//--------------------------------------------------------------------------------------


/**
  * @brief  This function provides accurate delay (in milliseconds) based
  *         on SysTick counter flag
  * @note   When a RTOS is used, it is recommended to avoid using blocking delay
  *         and use rather osDelay service.
  * @note   To respect 1ms timebase, user should call @ref LL_Init1msTick function which
  *         will configure Systick to 1ms
  * @param  Delay specifies the delay time length, in milliseconds.
  * @retval None
  */
static void LL_mDelay(uint32_t Delay)
{
  __IO uint32_t  tmp = SysTick->CTRL;  /* Clear the COUNTFLAG first */
  /* Add this code to indicate that local variable is not used */
  ((void)tmp);

  /* Add a period to guaranty minimum wait */
  if (Delay < LL_MAX_DELAY)
  {
    Delay++;
  }

  while (Delay)
  {
    if ((SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk) != 0U)
    {
      Delay--;
    }
  }
}

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
