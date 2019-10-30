/**
  ******************************************************************************
  * @file    stm32f0xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
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
#include "stm32f0xx_hal.h"
#include "stm32f0xx.h"
#include "stm32f0xx_it.h"

/* USER CODE BEGIN 0 */
#include "stm32f0xx_ll_usart.h" //for LL functions

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern SPI_HandleTypeDef hspi1;
extern TIM_HandleTypeDef htim2;
extern UART_HandleTypeDef huart2;

/******************************************************************************/
/*            Cortex-M0 Processor Interruption and Exception Handlers         */ 
/******************************************************************************/

/**
* @brief This function handles Non maskable interrupt.
*/
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */
    
  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
    
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
* @brief This function handles Hard fault interrupt.
*/
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
    
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
  /* USER CODE BEGIN HardFault_IRQn 1 */
    
  /* USER CODE END HardFault_IRQn 1 */
}

/**
* @brief This function handles System service call via SWI instruction.
*/
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVC_IRQn 0 */
    
  /* USER CODE END SVC_IRQn 0 */
  /* USER CODE BEGIN SVC_IRQn 1 */
    
  /* USER CODE END SVC_IRQn 1 */
}

/**
* @brief This function handles Pendable request for system service.
*/
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */
    
  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */
    
  /* USER CODE END PendSV_IRQn 1 */
}

/**
* @brief This function handles System tick timer.
*/
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
    
  /* USER CODE END SysTick_IRQn 0 */
  //HAL_IncTick();
  HAL_SYSTICK_IRQHandler();
  /* USER CODE BEGIN SysTick_IRQn 1 */
    
  /* USER CODE END SysTick_IRQn 1 */
}

/**
  * @brief  SYSTICK callback.
  * @retval None
  */
 void HAL_SYSTICK_Callback(void)
{
    //#include "stm32f0xx_hal.h" //HAL_GetTick(void);
    
	//Tick required to support Timeout with I2C:  HAL_I2C_Mem_Read()
	HAL_IncTick();
}


/******************************************************************************/
/* STM32F0xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f0xx.s).                    */
/******************************************************************************/

/**
* @brief This function handles EXTI line 4 to 15 interrupts.
*/
void EXTI4_15_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI4_15_IRQn 0 */
    
  /* USER CODE END EXTI4_15_IRQn 0 */
    
    #ifdef HW_NUCLEO_2PORTS
      HAL_GPIO_EXTI_IRQHandler(ALERT1_Pin); //ALERT_A_Pin
  HAL_GPIO_EXTI_IRQHandler(ALERT2_Pin); //ALERT_B_Pin
  HAL_GPIO_EXTI_IRQHandler(B1_Pin); //Button B1_Pin
  
  #endif
  
  #ifdef HW_BOARD_4PORTS
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_4);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_5);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_12);
  HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_15);
  #endif
  /* USER CODE BEGIN EXTI4_15_IRQn 1 */
    
  /* USER CODE END EXTI4_15_IRQn 1 */
}

/**
* @brief This function handles TIM2 global interrupt.
*/
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
    
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */
    
  /* USER CODE END TIM2_IRQn 1 */
}

/**
* @brief This function handles SPI1 global interrupt.
*/
void SPI1_IRQHandler(void)
{
  /* USER CODE BEGIN SPI1_IRQn 0 */
    
  /* USER CODE END SPI1_IRQn 0 */
  HAL_SPI_IRQHandler(&hspi1);
  /* USER CODE BEGIN SPI1_IRQn 1 */
    
  /* USER CODE END SPI1_IRQn 1 */
}

#if 0
/**
* @brief This function handles USART2 global interrupt / USART2 wake-up interrupt through EXTI line 26.
*/
void USART2_IRQHandler(void)
{
  /* USER CODE BEGIN USART2_IRQn 0 */
    
  /* USER CODE END USART2_IRQn 0 */
  HAL_UART_IRQHandler(&huart2);
  /* USER CODE BEGIN USART2_IRQn 1 */
    
  /* USER CODE END USART2_IRQn 1 */
}
#endif

/* USER CODE BEGIN 1 */

void USART2_IRQHandler(void)
{
    /* Customize process using LL interface to improve the performance (exhaustive feature management not handled) */
    
    /* Check RXNE flag value in ISR register */
    if(LL_USART_IsActiveFlag_RXNE(USART2) && LL_USART_IsEnabledIT_RXNE(USART2))
    {
        /* RXNE flag will be cleared by reading of RDR register (done in call) */
        /* Call function in charge of handling Character reception */
        UART_CharReception_Callback();
    }
    else if(LL_USART_IsEnabledIT_ERROR(USART2) && LL_USART_IsActiveFlag_NE(USART2))
    {
        /* Call Error function */
        UART_Error_Callback();
    }
    else
    {
        UART_Error_Callback();
    }
    
    
    
}
/* USER CODE END 1 */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
