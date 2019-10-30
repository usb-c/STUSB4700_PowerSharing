/* Includes ------------------------------------------------------------------*/
#include "main_process.h"
#include "stm32f0xx_hal.h" //for uint16_t



// Example from:   STM32Cube_FW_F0_V1.9.0\Projects\STM32F072RB-Nucleo\Examples\TIM\TIM_TimeBase\Src\main.c

//Example:
//SystemCoreClock = 8 MHz;
// --> PrescalerValue = 8 MHz / 800 = 10000 Hz -> 0.1ms

//Example:
//HCLK = 48MHz, APB1 = 48MHz,
// --> PrescalerValue = 48 MHz / 48000 = 1000 Hz -> 1ms


void Timer_Start(void)
{
    extern TIM_HandleTypeDef htim2;
    
    //    /* TIM handle declaration */
    //TIM_HandleTypeDef    TimHandle;
    
    //    /* Prescaler declaration */
    //uint32_t uwPrescalerValue = 0;
    
    /*##-1- Configure the TIM peripheral #######################################*/
    /* -----------------------------------------------------------------------
    In this example TIM2 input clock (TIM2CLK)  is set to APB1 clock (PCLK1),
    since APB1 prescaler is equal to 1.
    TIM2CLK = PCLK1
    PCLK1 = HCLK
    => TIM2CLK = HCLK = SystemCoreClock
    To get TIM2 counter clock at 10 KHz, the Prescaler is computed as following:
    Prescaler = (TIM2CLK / TIM2 counter clock) - 1
    Prescaler = (SystemCoreClock /10 KHz) - 1
    
    Note:
    SystemCoreClock variable holds HCLK frequency and is defined in system_stm32f0xx.c file.
    Each time the core clock (HCLK) changes, user had to update SystemCoreClock
    variable value. Otherwise, any configuration based on this variable will be incorrect.
    This variable is updated in three ways:
    1) by calling CMSIS function SystemCoreClockUpdate()
    2) by calling HAL API function HAL_RCC_GetSysClockFreq()
    3) each time HAL_RCC_ClockConfig() is called to configure the system clock frequency
    ----------------------------------------------------------------------- */
    
    //  /* Compute the prescaler value to have TIMx counter clock equal to 10000 Hz */
    //  uwPrescalerValue = (uint32_t)(SystemCoreClock / 10000) - 1;
    
    //  /* Set TIMx instance */
    //  TimHandle.Instance = TIM2;
    //
    //  /* Initialize TIMx peripheral as follows:
    //       + Period = 10000 - 1
    //       + Prescaler = (SystemCoreClock/10000) - 1
    //       + ClockDivision = 0
    //       + Counter direction = Up
    //  */
    //  TimHandle.Init.Period            = 10000 - 1;
    //  TimHandle.Init.Prescaler         = uwPrescalerValue;
    //  TimHandle.Init.ClockDivision     = 0;
    //  TimHandle.Init.CounterMode       = TIM_COUNTERMODE_UP;
    //  TimHandle.Init.RepetitionCounter = 0;
    //  TimHandle.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
    //  if (HAL_TIM_Base_Init(&TimHandle) != HAL_OK)
    //  {
    //    /* Initialization Error */
    //    Error_Handler();
    //  }
    
    /*##-2- Start the TIM Base generation in interrupt mode ####################*/
    /* Start Channel1 */
    if (HAL_TIM_Base_Start_IT(&htim2) != HAL_OK)
    {
        /* Starting Error */
        Error_Handler();
    }
    
}

/**
* @brief  Period elapsed callback in non blocking mode 
* @param  htim TIM handle
* @retval None
*/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    static uint32_t Timer2Counter = 0;
    
    if (htim->Instance == TIM2) //htim2.Init.Period = 99;
    {
        Timer2Counter++;
    }
    
    if (Timer2Counter >= 20) // every (20 * 100ms = 2000ms)
    {
        int UsbPort = 0;
#if 0
        Timer_Action_Flag[UsbPort] = 1;
#endif
        Timer2Counter = 0;
        
#ifdef CONSOLE_PRINTF
        //printf("TIM2 ");
#endif    
    }
}


int HAL_Delay_nonblocking(uint32_t Delay)
{
    static uint32_t tickstart = 0;
    uint32_t wait = Delay;
    
    if(tickstart == 0) //init once
        tickstart = HAL_GetTick();
    
    /* Add a period to guarantee minimum wait */
    if (wait < HAL_MAX_DELAY)
    {
        wait++;
    }
    
    //while((HAL_GetTick() - tickstart) < wait) {}
    if((HAL_GetTick() - tickstart) >= wait)
    {
        tickstart = 0; //clear
        return 1; //OK
    }
    else
    {
        return 0; //counter ongoing
    }
}

int HAL_Delay_nonblocking2(uint32_t Delay)
{
    static uint32_t tickstart = 0;
    uint32_t wait = Delay;
    
    if(tickstart == 0) //init once
        tickstart = HAL_GetTick();
    
    /* Add a period to guarantee minimum wait */
    if (wait < HAL_MAX_DELAY)
    {
        wait++;
    }
    
    //while((HAL_GetTick() - tickstart) < wait) {}
    if((HAL_GetTick() - tickstart) >= wait)
    {
        tickstart = 0; //clear
        return 1; //OK
    }
    else
    {
        return 0; //counter ongoing
    }
}

int HAL_Delay_nonblocking3(uint32_t Delay)
{
    static uint32_t tickstart = 0;
    uint32_t wait = Delay;
    
    if(tickstart == 0) //init once
        tickstart = HAL_GetTick();
    
    /* Add a period to guarantee minimum wait */
    if (wait < HAL_MAX_DELAY)
    {
        wait++;
    }
    
    //while((HAL_GetTick() - tickstart) < wait) {}
    if((HAL_GetTick() - tickstart) >= wait)
    {
        tickstart = 0; //clear
        return 1; //OK
    }
    else
    {
        return 0; //counter ongoing
    }
}
