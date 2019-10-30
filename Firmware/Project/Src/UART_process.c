/**
******************************************************************************
* @file    UART/UART_HyperTerminal_TxPolling_RxIT/Src/main.c
* @author  MCD Application Team
* @brief   This sample code shows how to use UART HAL and LL APIs to transmit
*          data in polling mode while receiving data in Interrupt mode, by mixing 
*          use of LL and HAL APIs;
*          HAL driver is used to perform UART configuration, 
*          then TX transfer procedure is based on HAL APIs use (polling)
*               RX transfer procedure is based on LL APIs use (IT)
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
//#include "main.h"
#include "stm32f0xx_hal.h"
#include "stm32f0xx_ll_usart.h" //for LL functions

#include "Hardware_configuration.h" //for I2CBUS_MAX, USBPORT_MAX
#include "utils.h"
#include "STUSB4700_USBPD_functions.h"
#include "USB_PD_core.h"
#include "main_process.h"
#include "USB_PD_user.h"

#if CONSOLE_PRINTF
#include <stdio.h>
#endif

/** @addtogroup STM32F0xx_HAL_LL_MIX_Examples
* @{
*/

/** @addtogroup UART_HyperTerminal_TxPolling_RxIT
* @{
*/

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define USARTx                           USART2
#define RX_BUFFER_SIZE                   10
#define USARTx_IRQn                      USART2_IRQn

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* UART handler declaration */
//UART_HandleTypeDef UartHandle;
extern UART_HandleTypeDef huart2;
volatile uint8_t Uart_Message_Ready = 0;


/* Buffer used for transmission */
uint8_t aTxStartMessage[] = "\r\n ****UART-Hyperterminal TXRX communication (TX based on HAL polling API, RX based on IT LL API) ****\r\n Enter characters using keyboard ...\r\n";
uint8_t ubSizeToSend = sizeof(aTxStartMessage);

/* Buffer used for reception */
uint8_t aRXBufferA[RX_BUFFER_SIZE];
uint8_t aRXBufferB[RX_BUFFER_SIZE];
__IO uint32_t uwNbReceivedChars = 0;
__IO uint32_t uwBufferReadyIndication = 0;
uint8_t *pBufferReadyForUser;
uint8_t *pBufferReadyForReception;

/* Private function prototypes -----------------------------------------------*/
//static void SystemClock_Config(void);
static void Error_Handler_uart(void);

int UART_start_message(void);
int uart_process(void);
void UART_Error_Callback(void);

/* Private functions ---------------------------------------------------------*/
char BufferText[16];


extern volatile uint8_t  PB_Action_Flag[4];
extern volatile uint8_t  Timer_Action_Flag[4];

int UART_start_message(void)
{
    //uint8_t aTxStartMessage1[] = "\r\n***** USB-PD Power-Sharing firmware for STUSB4700 - [v0.3.2] ***** \r\n";
    uint8_t aTxStartMessage1[] = "\r\n***** USB-C PD Power-Sharing firmware for STUSB4700 - [v0.1] ***** \r\n";
    uint8_t ubSizeToSend1 = sizeof(aTxStartMessage1);
    
    if(HAL_UART_Transmit(&huart2, (uint8_t*)aTxStartMessage1, ubSizeToSend1, 1000)!= HAL_OK)
    {
        /* Transfer error in transmission process */
        Error_Handler_uart();
    }
    
    return 0;
}

#define LL_USED
//#define HAL_USED

int UART_Rx_init(void)
{
    /*##-2- Configure UART peripheral for reception process (using LL) ##########*/  
    /* Initializes Buffer swap mechanism :
    2 physical buffers aRXBufferA and aRXBufferB (RX_BUFFER_SIZE length)
    Any data received will be stored in active buffer : the number max of 
    data received is RX_BUFFER_SIZE */
    pBufferReadyForReception = aRXBufferA;
    pBufferReadyForUser      = aRXBufferB;
    uwNbReceivedChars = 0;
    uwBufferReadyIndication = 0;
    
#ifdef LL_USED
    /* Enable RXNE and Error interrupts */  
    LL_USART_EnableIT_RXNE(USARTx);
    LL_USART_EnableIT_ERROR(USARTx);
#endif
    
#if HAL_USED
    //__HAL_UART_ENABLE_IT(&huart2,UART_IT_RXNE);
    
#define RX_COMPLETE_SIZE 1
    /*##-4- Put UART peripheral in reception process ###########################*/  
    if(HAL_UART_Receive_IT(&huart2, (uint8_t *)aRXBufferA, RX_COMPLETE_SIZE) != HAL_OK)
    {
        Error_Handler();
    }
#endif
    
    return 0;
}

void Print_OK()
{
    uint8_t pData[] = "Cmd-OK\r\n";
    uint8_t ubSizeToSend = sizeof(pData);
    
    if(HAL_UART_Transmit(&huart2, (uint8_t*)pData, ubSizeToSend, 1000)!= HAL_OK)
    {
        /* Transfer error in transmission process */
        Error_Handler_uart();
    }
}

void UART_CheckReceivedMessage()
{
    if(Uart_Message_Ready == 1)
    {
        int Action_PrintOK = 0;
        int UsbPort = 0;
        
        Uart_Message_Ready = 0; //clear
        
        uint8_t Message[RX_BUFFER_SIZE];
        
        //memcpy(Message, pBufferReadyForReception, uwNbReceivedChars);
        for(int i=0; i< uwNbReceivedChars && i< RX_BUFFER_SIZE; i++)
        {
            Message[i] = pBufferReadyForReception[i];
        }
        
        uwNbReceivedChars = 0; //clear buffer
        
        
        int idx = 0;
        
        
        if(Message[idx] == '1') //to simulate a push button
        {
            Print_OK();
            Action_PrintOK = 1;
            
            UsbPort = 0;
            PB_Action_Flag[UsbPort] = 1;
        }
        else if(Message[idx] == '2') //to simulate a push button
        {
            Print_OK();
            Action_PrintOK = 1;
            
            UsbPort = 1;
            if(UsbPort < USBPORT_MAX)
                PB_Action_Flag[UsbPort] = 1;
        }
        else if(Message[idx] == '3') //to simulate a push button
        {
            Print_OK();
            Action_PrintOK = 1;
            
            UsbPort = 2;
            if(UsbPort < USBPORT_MAX)
                PB_Action_Flag[UsbPort] = 1;
        }
        else if(Message[idx] == '4') //to simulate a push button
        {
            Print_OK();
            Action_PrintOK = 1;
            
            UsbPort = 3;
            if(UsbPort < USBPORT_MAX)
                PB_Action_Flag[UsbPort] = 1;
        }
        
        if(Message[idx] == 't') //to simulate a timer action
        {
            UsbPort = -1;
            
            if(Message[idx+1] == '1')
            {
                UsbPort = 0;
            }
            else if(Message[idx+1] == '2')
            {
                UsbPort = 1;
            }
            else if(Message[idx+1] == '3')
            {
                UsbPort = 2;
            }
            else if(Message[idx+1] == '4')
            {
                UsbPort = 3;
            }
            
            if( (UsbPort >= 0) && (UsbPort < USBPORT_MAX) )
            {
                Print_OK();
                Action_PrintOK = 1;
                
                Timer_Action_Flag[UsbPort] = 1;
            }
        }
        
        //terminal commands: r, r1, r2, r3, r4
        
        if(Message[idx] == 'r') //reset port
        {
            UsbPort = -1;
            
            if(Message[idx+1] == NULL)
            {
                Print_OK();
                Action_PrintOK = 1;
                
                for(int i = 0; (i < GetDeviceFoundCount() ) && (i < USBPORT_MAX) ; i++)
                {
                    STUSB4700_SEND_SW_Reset(i);
                }
            }
            
            if(Message[idx+1] == '1')
            {
                UsbPort = 0;
            }
            else if(Message[idx+1] == '2')
            {
                UsbPort = 1;
            }
            else if(Message[idx+1] == '3')
            {
                UsbPort = 2;
            }
            else if(Message[idx+1] == '4')
            {
                UsbPort = 3;
            }
            
            if( (UsbPort >= 0) && (UsbPort < USBPORT_MAX) )
            {
                Print_OK();
                Action_PrintOK = 1;
                
                STUSB4700_SEND_SW_Reset(UsbPort);
            }
        }
        
        //terminal commands: g, g1, g2, g3, g4
        
        if(Message[idx] == 'g') //GetSinkCap
        {
            UsbPort = -1;
            
            if(Message[idx+1] == NULL)
            {
                Print_OK();
                Action_PrintOK = 1;
                
                for(int i = 0; (i < GetDeviceFoundCount() ) && (i < USBPORT_MAX) ; i++)
                {
                    push_button_Get_Sink_Cap(i);
                }
            }
            else
            {
                if(Message[idx+1] == '1')
                {
                    UsbPort = 0;
                }
                else if(Message[idx+1] == '2')
                {
                    UsbPort = 1;
                }
                else if(Message[idx+1] == '3')
                {
                    UsbPort = 2;
                }
                else if(Message[idx+1] == '4')
                {
                    UsbPort = 3;
                }
                
                if( (UsbPort >= 0) && (UsbPort < USBPORT_MAX) )
                {
                    Print_OK();
                    Action_PrintOK = 1;
                    
                    push_button_Get_Sink_Cap(UsbPort);
                }
            }
        }
        
        //terminal commands: p
        
        if(Message[idx] == 'p') //print
        {
            Print_OK();
            Action_PrintOK = 1;
            
            
            //Print_PolicyEngine_StateMachineFSM();
            
            //for debug
            int Contract_nb = 0;
            int Pdo_nb[USBPORT_MAX];
            extern float PowerAllocated[USBPORT_MAX];
            
            
            printf("\r\n---- Checking all contracts:");
            printf("\r\n");
            for(int UsbPort = 0; (UsbPort < USBPORT_MAX) && (UsbPort < GetDeviceFoundCount() ); UsbPort++)
            {
                Contract_nb = Get_PD_Contract(UsbPort, &PowerAllocated[UsbPort]);
                
                Pdo_nb[UsbPort] = Contract_nb - 1;;
            }
            
            printf("\r\n---- Checking all PDOs status:");
            for(int UsbPort = 0; (UsbPort < USBPORT_MAX) && (UsbPort < GetDeviceFoundCount()) ; UsbPort++)
            {
                Print_PDO(UsbPort);
                
                
                if(Pdo_nb[UsbPort] == -2)
                {
                    printf("                             | ");
                    printf("Not-connected\r\n");
                }
                if(Pdo_nb[UsbPort] == -1)
                {
                    printf("                             | ");
                    printf("Type-C selected\r\n");
                }
                else if(Pdo_nb[UsbPort] >= 0)
                {
                    printf("                             | ");
                    printf("PDO%d selected\r\n", Pdo_nb[UsbPort] );
                }
            }
            
            {
                extern int TotalConnectionCount;
                float TotalSystemPower = TOTAL_SYSTEM_POWER;
                float TotalPowerAllocated = 0;
                for(int i = 0; i < USBPORT_MAX; i++)
                {
                    TotalPowerAllocated += PowerAllocated[i];
                }
                int Power_MAX_remaining = (int)(TotalSystemPower - TotalPowerAllocated);
                
                int Power_MAX_remaining_PerPDO = 0;
                if(Power_MAX_remaining > MAX_POWER_PER_USBPORT) Power_MAX_remaining_PerPDO = MAX_POWER_PER_USBPORT;
                else Power_MAX_remaining_PerPDO = Power_MAX_remaining;
                
                printf("\r\n");
                
                printf("                             | ");
                printf("UsbPortAttached: %d/%d \r\n", TotalConnectionCount, USBPORT_MAX );
                
                printf("                             | ");
                printf("TotalSystemPower: %d W \r\n", TOTAL_SYSTEM_POWER );
                printf("                             | ");
                printf("TotalPowerAllocated: %f W \r\n", TotalPowerAllocated );
            }
            
        }
        
        //terminal commands: s
        
        if(Message[idx] == 's') //print status
        {
            Print_OK();
            Action_PrintOK = 1;
            
            //for debug
            for(int UsbPort = 0; UsbPort < USBPORT_MAX; UsbPort++)
            {
                PrintAlertStatusRegister(UsbPort);
                Print_PolicyEngine_StateMachineFSM(UsbPort);
            }
        }
        
        if(Action_PrintOK == 1)
        {
            //Print_OK();
        }
        else
        {
            uint8_t pData[] = "??\r\n";
            uint8_t ubSizeToSend = sizeof(pData);
            
            if(HAL_UART_Transmit(&huart2, (uint8_t*)pData, ubSizeToSend, 1000)!= HAL_OK)
            {
                /* Transfer error in transmission process */
                Error_Handler_uart();
            }
        }
        
    }
    
}

/**
* @brief  Main program
* @retval None
*/
int uart_process(void)
{
    int i = 0;
    
    /* STM32F0xx HAL library initialization:
    - Configure the Flash prefetch
    - Systick timer is configured by default as source of time base, but user 
    can eventually implement his proper time base source (a general purpose 
    timer for example or other time source), keeping in mind that Time base 
    duration should be kept 1ms since PPP_TIMEOUT_VALUEs are defined and 
    handled in milliseconds basis.
    - Low Level Initialization
    */
    //HAL_Init();
    
    /* Configure the system clock to 48 MHz */
    //SystemClock_Config();
    
    /* Configure leds */
    //BSP_LED_Init(LED2);
    
    /*##-1- Configure the UART peripheral using HAL services ###################*/
    /* Put the USART peripheral in the Asynchronous mode (UART Mode) */
    /* UART configured as follows:
    - Word Length = 8 Bits (7 data bit + 1 parity bit) : 
    BE CAREFUL : Program 7 data bits + 1 parity bit in PC HyperTerminal
    - Stop Bit    = One Stop bit
    - Parity      = ODD parity
    - BaudRate    = 9600 baud
    - Hardware flow control disabled (RTS and CTS signals) */
    //  UartHandle.Instance        = USARTx;
    //
    //  UartHandle.Init.BaudRate   = 9600;
    //  UartHandle.Init.WordLength = UART_WORDLENGTH_8B;
    //  UartHandle.Init.StopBits   = UART_STOPBITS_1;
    //  UartHandle.Init.Parity     = UART_PARITY_ODD;
    //  UartHandle.Init.HwFlowCtl  = UART_HWCONTROL_NONE;
    //  UartHandle.Init.Mode       = UART_MODE_TX_RX;
    
    //  if(HAL_UART_Init(&UartHandle) != HAL_OK)
    //  {
    //    /* Initialization Error */
    //    Error_Handler_uart();
    //  }
    
    /*##-2- Configure UART peripheral for reception process (using LL) ##########*/  
    /* Initializes Buffer swap mechanism :
    2 physical buffers aRXBufferA and aRXBufferB (RX_BUFFER_SIZE length)
    Any data received will be stored in active buffer : the number max of 
    data received is RX_BUFFER_SIZE */
    pBufferReadyForReception = aRXBufferA;
    pBufferReadyForUser      = aRXBufferB;
    uwNbReceivedChars = 0;
    uwBufferReadyIndication = 0;
    /* Enable RXNE and Error interrupts */  
    //  LL_USART_EnableIT_RXNE(USARTx);
    //  LL_USART_EnableIT_ERROR(USARTx);
    
    /*##-3- Start the transmission process (using HAL Polling mode) #############*/  
    /* In main loop, Tx buffer is sent every 0.5 sec. 
    As soon as RX buffer is detected as full, received bytes are echoed on TX line to PC com port */
    
    
    if(HAL_UART_Transmit(&huart2, (uint8_t*)aTxStartMessage, ubSizeToSend, 1000)!= HAL_OK)
    {
        /* Transfer error in transmission process */
        Error_Handler_uart();
    }
    
    
    /* Infinite loop */
    while (1)
    {
#if 0
        /* USART IRQ handler is not anymore routed to HAL_UART_IRQHandler() function 
        and is now based on LL API functions use. 
        Therefore, use of HAL IT based services is no more possible : use TX HAL polling services */
        if(HAL_UART_Transmit(&huart2, (uint8_t*)aTxStartMessage, ubSizeToSend, 1000)!= HAL_OK)
        {
            /* Transfer error in transmission process */
            Error_Handler_uart();
        }
#endif
        
#if CONSOLE_PRINTF
        printf("Hi %d \t", i++);
        //sprintf(BufferText, "Hello USB-C");
#endif
        
#if 0
        /* Checks if Buffer full indication has been set */
        if (uwBufferReadyIndication != 0)
        {
            /* Reset indication */
            uwBufferReadyIndication = 0;
            
            /* USART IRQ handler is not anymore routed to HAL_UART_IRQHandler() function 
            and is now based on LL API functions use. 
            Therefore, use of HAL IT based services is no more possible : use TX HAL polling services */
            if(HAL_UART_Transmit(&huart2, (uint8_t*)pBufferReadyForUser, RX_BUFFER_SIZE, 1000)!= HAL_OK)
            {
                /* Transfer error in transmission process */
                Error_Handler_uart();
            }
            
            /* Toggle LED2 */
            //BSP_LED_Toggle(LED2);
        }
#endif
        
        /* Manage temporisation between TX buffer sendings */
        //HAL_Delay(500);
        HAL_Delay(1000);
    }
}


/**
* @brief  This function is executed in case of error occurrence.
* @retval None
*/
static void Error_Handler_uart(void)
{
    /* Turn LED2 to on for error */
    //BSP_LED_On(LED2); 
    while(1)
    {
    }
}

/**
* @brief  Rx Transfer completed callback
* @note   This example shows a simple way to report end of IT Rx transfer, and 
*         you can add your own implementation.
* @retval None
*/
void UART_CharReception_Callback(void)
{
    uint8_t DataRx8;
    
    /* Read Received character. RXNE flag is cleared by reading of RDR register */
    DataRx8 = LL_USART_ReceiveData8(USARTx);
    
    if(DataRx8 == '\r') //\r\n
    {
        if (uwNbReceivedChars < RX_BUFFER_SIZE)
            pBufferReadyForReception[uwNbReceivedChars++] = '\0';
        
        Uart_Message_Ready = 1;
        
        //uwNbReceivedChars = 0; //clear buffer
    }
    else if(DataRx8 == '\n') //\r\n
    {
        // do nothing, do not store
    }
    else if (uwNbReceivedChars < RX_BUFFER_SIZE)
    {
        pBufferReadyForReception[uwNbReceivedChars++] = DataRx8;
    }
}

/**
* @brief  UART error callbacks
* @note   This example shows a simple way to report transfer error, and you can
*         add your own implementation.
* @retval None
*/
void UART_Error_Callback(void)
{
    __IO uint32_t isr_reg;
    
    /* Disable USARTx_IRQn */
    NVIC_DisableIRQ(USARTx_IRQn);
    
    /* Error handling example :
    - Read USART ISR register to identify flag that leads to IT raising
    - Perform corresponding error handling treatment according to flag
    */
    isr_reg = LL_USART_ReadReg(USARTx, ISR);
    if (isr_reg & LL_USART_ISR_NE)
    {
        /* Turn LED2 on: Transfer error in reception/transmission process */
        //BSP_LED_On(LED2);
        
        /* case Noise Error flag is raised : Clear NF Flag */
        LL_USART_ClearFlag_NE(USARTx);
    }
    else if ((isr_reg & LL_USART_ISR_ORE) != 0) //USART-Overrun management
    {
        volatile uint8_t dataRx;
        static __IO uint32_t UartOverrunError = 0;
        
        
        UartOverrunError = 1;
        
        /*!< Overrun error flag */
        LL_USART_ClearFlag_ORE(USARTx);
        
        
        if (UartOverrunError == 1)
        {
            UartOverrunError = 0; //clear
            
            //PrintInfo((uint8_t*) "ERR_OVERRUN\r\n", 13);
            
            uint8_t aTxStartMessage1[] = "ERR_OVERRUN\r\n";
            uint8_t ubSizeToSend1 = sizeof(aTxStartMessage1);
            
            if(HAL_UART_Transmit(&huart2, (uint8_t*)aTxStartMessage1, ubSizeToSend1, 1000)!= HAL_OK)
            {
                /* Transfer error in transmission process */
                Error_Handler_uart();
            }
            
            //            {
            //                /* Transfer error in transmission process */
            //                Error_Handler_uart();
            //            }
            
            //reset the buffer:
            uwNbReceivedChars = 0;
            pBufferReadyForReception[uwNbReceivedChars] = 0xFF;
            
        }
        
        if (LL_USART_IsActiveFlag_RXNE(USARTx) != 0)
        {
            //read USARTx->RDR register
            dataRx = LL_USART_ReceiveData8(USARTx);  //to clear error
        }
        
        NVIC_EnableIRQ(USARTx_IRQn); //re-enable the IRQ  //added GG to management consecutive Overrun
    }
    else
    {
        /* Turn LED2 on: Transfer error in reception/transmission process */
        //BSP_LED_On(LED2);
    }
}

#ifdef  USE_FULL_ASSERT
/**
* @brief  Reports the name of the source file and the source line number
*         where the assert_param error has occurred.
* @param  file: pointer to the source file name
* @param  line: assert_param error line source number
* @retval None
*/
void assert_failed(char* file, uint32_t line)
{ 
    /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
    
    /* Infinite loop */
    while (1)
    {
    }
}
#endif

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
