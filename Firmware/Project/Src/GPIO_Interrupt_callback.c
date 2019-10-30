#include "main.h" //for ALERT1_Pin, ALERT2_Pin
#include "stm32f0xx_hal.h" //stm32f0xx_hal_gpio.h //for GPIO_PIN_4, GPIO_PIN_5, GPIO_PIN_12, GPIO_PIN_15
#include "Hardware_configuration.h" //for I2CBUS_MAX, USBPORT_MAX
#include "global_var_IRQ.h"


/**
* @brief  EXTI line detection callback.
* @param  GPIO_Pin Specifies the port pin connected to corresponding EXTI line.
* @retval None
*/
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    int _UsbPort;
    
    switch(GPIO_Pin)
    {
    case ALERT1_Pin:
        _UsbPort = 0;
        USB_PD_Interupt_Flag[_UsbPort]++; //set Interrupt flag
        break;
        
#if (USBPORT_MAX >=2) && (defined ALERT2_Pin) 
    case ALERT2_Pin:
        _UsbPort = 1;
        USB_PD_Interupt_Flag[_UsbPort]++; //set Interrupt flag
        break;
#endif
        
#if (USBPORT_MAX >=3) && (defined ALERT3_Pin) 
    case ALERT3_Pin:
        _UsbPort = 2;
        USB_PD_Interupt_Flag[_UsbPort]++; //set Interrupt flag
        break;
#endif
        
#if (USBPORT_MAX >=4)  && (defined ALERT4_Pin) 
    case ALERT4_Pin:
        _UsbPort = 3;
        USB_PD_Interupt_Flag[_UsbPort]++; //set Interrupt flag
        break;
#endif
    }
}
