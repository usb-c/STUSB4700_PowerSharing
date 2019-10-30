/**
******************************************************************************
* File Name          : USB_PD_user.h
* Date               : 03/09/2015 10:51:46
* Description        : This file contains all the functions prototypes for 
*                      the USB_PD_user  
******************************************************************************
*
* COPYRIGHT(c) 2015 STMicroelectronics
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __USB_PD_USER_H
#define __USB_PD_USER_H
#ifdef __cplusplus
extern "C" {
#endif
    
    /* Includes ------------------------------------------------------------------*/

#include "USB_PD_core.h"
    
    
    void Dual_TypeC_power_sharing(int UsbPort);
    void Single_TypeC_port(uint8_t UsbPort);
    int push_button_Get_Sink_Cap(uint8_t UsbPort);
    void push_button_TypeA_emulation(uint8_t UsbPort); 
    void push_button_circular(uint8_t UsbPort); 
    void Timer_circular_ChangePdoNb(uint8_t UsbPort); 
    
    
#ifdef __cplusplus
}
#endif

#endif /*usbpd user header */

/**
* @}
*/

/**
* @}
*/

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
