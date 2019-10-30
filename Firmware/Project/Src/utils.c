
#include "Hardware_configuration.h" //for I2CBUS_MAX, USBPORT_MAX
#include "I2C_ReadWrite.h" //for I2cRead

#include "USB_PD_defines_stusb4700.h"
#include "stusb4700_defines_2.h"

#include "USB_PD_user.h"
#include "USB_PD_core.h"

#include "printf_debug.h"

extern USB_PD_I2C_PORT STUSB47DeviceConf[USBPORT_MAX];

int PrintAlertStatusRegister(int UsbPort)
{
    //for(int UsbPort = 0; UsbPort < USBPORT_MAX; UsbPort++)
    {
        uint16_t Address; 
        uint8_t DataR;
        uint8_t MaskR;
        int Status;
        
        Address = STUSBADDR_ALERT_STATUS_1;
        Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, Address, &DataR, 1 ); if(Status != I2C_OK) return I2C_ERR;
        
        Address = STUSBADDR_ALERT_STATUS_MASK;
        Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, Address, &MaskR, 1 ); if(Status != I2C_OK) return I2C_ERR;
        
        
        printf("UsbPort-%d, ", UsbPort);
        printf("STUSB47 AlertStatus-Reg: 0x%X, ", DataR);
        printf("Useful: 0x%X\r\n", DataR & (~MaskR) );
        
        if(DataR != 0)
        {
            printf("\t ");
            
            //bit-0 //Physical layer alert
            if ((DataR & VALUE_PHY_STATUS_AL) != 0)
            {
                printf("bit-0 PHY_STATUS, ");
            }
            
            //bit-1 //Protocol layer alert
            if ((DataR & VALUE_PRT_STATUS_AL) != 0)
            {
                printf("bit-1 PRT_STATUS, ");
            }
            
            //bit-2 //Policy Engine layer alert (Reserved)
            if ((DataR & VALUE_PE_STATUS_AL) != 0)
            {
                printf("bit-2 PE_STATUS, ");
            }
            
            //bit-3 //DPM layer alert
            if ((DataR & VALUE_DPM_STATUS_AL) != 0)
            {
                printf("bit-3 DPM_STATUS, ");
            }
            
            
            //bit-4 //Change occurred on HW_FAULT_STATUS_TRANS register (Hardware fault from analog logic)
            if ((DataR & VALUE_CC_HW_FAULT_STATUS_AL) != 0)
            {
                printf("bit-4 C_HW_FAULT_STATUS, ");
            }
            
            //bit-5 //Change occurred on MONITORING_STATUS_TRANS register (see POWER_STATUS register).
            if ((DataR & VALUE_TYPEC_MONITORING_STATUS_AL) != 0)
            {
                printf("bit-5 TYPEC_MONITORING_STATUS, ");
                
                //reading register TYPEC_MONITORING_STATUS_0 (0x0D) clear the bit TYPEC_MONITORING_STATUS_AL of register ALERT_STATUS_1
                //Status = I2C_Read_USB_PD(Devices[Port].Bus, Devices[Port].I2cDeviceID_7bit, TYPEC_MONITORING_STATUS_0, &DataRW[0], 2);
                //PD_status[Port].Monitoring = ((DataRW[1] & 0x0F) << 8) | (PD_status[Port].Monitoring & 0x000F) | (DataRW[0] & 0x0F);
                //PD_status[Port].TypeC = (DataRW[0] & 0xF0) >> 4;
            }
            
            //bit-6 //Change occurred on PORT_STATUS_TRANS register  (see CC_STATUS register).
            if ((DataR & VALUE_PORT_STATUS_AL) != 0)
            {
                printf("bit-6 PORT_STATUS, ");
                
                //reading register PORT_STATUS_TRANS (0x0D) clear the bit PORT_STATUS_AL of register ALERT_STATUS_1
                //Status = I2C_Read_USB_PD(Devices[Port].Bus, Devices[Port].I2cDeviceID_7bit, PORT_STATUS_TRANS, &DataRW[0], 2);
                //PD_status[Port].Port_status = (DataRW[1] << 8) | DataRW[0];
                
            }
            
            //bit-7 //Hard Reset Message received
            if ((DataR & VALUE_HARD_RESET_AL) != 0)
            {
                printf("bit-7 HARD_RESET, ");
            }
            
            printf("\r\n");
        }
    }
    
    return 0;
}

int Print_PolicyEngine_StateMachineFSM(int UsbPort)
{
    uint8_t Data;
    uint16_t Address;
    int Status;
    extern USB_PD_I2C_PORT STUSB47DeviceConf[USBPORT_MAX];
    
    //for(int UsbPort = 0; UsbPort < USBPORT_MAX; UsbPort++)
    {
        Address = STUSBADDR_PE_FSM; 
        Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, Address, &Data, 1 ); if(Status != I2C_OK) return I2C_ERR;
        
        printf("\t");
        printf("PolicyEngine_FSM: 0x%X, ", Data);
    }
    
    switch(Data)
    {
    case PE_INIT:
        printf("(PE_INIT)");
        break;
        
    case PE_SOFT_RESET:
        printf("(PE_SOFT_RESET)");
        break;
        
    case PE_SRC_DISCOVERY:
        printf("(PE_SRC_DISCOVERY)");
        break;
        
    case PE_SRC_TRANSITION_SUPPLY_2:
        printf("(PE_SRC_TRANSITION_SUPPLY_2)");
        break;
        
    case PE_SRC_DISABLED:
        printf("(PE_SRC_DISABLED)");
        break;
        
    case PE_SRC_READY:
        printf("(PE_SRC_READY)");
        break;
        
    case PE_VCS_DFP_SEND_PS_RDY:
        printf("(PE_VCS_DFP_SEND_PS_RDY)");
        break;
        
    }
    
    printf("\r\n");
    
    return 0;
}
