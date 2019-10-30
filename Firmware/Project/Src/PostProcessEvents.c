#include "USB_PD_core.h"
#include "i2c.h"
#include "USBPD_spec_defines.h"
#include "PostProcessEvents.h"
#include "Hardware_configuration.h" //for I2CBUS_MAX, USBPORT_MAX

#include "USB_PD_defines_stusb4700.h"
#include "stusb4700_defines_2.h"


volatile int PostProcess_IrqReceived[USBPORT_MAX] = {0};
volatile int PostProcess_AttachTransition[USBPORT_MAX] = {0};
volatile int PostProcess_IrqHardreset = 0;
volatile int PostProcess_PD_MessageReceived[USBPORT_MAX] = {0};
volatile int PostProcess_SRC_PDO_Received = 0;
volatile int PostProcess_PSRDY_Received = 0;
volatile int PostProcess_Msg_Accept = 0;
volatile int PostProcess_Msg_Reject = 0;
volatile int PostProcess_Msg_GoodCRC = 0;




extern USB_PD_StatusTypeDef PD_status[USBPORT_MAX] ;
extern USB_PD_I2C_PORT STUSB47DeviceConf[USBPORT_MAX];


static char MsgBuffer[32];
static int idx_head = 0;
static int idx_tail = 0;
//"C" = CtrlMsg
//"D" = DataMsg


int Push_PD_MessageReceived(int UsbPort, char MessageType, char MessageValue)
{
    MsgBuffer[idx_head] = MessageType;
    idx_head++;
    
    MsgBuffer[idx_head] = MessageValue;
    idx_head++;
    
    if(idx_tail == idx_head)
    {
        //error: buffer overflow
        return -1;
    }
    if(idx_head > 31) idx_head = 0;
    
    PostProcess_PD_MessageReceived[UsbPort]++;
    
    return 0;
}

int Push_PD_MessageReceived1(int UsbPort, char MessageType)
{
    MsgBuffer[idx_head] = MessageType;
    idx_head++;
    
    if(idx_tail == idx_head)
    {
        //error: buffer overflow
        return -1;
    }
    if(idx_head > 31) idx_head = 0;
    
    PostProcess_PD_MessageReceived[UsbPort]++;
    
    return 0;
}

int Pop_PD_MessageReceived(int UsbPort)
{
    char MessageType;
    
    if(idx_tail == idx_head)
    {
        //error: buffer empty
        return -1; //no data available
    }
    
    MessageType = MsgBuffer[idx_tail];
    MsgBuffer[idx_tail] = 0; //clear
    
    idx_tail++;
    if(idx_tail > 31) idx_tail = 0;
    
    return MessageType;
}

static char IrqBuffer[32];
static int irq_idx_head = 0;
static int irq_idx_tail = 0;

int Push_IrqReceived(int UsbPort, char MessageType)
{
    PostProcess_IrqReceived[UsbPort]++;
    
    IrqBuffer[irq_idx_head] = MessageType;
    
    irq_idx_head++;
    
    if(irq_idx_tail == irq_idx_head)
    {
        //error: buffer overflow
        return -1;
    }
    if(irq_idx_head > (32-1)) irq_idx_head = 0;
    
    return 0;
}

int Pop_IrqReceived(int UsbPort)
{
    char MessageType;
    
    if(irq_idx_tail == irq_idx_head)
    {
        //error: buffer empty
        return -1; //no data available
    }
    
    MessageType = IrqBuffer[irq_idx_tail];
    IrqBuffer[irq_idx_tail] = 0; //clear
    
    irq_idx_tail++;
    if(irq_idx_tail > (32-1)) irq_idx_tail = 0;
    
    return MessageType;
}



int PostProcess_UsbEvents(uint8_t UsbPort)
{
    //int UsbPort = 0;
    int Status;
    unsigned int Address;
    
    if( PostProcess_IrqReceived[UsbPort] != 0)
    {
        int IrqStatus;
        
        //printf("\r\n====> IRQ from UsbPort-%i ", UsbPort);
        printf("usb%i-", UsbPort);
        
        while( PostProcess_IrqReceived[UsbPort] != 0)
        {
            //__disable_irq(); //CMSIS
            PostProcess_IrqReceived[UsbPort]--;
            //__enable_irq();
            
#if DEBUG_PRINTF
            IrqStatus = Pop_IrqReceived(UsbPort);
            printf("irq_%02X ", IrqStatus);
#endif
        }
    }
    
    
    if( PostProcess_AttachTransition[UsbPort] != 0)
    {
        //__disable_irq(); //CMSIS
        PostProcess_AttachTransition[UsbPort]--;
        //__enable_irq();
        
        //printf("Trans. occurred on ATTACH_STATUS bit: ");
        printf("\r\n=== CABLE: ");

        if( (PD_status[UsbPort].Port_status.d8 & STUSBMASK_ATTACHED_STATUS) == VALUE_ATTACHED)
        {
            //printf("Attached ");
            
            uint8_t Data;
            Address = STUSBADDR_TYPEC_STATUS; //[Read]
            Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, Address , &Data, 1 );
            if(( Data & MASK_REVERSE) == 0)
            {
                printf("Attached [CC1] ");
            }
            else
            {
                printf("Attached [CC2] ");
            }
        }
        else
        {
            printf("Not-attached ");
        }
        
        printf("\r\n");
    }
    
    
    if( PostProcess_PD_MessageReceived[UsbPort] != 0)
    {
        int MessageType;
        
        //__disable_irq(); //CMSIS
        PostProcess_PD_MessageReceived[UsbPort]--;
        //__enable_irq();
        
        while( (MessageType = Pop_PD_MessageReceived(UsbPort) ) != -1)
        {
#if DEBUG_PRINTF
            if(MessageType == 'C')
            {
                printf("CtrlMsg");
                int HeaderMessageType = Pop_PD_MessageReceived(UsbPort);
                printf("%02X", HeaderMessageType);
                
                switch (HeaderMessageType )
                {   
                case USBPD_CTRLMSG_Reserved1:
                    break;
                    
                case USBPD_CTRLMSG_GoodCRC:
                    printf("(GoodCRC)");
                    break;
                    
                case USBPD_CTRLMSG_Accept:
                    printf("(Accept)");
                    break;
                    
                case USBPD_CTRLMSG_Reject:
                    printf("(Reject)");
                    break;
                    
                case USBPD_CTRLMSG_PS_RDY:
                    printf("(PS_RDY)");
                    break;
                    
                case USBPD_CTRLMSG_Get_Source_Cap:
                    break;
                    
                case USBPD_CTRLMSG_Get_Sink_Cap:
                    break;
                    
                case USBPD_CTRLMSG_Wait:
                    break;
                    
                case USBPD_CTRLMSG_Soft_Reset:
                    break;
                    
                case USBPD_CTRLMSG_Not_Supported:
                    break;
                    
                case USBPD_CTRLMSG_Get_Source_Cap_Extended:
                    break;
                    
                case USBPD_CTRLMSG_Get_Status:
                    break;
                    
                case USBPD_CTRLMSG_FR_Swap:
                    break;
                    
                case USBPD_CTRLMSG_Get_PPS_Status:
                    break;
                    
                case USBPD_CTRLMSG_Get_Country_Codes:
                    break;
                    
                default:
                    break;
                }
                
                printf(" ");
            }
            else if(MessageType == 'D')
            {
                printf("DataMsg");
                int HeaderMessageType = Pop_PD_MessageReceived(UsbPort);
                printf("%02X", HeaderMessageType);
                
                switch ( HeaderMessageType )
                {
                case USBPD_DATAMSG_Source_Capabilities :
                    printf("(SourceCap)");
                    break;
                case USBPD_DATAMSG_Request:  /* get request message */
                    printf("(Request)");
                    break;
                case USBPD_DATAMSG_Sink_Capabilities: /* receive Sink cap  */
                    printf("(SinkCap)");
                    break;
                    
                case USBPD_DATAMSG_Vendor_Defined:  /* VDM message */ 
                    printf("(VDM)");
                    break;
                default :
                    break;
                }
                
                printf(" ");
            }
            else
            {
                printf("Empty ");
            }
            
#endif
        } //END while
    }
    
    
    
    
//    if( PostProcess_SRC_PDO_Received != 0)
//    {
//        //__disable_irq(); //CMSIS
//        //PostProcess_SRC_PDO_Received--;
//        PostProcess_SRC_PDO_Received = 0;
//        //__enable_irq();
//        
//        Print_PDO_FROM_SRC(UsbPort);
//        
//        //Read_RDO(UsbPort);
//        //Print_RDO(UsbPort);
//        
//        return 0;
//    }
    
//    if( PostProcess_PSRDY_Received != 0)
//    {
//        //__disable_irq(); //CMSIS
//        //PostProcess_PSRDY_Received--;
//        PostProcess_PSRDY_Received = 0;
//        //__enable_irq();
//        
//        printf("\r\n");
//        Print_RDO(UsbPort);
//    }
    
    
    
    return 0;
}
