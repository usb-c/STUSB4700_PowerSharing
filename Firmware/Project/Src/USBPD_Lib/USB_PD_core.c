#include "USB_PD_core.h"
#include "USB_PD_defines_stusb4700.h"
#include "stusb4700_defines_2.h"
#include "i2c.h"
#include "I2C_ReadWrite.h"
#include "Hardware_configuration.h" //for I2CBUS_MAX, USBPORT_MAX
#include "USBPD_spec_defines.h"
#include "global_var_IRQ.h"
#include "utils.h"
#include "printf_debug.h"
#include "PostProcessEvents.h"


#if CONSOLE_PRINTF
#define DEBUG_IRQ
#endif

extern I2C_HandleTypeDef *hi2c[I2CBUS_MAX];
//extern unsigned int I2cDeviceID_7bit;
//extern unsigned int Address;


USB_PD_StatusTypeDef PD_status[USBPORT_MAX] ;
USB_PD_CTRLTypeDef   PD_ctrl[USBPORT_MAX];

USB_PD_SRC_PDOTypeDef PDO_SRC[USBPORT_MAX][5];

//USB_PD_Fixed_EVAL_PDOTypeDef Rx_PDO[USBPORT_MAX];

USB_PD_SNK_PDOTypeDef PDO_FROM_SNK[USBPORT_MAX][3];
uint8_t PDO_FROM_SNK_Num[USBPORT_MAX];
USB_PD_SRC_PDOTypeDef PDO_FROM_SRC[USBPORT_MAX][7];
uint8_t PDO_FROM_SRC_Num[USBPORT_MAX];
uint8_t PDO_FROM_SNK_Valid[USBPORT_MAX];

uint32_t VDM_Message[USBPORT_MAX][7];
uint8_t  USB_PD_VDM_received_Flag[USBPORT_MAX] ;

USB_PD_Debug_FSM_TypeDef Debug_fsm[USBPORT_MAX];

//USB_PD_RDOTypeDef registered_PDO[USBPORT_MAX];
STUSB_RDO_REG_STATUS_RegTypeDef STUSB4700Register_Nego_RDO[USBPORT_MAX];
STUSB_RDO_REG_STATUS_RegTypeDef STUSB4700Register_registered_RDO[USBPORT_MAX];

//uint8_t PDO_SNK_NUMB;
uint8_t PDO_SRC_NUMB[USBPORT_MAX];
uint8_t PD_reinit ;

extern volatile uint8_t  USB_PD_Resquest_received_Flag[USBPORT_MAX] ;

uint8_t USB_PD_Status_change_flag[USBPORT_MAX] ;
uint8_t PortConnected_flag[USBPORT_MAX];
int TotalConnectionCount;
extern USB_PD_I2C_PORT STUSB47DeviceConf[USBPORT_MAX];
uint8_t Cut[USBPORT_MAX];

volatile uint8_t DoActionAfterMsgReceived_GoodCrc[USBPORT_MAX];

extern float PowerAllocated[USBPORT_MAX];


void usb_pd_init(uint8_t UsbPort)
{
    STUSB_ALERT_STATUS_MASK_RegTypeDef STUSB4700Register_AlertMask;
    
    int Status;
    int i;
    static unsigned char DataRW[40];	
    DataRW[0]= 0;
    unsigned int Address;
    
    uint8_t Cut;
    I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,0x2F ,&Cut, 1 );
    
    
    // clear all ALERT Status
    Address = STUSBADDR_ALERT_STATUS_1;
    //for (i=0;i<=12;i++) Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,Address+i ,&DataRW[0], 1 );  // clear ALERT Status
    Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,Address,&DataRW[0], 13 );  // clear ALERT Status
    
    
    
    //    Address = ALERT_STATUS_MASK;
    //    DataRW[0] = ~(TYPEC_MONITORING_STATUS_AL | PORT_STATUS_AL); // interrupt unmask 
    //    Status = I2cWrite(Devices[Port].Bus, Devices[Port].I2cDeviceID_7bit, Address, &DataRW[0], 1); // unmask port status alarm 
    
    
    Address = STUSBADDR_ALERT_STATUS_MASK;
    STUSB4700Register_AlertMask.d8 = 0xFF; // all masked
    STUSB4700Register_AlertMask.bitfield.CC_DETECTION_STATUS_AL_MASK = 0; //PORT_STATUS_AL // interrupt unmask //bit-6 
    STUSB4700Register_AlertMask.bitfield.PRT_STATUS_AL_MASK = 0; // interrupt unmask //bit-1
    //STUSB4700Register_AlertMask.bitfield.MONITORING_STATUS_AL_MASK = 0; //bit-5 //Change occurred on MONITORING_STATUS_TRANS register (see POWER_STATUS register).
    //STUSB4700Register_AlertMask.bitfield.HW_FAULT_STATUS_AL_MASK = 0; //bit-4 //Change occurred on HW_FAULT_STATUS_TRANS register (Hardware fault from analog logic)
    
    DataRW[0]= STUSB4700Register_AlertMask.d8;// interrupt unmask 
    Status = I2cWrite(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,Address ,&DataRW[0], 1 ); // unmask port status alarm 
    
    
    
    USB_PD_Resquest_received_Flag[UsbPort] = 0 ;
    USB_PD_Interupt_Flag[UsbPort]=0 ;
    PDO_FROM_SNK_Valid[UsbPort]=0;
    USB_PD_Status_change_flag[UsbPort]=0 ;
    USB_PD_VDM_received_Flag[UsbPort] = 0 ;
    
    PortConnected_flag[UsbPort]=0;
    
    Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,STUSBADDR_PORT_STATUS ,&DataRW[0], 10 ); 
    PD_status[UsbPort].Port_status.d8 = DataRW[0];
    PD_status[UsbPort].CC_status = DataRW[3];
    PD_status[UsbPort].HWFault_Status.d8 = DataRW[5];
    PD_status[UsbPort].Monitoring_status.d8= DataRW[2];
    // clear ALERT Status
    
    return;
}


void ALARM_MANAGEMENT(uint8_t UsbPort)  // interrupt Handler 
{
    STUSB_ALERT_STATUS_RegTypeDef STUSB4700Register_AlertStatus;
    STUSB_ALERT_STATUS_MASK_RegTypeDef STUSB4700Register_AlertMask;
    
    int i,Status;
    static unsigned char DataRead,Alert_status,Alert_MASK,DataWrite=0;
    
    unsigned char Alert_status_bit;
    static unsigned char DataRW[30];
    unsigned int Address;
    
    if (USB_PD_Interupt_Flag[UsbPort] != 0)
    {
        Address = STUSBADDR_ALERT_STATUS_1; //Not Read/Clear
        Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,Address ,&DataRW[0], 2 );
        
        STUSB4700Register_AlertMask.d8 = DataRW[1]; 
        STUSB4700Register_AlertStatus.d8 = DataRW[0] & ~STUSB4700Register_AlertMask.d8;
        
        
#ifdef DEBUG_IRQ
        Push_IrqReceived(UsbPort, STUSB4700Register_AlertStatus.d8);
#endif
        
        if (STUSB4700Register_AlertStatus.d8 != 0)
        {
            //bit 7
            PD_status[UsbPort].HW_Reset = (DataRW[ 0 ] >> 7);
            if (PD_status[UsbPort].HW_Reset !=0)
            {
                PostProcess_IrqHardreset++;
            }
            
            //bit-6 //Change occurred on PORT_STATUS_TRANS register  (see CC_STATUS register).   //PORT_STATUS_AL
            if (STUSB4700Register_AlertStatus.bitfield.PORT_CC_DETECTION_STATUS_AL !=0)
            {
                //("bit-6 Alarm PORT_STATUS, ");
                
                Address = STUSBADDR_PORT_STATUS_TRANS; //[Read/Clear]
                Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, Address ,&DataRW[0], 2 );
                PD_status[UsbPort].Port_trans =  DataRW[ 0 ]; 
                PD_status[UsbPort].Port_status.d8= DataRW[ 1 ];
                
                if( (DataRW[0] & STUSB4700MASK_ATTACH_STATUS_TRANS) != 0)
                {
                    PostProcess_AttachTransition[UsbPort]++;
                }
            }
            
            //bit-5 //Change occurred on MONITORING_STATUS_TRANS register (see POWER_STATUS register).  //VALUE_TYPEC_MONITORING_STATUS_AL
            if (STUSB4700Register_AlertStatus.bitfield.MONITORING_STATUS_AL !=0)
            {
                Address = STUSBADDR_TYPEC_MONITORING_STATUS_0; //[Read/Clear]
                Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit , Address ,&DataRW[0], 2 );
                PD_status[UsbPort].Monitoring_status.d8 = DataRW[1] ; 
            }
            
            //Always read & update CC Attachement status
            Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit , STUSBADDR_CC_STATUS ,&DataRW[0], 1);
            PD_status[UsbPort].CC_status = DataRW[ 0];
            
            
            //bit-4 //Change occurred on HW_FAULT_STATUS_TRANS register (Hardware fault from analog logic)  //CC_HW_FAULT_STATUS_AL
            if (STUSB4700Register_AlertStatus.bitfield.HW_FAULT_STATUS_AL !=0)
            {
                Address = STUSBADDR_CC_HW_FAULT_STATUS_0; //[Read/Clear]
                Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit , Address ,&DataRW[0], 2 );
                PD_status[UsbPort].HWFault_Status.d8= DataRW[ 1 ]; 
            }
            
            //bit-3 //DPM layer alert  //DPM_STATUS_AL
            if (STUSB4700Register_AlertStatus.bitfield.PD_TYPEC_STATUS_AL !=0)
            {
                Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,RX_HEADER ,&DataRW[0], 30 );
                //printf(" read Rx Buf");
            }
            
            //bit-1 //Protocol layer alert  //PRT_STATUS_AL
            if (STUSB4700Register_AlertStatus.bitfield.PRT_STATUS_AL !=0) //Protocol layer alert (for USB PD mode)
            {                
                USBPD_MsgHeader_TypeDef Header;
                STUSB_PRT_STATUS_RegTypeDef Prt_Status; //HW Protocol Layer
                
                //clear IRQ ALARM
                Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,PRT_STATUS ,&Prt_Status.d8, 1 );
                
                
                if (Prt_Status.bitfield.MSG_RECEIVED == 1)
                {
                    // RX_HEADER = RX message header 
                    Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,RX_HEADER ,(uint8_t *)&Header.d16, 2 );
                    //Header.d16 = LE16(&DataRW[0]);
                    //Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,RX_HEADER ,(uint8_t *)&DataRW[0], 2 );
                    //Header.d16 = LE16(&DataRW[0]);
                    
                    
                    if( Header.bitfield.NumberOfDataObjects > 0  ) //Number_of_Data_Objects field > 0 --> Message is a Data Message
                    {
#ifdef DEBUG_IRQ
                        Push_PD_MessageReceived(UsbPort, 'D', Header.bitfield.MessageType); //DataMsg
#endif
                        
                        int RxByteCount;
                        Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,RX_BYTE_CNT ,&DataRW[0], 1 );
                        RxByteCount = DataRW[0];
                        
                        if(RxByteCount != Header.bitfield.NumberOfDataObjects * 4)
                        {
                            //error, missing Data
                            //return;
                        }
                        
                        switch (Header.bitfield.MessageType )
                        {   
                            
                        case USBPD_DATAMSG_Source_Capabilities : //unused here
                            break;
                            
                        case USBPD_DATAMSG_Request:  /* get request message */
                            {
                                static int i ,j ;                  
                                Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,RX_DATA_OBJ ,(uint8_t *)&STUSB4700Register_Nego_RDO[UsbPort].d32, Header.bitfield.NumberOfDataObjects * 4 ); 
                                
                                USB_PD_Resquest_received_Flag[UsbPort]=1;
                                /* here the request from the sink was done , we just have to wait for transition state on PE state machine 
                                this register is available on adress 0x29*/
                            }break;
                            
                        case USBPD_DATAMSG_Sink_Capabilities: /* receive Sink cap  */
                            {
                                static int i ,j ;                  
                                Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,RX_DATA_OBJ ,&DataRW[0], Header.bitfield.NumberOfDataObjects * 4 ); 
                                j=0;
                                
                                PDO_FROM_SNK_Num[UsbPort]= Header.bitfield.NumberOfDataObjects;
                                
                                for ( i = 0 ; i < Header.bitfield.NumberOfDataObjects ; i++)
                                {
                                    PDO_FROM_SNK[UsbPort][i].d32 = (uint32_t )( DataRW[j] +(DataRW[j+1]<<8)+(DataRW[j+2]<<16)+(DataRW[j+3]<<24));
                                    j +=4;
                                    
                                }
                                PDO_FROM_SNK_Valid[UsbPort]=1;
                            }break;
                            
                        case USBPD_DATAMSG_Vendor_Defined:  /* VDM message */                               {
                            static int i ,j ;                  
                            Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,RX_DATA_OBJ ,&DataRW[0], Header.bitfield.NumberOfDataObjects * 4 ); 
                            j=0;
                            
                            for ( i = 0 ; i < Header.bitfield.NumberOfDataObjects ; i++)
                            {
                                VDM_Message[UsbPort][i] = (uint32_t )( DataRW[j] +(DataRW[j+1]<<8)+(DataRW[j+2]<<16)+(DataRW[j+3]<<24));
                                j +=4;
                            }
                            USB_PD_VDM_received_Flag[UsbPort] = 1 ;
                        }break;
                        
                        
                        default:
                            break;
                        }
                    }
                    
                    
                    else //Number_of_Data_Objects field == 0 --> Message is a Control-Message
                    {
#ifdef DEBUG_IRQ
                        Push_PD_MessageReceived(UsbPort, 'C', Header.bitfield.MessageType); //CtrlMsg
#endif
                        
                        switch (Header.bitfield.MessageType )
                        {   
                            //printf(":%i--\r\n",Header.bitfield.MessageType);
                        case USBPD_CTRLMSG_Reserved1:
                            break;
                            
                        case USBPD_CTRLMSG_GoodCRC:
                            PostProcess_Msg_GoodCRC++;
                            DoActionAfterMsgReceived_GoodCrc[UsbPort] = 1;
                            break;
                            
                        case USBPD_CTRLMSG_Accept:
                            PostProcess_Msg_Accept++;
                            break;
                            
                        case USBPD_CTRLMSG_Reject:
                            PostProcess_Msg_Reject++;
                            break;
                            
                        case USBPD_CTRLMSG_PS_RDY:
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
                        
                    } //END if( Header.b.NumberOfDataObjects > 0 )
                }
                else //if (PD_status[Usb_Port].PRT_status.b.MSG_RECEIVED == 0)
                {
                }
            } //END if (Alert_Status.b.PRT_STATUS_AL !=0) //bit 1 
            
            
            //bit-0 //Physical layer alert
            //VALUE_PHY_STATUS_AL
            if (STUSB4700Register_AlertStatus.bitfield.PHY_STATUS_AL !=0)
            {
                //("bit-0 Alarm PHY_STATUS, ");
            }
        }
        
        
        USB_PD_Interupt_Flag[UsbPort]--; //clear current IRQ
        
        if( USB_PD_Interupt_Flag[UsbPort] < 0)
        {
            //should not happen, to be debugged
            printf("\r\n---- Error too many IRQ not processed:");
            while(1);
        }
        
        
        
        
#if (CONSOLE_PRINTF && DBG_LEVEL_2)
        Address = STUSBADDR_ALERT_STATUS_1; 
        Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,Address ,&DataRW[0], 2 );
        
        printf("\r\nEND AlertStatus-Reg: 0x%X, ", DataRW[0]);
        printf("Useful: 0x%X\r\n", DataRW[0] & (~DataRW[1]) );
#endif
        
        
#if 0
        //for debug
        Print_PolicyEngine_StateMachineFSM(UsbPort);        
        
        //for(UsbPort = 0; UsbPort < USBPORT_MAX; UsbPort++)
        {
            int Pdo_nb = Get_PD_Contract(UsbPort, &PowerAllocated[UsbPort]);
        }
        
        //        printf("\r\n---- Checking all PDOs status:");
        //        for(UsbPort = 0; UsbPort < USBPORT_MAX; UsbPort++)
        //        {
        //            Print_PDO(UsbPort);
        //        }
#endif
    }
} 


//void Read_PDO(uint8_t UsbPort)
void Get_PDO(uint8_t UsbPort) //get & update the PDO from STUSB47
{
    unsigned char DataRW[40];	
    DataRW[0]= 0;
    
    static int i =0,j = 1 ;
    uint32_t PDO ;
    unsigned int Address;
    
    Address = STUSBADDR_DPM_PDO_NUMB ;
    if ( I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,Address ,&DataRW[0], 39 )== HAL_I2C_ERROR_NONE  ) 
    {
        
        PDO_SRC_NUMB[UsbPort] = ((DataRW[0] & 0xE0 ) >> 5) ;
        j=1;
        for ( i = 0 ; i < PDO_SRC_NUMB[UsbPort] ; i++)
        {
            PDO_SRC[UsbPort][i].d32 = (uint32_t )( DataRW[j] +(DataRW[j+1]<<8)+(DataRW[j+2]<<16)+(DataRW[j+3]<<24));
            
            j +=4;
        }
        j +=12;
    }
    
    //USB_PD_Interupt_Flag[UsbPort] =0;
    return;
}

void Print_PDO(uint8_t UsbPort)  
{
#ifdef CONSOLE_PRINTF
    
    static uint8_t i;
    static float PDO_V;
    static float PDO_I;
    float   PDO_P;
    float MAX_POWER;
    
    MAX_POWER = 0;
    
    // Does not work if instanciated here
    
    printf("\r\n");
    printf("                             | ");
    printf("---- UsbPort #%i : Get PDO from registers ------\r\n",UsbPort);
    
    Get_PDO(UsbPort); //get & update the PDO from STUSB47
    
    printf("                             | ");
    printf("PDO_number: %d \r\n", PDO_SRC_NUMB[UsbPort] );
    
    for (i=0; i< PDO_SRC_NUMB[UsbPort]; i++)
    {
        PDO_V = (float) (PDO_SRC[UsbPort][i].fix.Voltage)/20;
        PDO_I = (float) (PDO_SRC[UsbPort][i].fix.Max_Operating_Current)/100;
        PDO_P = (float) (PDO_V * PDO_I); 
        
        printf("                             | ");
        printf(" - PDO%u=(%4.2fV, %4.2fA, = %4.1fW)\r\n",i, PDO_V, PDO_I, PDO_P );
        
        if (PDO_P >=MAX_POWER)
        { 
            MAX_POWER = PDO_P;
        }
    }
    
    printf("                             | ");
    printf("P(max)=%4.1fW \r\n", MAX_POWER);
    printf("                             | ");
    printf("Power_allocated=%4.1fW \r\n", PowerAllocated[UsbPort]);
    
#endif  
}

void Print_PDO_old(uint8_t UsbPort)  
{
#ifdef CONSOLE_PRINTF
    
    static uint8_t i;
    static float PDO_V;
    static float PDO_I;
    static int   PDO_P;
    static int MAX_POWER = 0;
    MAX_POWER = 0;
    
    // Does not work if instanciated here
    
    printf("\r\n---- UsbPort #%i : Read PDO from NVM ------\r\n",UsbPort);
    
    Get_PDO(UsbPort);
    printf("%x x PDO:\r\n",PDO_SRC_NUMB[UsbPort]);
    
    for (i=0; i< PDO_SRC_NUMB[UsbPort]; i++)
    {
        PDO_V = (float) (PDO_SRC[UsbPort][i].fix.Voltage)/20;
        PDO_I = (float) (PDO_SRC[UsbPort][i].fix.Max_Operating_Current)/100;
        PDO_P = (int) (PDO_V * PDO_I); 
        
        printf(" - PDO%u=(%4.2fV, %4.2fA, = %uW)\r\n",i, PDO_V, PDO_I, PDO_P );
        
        if (PDO_P >=MAX_POWER)
        { MAX_POWER = PDO_P;}
    }
    
    printf("P(max)=%uW\r\n", MAX_POWER );
    
#endif  
}

void Print_PDO_FROM_SNK(uint8_t UsbPort)  
{
    static uint8_t i;
    static float PDO_V;
    static float PDO_I;
    static int   PDO_P;
    static int MAX_POWER = 0;
    MAX_POWER = 0;
    
    // Does not work if instanciated here
#ifdef CONSOLE_PRINTF
    printf("\r\n---- UsbPort #%i : Read PDO from Connected Sink ------\r\n",UsbPort);
#endif
    //    Get_PDO(UsbPort);
#ifdef CONSOLE_PRINTF          
    printf("%x x PDO:\r\n",PDO_FROM_SNK_Num[UsbPort]);
#endif
    for (i=0; i< PDO_FROM_SNK_Num[UsbPort]; i++)
    {
        PDO_V = (float) (PDO_FROM_SNK[UsbPort][i].fix.Voltage)/20;
        PDO_I = (float) (PDO_FROM_SNK[UsbPort][i].fix.Max_Operating_Current)/100;
        PDO_P = (int) PDO_V*PDO_I; 
#ifdef CONSOLE_PRINTF   
        printf(" - PDO%u=(%4.2fV, %4.2fA, = %uW)\r\n",i, PDO_V, PDO_I, PDO_P );
#endif 
        if (PDO_P >=MAX_POWER)
        { MAX_POWER = PDO_P;}
    }
#ifdef CONSOLE_PRINTF  
    printf("P(max)=%uW\r\n", MAX_POWER );
#endif  
    //printf("BEN: END PRINT_PDO\n");
}

int Update_TypeC_resistor_with_new_budget(uint8_t UsbPort, float PowerMax)
{
    if(PowerMax >= 15) // 5V*3A=15W
    {
        SetRpResistorValue(UsbPort, POWER_RP_3A);
    }
    else if(PowerMax >= 7.5) // 5V*1.5A=15W
    {
        SetRpResistorValue(UsbPort, POWER_RP_1_5A);
    }
    else
    {
        SetRpResistorValue(UsbPort, POWER_RP_LEGACY);
    }
    
    return 0;
}


int PDO_Source_Voltage_Custom[PDO_NUMBER_MAX] = PDO_VOLTAGE_TABLE;

//void Update_all_PDO_with_new_budget (uint8_t UsbPort,uint8_t P_MAX, float I_MIN)
void Update_all_PDO_with_new_budget (uint8_t UsbPort, float P_max, float I_min)
{
    
    static int i;
    int Status;
    static float PDO_V;
    static float PDO_I;
    float   PDO_P;
    
    uint32_t new_PDO ;
    uint8_t new_PDO_numb;
    static unsigned char RW_Buffer;
    unsigned int Address;
    
    //if (P_max >= 20) {PDO_SRC_NUMB[UsbPort] = 5;}
    new_PDO_numb=0;
    
#ifdef CONSOLE_PRINTF
    printf("    # ");
    //printf("UsbPort #%i : Calculating PDO with MAX_POWER=%4.1fW and MIN_CURRENT_per_PDO=%4.2fA\r\n",UsbPort,P_max, I_min );
    printf("UsbPort #%i: Calc. PDO with MAX_POWER=%4.1fW and MIN_CURRENT_per_PDO=%4.2fA\r\n", UsbPort, P_max, I_min );
#endif
    
    for (i=0; i< PDO_NUMBER_MAX; i++)
    {
        //PDO_V = (float) (PDO_SRC[UsbPort][i].fix.Voltage)/20;
        PDO_V = (float) PDO_Source_Voltage_Custom[i];
        PDO_I = P_max/PDO_V;
        
        if (PDO_I > MAX_CURRENT_PER_PDO) { PDO_I = MAX_CURRENT_PER_PDO;}
        
        if (PDO_I <I_min) 
        { 
#ifdef CONSOLE_PRINTF
            printf("    # ");
            printf(" - PDO%u=%4.2fV , I=%4.2fA => Current too low, removing PDO\r\n",i, PDO_V, PDO_I);   
#endif
            break;
        }
        else
        {
            new_PDO_numb++;
        }
        
        PDO_P = (float) (PDO_V*PDO_I); 
        
#ifdef CONSOLE_PRINTF
        printf("    # ");
        printf(" - Writing new PDO%u=(%4.2fV, %4.2fA, = %4.1fW)\r\n",i, PDO_V, PDO_I, PDO_P );
#endif
        
        PDO_SRC[UsbPort][i].fix.Voltage = (uint16_t)(20 * PDO_Source_Voltage_Custom[i]); //overwrite the default unknown Voltage config
        PDO_SRC[UsbPort][i].fix.Max_Operating_Current = (uint16_t)(100 * PDO_I);
        //printf("PDO_SRC[%d].Current=%u \n",i,PDO_SRC[i].Current);
        
        Address = STUSBADDR_DPM_SRC_PDO1 + 4 * i;
        new_PDO = PDO_SRC[UsbPort][i].d32;
        Status = I2cWrite(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,Address ,(uint8_t *)&PDO_SRC[UsbPort][i].d32, 4 ) ;
        
        if (Status != HAL_I2C_ERROR_NONE) return ;
        
        //new_PDO_numb = new_PDO_numb+1;
    }
    
    PDO_SRC_NUMB[UsbPort] = new_PDO_numb;
#ifdef CONSOLE_PRINTF
    //	printf("New PDO number=%d \n",PDO_SRC_NUMB[UsbPort]);
#endif
    
    // write new PDO number to STUSB
    Address = STUSBADDR_DPM_PDO_NUMB;
    RW_Buffer =  ((PDO_SRC_NUMB[UsbPort]<<5)& 0xE0);
    I2cWrite(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,Address ,&RW_Buffer, 1) ;
    
    // set PD_COMMAND = SRC_capa_changed
    Address = PD_COMMAND;
    RW_Buffer =  SRC_capa_changed;   
    I2cWrite(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,Address ,&RW_Buffer, 1) ;
}


/*  ----------------- CHECK_CONTRACT -------------------------------------------
This function can be optionally called when a Type-C port is connected to a sink
(valid connection). It allows to report which has been the agreed contract 
between the STUSB47 and the connected device.

In case a type-C only device is connected (implicit contract), the connection 
can be:
- Rp = STD USB
- Rp = 1.5A
- Rp = 3.0A

In case a USB PD device is connected (explicit contract), the PDO number 
selected by the sink is reported, allowing to understand which power budget 
is allocated to the attached port.

return 0 ATTACHED to type-C
return -1 if NOT attached 
return PDO number if ATTACHED TO PD DEVICE
------------------------------------------------------------------------------*/
int Check_Contract(uint8_t Port)
{
    return Get_PD_Contract(Port, NULL);
}

//int Check_Contract2(uint8_t Port)
int Get_PD_Contract(uint8_t UsbPort, float * pUsedPower) 
{
    static uint8_t i;
    int Status;
    static float PDO_V;
    static float PDO_I;
    //static int   PDO_P;
    float PDO_P;
    
    static int MAX_POWER = 0;
    uint8_t Pdo_nb=0;
    static uint8_t DataRW[4];
    uint32_t RDO ;
    uint8_t CONTRACT_nb;
    unsigned int attached_status;
    int ReturnValue = -1;
    uint16_t Address;
    
    //printf("\r\n");
    
    
    // CHECK ATTACH STATUS
    attached_status = PD_status[UsbPort].Port_status.bitfield.CC_ATTACH_STATE;
    
    switch(attached_status)
    {
    case 0:
        
#ifdef CONSOLE_PRINTF 
        printf("          | ");
        printf("---- UsbPort #%i: NO device attached ", UsbPort);
        printf("\r\n");
#endif  
        //return -1;
        ReturnValue = -1;
        break;
        
    case 1: //attached
        
        // READ CONTRACT NUMBER (depending on Device version)
        Address = RDO_REG_STATUS; 
        I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit , Address, (uint8_t *)&STUSB4700Register_Nego_RDO[UsbPort], 4 );
        
        if(Cut[UsbPort] == 0) //variable not initialize
        {
            I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,STUSB47ADDR_DEVICE_ID ,&Cut[UsbPort], 1 );
        }
        
        if (((Cut[UsbPort] >> 2) & 0x07) > 3 ) // check silicon version 
            Pdo_nb = STUSB4700Register_Nego_RDO[UsbPort].bitfield.Object_Pos;
        else Pdo_nb = (uint8_t)(STUSB4700Register_Nego_RDO[UsbPort].d32 & 0x07);
        
        //       registered_PDO[UsbPort].Position = CONTRACT_nb;
        // for cut 1.3 other values must be added 
        
        
        Address = STUSBADDR_PE_FSM; 
        I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, Address ,&DataRW[0], 1 ); 
        
        switch(DataRW[0])
        {
        case PE_SRC_READY:
            {
                PDO_V = (float) (PDO_SRC[UsbPort][Pdo_nb-1].fix.Voltage)/20;
                PDO_I = (float) (PDO_SRC[UsbPort][Pdo_nb-1].fix.Max_Operating_Current)/100;
                PDO_P = (float) PDO_V*PDO_I;
                
#ifdef CONSOLE_PRINTF
                int ContractNb = Pdo_nb-1;
                printf("          | ");
                printf("---- UsbPort #%i: Explicit contract: PDO%u=(%4.2fV, %4.2fA, = %4.2fW)", UsbPort, ContractNb, PDO_V, PDO_I, PDO_P );
                printf("\r\n");
#endif 
                if (pUsedPower != NULL)
                {
                    *pUsedPower = PDO_P;
                }
                
                //return (CONTRACT_nb-1);
                //return CONTRACT_nb;
                //return Pdo_nb;
                ReturnValue = Pdo_nb;
            }
            break;
            
        case PE_SRC_DISCOVERY:
        case PE_SRC_TRANSITION_SUPPLY_2:
        case PE_VCS_DFP_SEND_PS_RDY:
            {
#ifdef CONSOLE_PRINTF
                printf("          | ");
                printf("---- UsbPort #%i: PD communication ongoing (0x%X)", UsbPort, DataRW[0] );
                printf("\r\n");
#endif 
            }
            //break; //continue with Type-C resistor
            
        case PE_INIT:
        case PE_SRC_DISABLED: //Implicit contract (pull-up Rp)
            {
                float Power = 0;
                
                Address = PD_ROLE_CNTRL; 
                I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit , Address, &DataRW[0], 1 ); 
                
                uint8_t FieldValue = (DataRW[0] & RP_VALUE) >> RP_VALUE_POSITION;
                
                switch (FieldValue)
                {
                case Rp_standard_current:
                    Power = 5 * 0.9;
#ifdef CONSOLE_PRINTF
                    printf("          | ");
                    printf("---- UsbPort #%i: Implicit type-C contract: 5V; Rp=std ", UsbPort);
                    printf("\r\n");
#endif
                    break;
                    
                case Rp_1_5A:
                    Power = 5 * 1.5;
#ifdef CONSOLE_PRINTF
                    printf("          | ");
                    printf("---- UsbPort #%i: Implicit type-C contract: 5V; Rp=1.50A ", UsbPort);
                    printf("\r\n");
#endif
                    break;
                    
                case Rp_3A:
                    Power = 5 * 3;
#ifdef CONSOLE_PRINTF
                    printf("          | ");
                    printf("---- UsbPort #%i: Implicit type-C contract: 5V; Rp=3.00A ", UsbPort);
                    printf("\r\n");
#endif
                    break;
                }
                
                if (pUsedPower != NULL)
                {
                    *pUsedPower = Power;
                }
                
                //return 0;
                //return CONTRACT_nb;
                //return Pdo_nb;
                ReturnValue = Pdo_nb;
            }
            break;
            
        default:
            {
                printf("          | ");
                printf("---- UsbPort #%i: Unmanaged PE_FSM State: 0x%02X ", UsbPort, DataRW[0]);
                printf("\r\n");
            }
            break;
        }
    }
    
    
#if (CONSOLE_PRINTF && DBG_LEVEL_2)
    printf("          | ");
    Print_PolicyEngine_StateMachineFSM(UsbPort);
#endif 
    
    //return -1;
    return ReturnValue;
}

int SetRpResistorValue(uint8_t UsbPort, int Param)
{
    uint8_t Data;
    uint8_t Mask;
    uint8_t NewMask = 0;
    int Status;
    
    switch(Param)
    {
    default:
    case POWER_RP_LEGACY:
        NewMask = Rp_standard_current << RP_VALUE_POSITION;
        break;
    case POWER_RP_1_5A:
        NewMask = Rp_1_5A << RP_VALUE_POSITION;
        break;
    case POWER_RP_3A:
        NewMask = Rp_3A << RP_VALUE_POSITION;
        break;
    }
    
    
    uint16_t Address = PD_ROLE_CNTRL;
    
    Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, PD_ROLE_CNTRL, &Data, 1);
    if (Status != HAL_I2C_ERROR_NONE) return -1;
    
    Mask = (RP_VALUE_MSK << RP_VALUE_POSITION);
    Data &=  ~(Mask); //clear bits to 0
    Data |= NewMask; //set bits
    
    Status = I2cWrite(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, Address, &Data, 1);
    if (Status != HAL_I2C_ERROR_NONE) return -1;
    
    return 0;
}


//-----------------------------------
//Check USB-C cable attachment status
//return -1 if not connected
//return 1 if connected on CC1
//return 2 if connected on CC2
//-----------------------------------
int CheckCableAttached(uint8_t UsbPort)
{
    int status;
    //uint8_t UsbPort = 0;
    uint8_t Data;
    uint8_t Address;
    
    // read CC pin Attachment status
    status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, STUSBADDR_PORT_STATUS, &Data, 1);  if(status != 0) { return -2; }
    
    if( (Data & STUSBMASK_ATTACHED_STATUS) == VALUE_ATTACHED) //only if USB-C cable attached
    {
        Address = STUSBADDR_TYPEC_STATUS; //[Read only]
        status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, Address , &Data, 1 ); if(status != 0) return -2; //I2C Error
        
        if(( Data & MASK_REVERSE) == 0)
        {
            return 1;  //OK, cable attached on CC1 pin
        }
        else
        {
            return 2;  //OK, cable attached on CC2 pin
        }
    }
    else
    {
        return -1; //Error, USB-C cable not attached
    }
}

/*  ----------------- GET_SINK_CAP -------------------------------------------*/

int Get_sink_cap(uint8_t UsbPort)
{
    int Status;
    uint8_t New_CMD;
    
    if(CheckCableAttached(UsbPort) >= 1) //USB-C cable attached
    {
        
        PDO_FROM_SNK_Valid[UsbPort] = 0;  // 
        // SEND GET_SINK_CAP command
        
        New_CMD = 0x0C;    
        Status = I2cWrite(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, PD_COMMAND ,&New_CMD, 1 ) ;
        return Status;
        
        // this command will be sent by the source to the sink that must answer within some ms 
        // the interrupt handler will put the datat's in PDO_FROM_SNK buffer 
        // PDO_FROM_SNK_Valid[UsbPort] = 1 when the PDO's are available for Main 
        
    }
}







void ForceClear_PRT_STATUS_AL(uint8_t UsbPort)
{
    unsigned int Address;
    int Status;
    STUSB_ALERT_STATUS_RegTypeDef STUSB4700Register_AlertStatus;
    STUSB_ALERT_STATUS_MASK_RegTypeDef STUSB4700Register_AlertMask;
    unsigned char DataRW[2];
    
    Address = STUSBADDR_ALERT_STATUS_1; //[Read/Clear]
    Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,Address ,&DataRW[0], 2 );
    
    STUSB4700Register_AlertMask.d8 = DataRW[1]; 
    STUSB4700Register_AlertStatus.d8 = DataRW[0] & ~STUSB4700Register_AlertMask.d8;
    
    //bit-1 //Protocol layer alert
    //PRT_STATUS_AL
    if (STUSB4700Register_AlertStatus.bitfield.PRT_STATUS_AL !=0) //Protocol layer alert (for USB PD mode)
    {
        int Status;
        
#if CONSOLE_PRINTF
        printf("bit-1 Alarm PRT_STATUS, ");
#endif
        
        USBPD_MsgHeader_TypeDef Header;
        STUSB_PRT_STATUS_RegTypeDef Prt_Status; //HW Protocol Layer
        //clear IRQ ALARM
        Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,PRT_STATUS ,&Prt_Status.d8, 1 );
        
#if CONSOLE_PRINTF
        if((Prt_Status.d8 & VALUE_PRL_TX_ERR) != 0)
        {
            printf("Protocol_Layer PRL_TX_ERR. ");
        }
        if((Prt_Status.d8 & VALUE_PRL_BIST_SENT) != 0)
        {
            printf("Protocol_Layer PRL_BIST_SENT. ");
        }
        if((Prt_Status.d8 & VALUE_PRL_BIST_RECEIVED) != 0)
        {
            printf("Protocol_Layer PRL_BIST_RECEIVED. ");
        }
        if((Prt_Status.d8 & VALUE_PRL_MSG_SENT) != 0)
        {
            printf("Protocol_Layer PRL_MSG_SENT. ");
        }
        if((Prt_Status.d8 & VALUE_PRL_MSG_RECEIVED) != 0)
        {
            printf("Protocol_Layer PRL_MSG_RECEIVED. ");
        }
        if((Prt_Status.d8 & VALUE_PRL_HW_RST_DONE) != 0)
        {
            printf("Protocol_Layer PRL_HW_RST_DONE. ");
        }
        if((Prt_Status.d8 & VALUE_PRL_HW_RST_RECEIVED) != 0)
        {
            printf("Protocol_Layer HW_RST_RECEIVED. ");
        }
#endif
    }
}


void Clear_All_pendingIRQ(uint8_t UsbPort)
{
    unsigned int Address;
    int Status;
    uint8_t DataR;
    uint8_t DataRW[30];
    
    printf("Clearing all IRQ on Port %d ... \r\n", UsbPort);
    
    Address = STUSBADDR_ALERT_STATUS_1;
    Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,Address ,&DataR, 1 );
    
    if(DataR != 0)
    {
        //bit-7 //Hard Reset Message received
        if ((DataR & VALUE_HARD_RESET_AL) != 0)
        {
            DBGPRINTF("bit-7 HARD_RESET, ");
        }
        
        //bit-6 //Change occurred on PORT_STATUS_TRANS register  (see CC_STATUS register).
        //PORT_STATUS_AL
        if ((DataR & VALUE_PORT_STATUS_AL) != 0)
        {
            DBGPRINTF("bit-6 Alarm PORT_STATUS, ");
            
            Address = STUSBADDR_PORT_STATUS_TRANS; //[Read/Clear]
            Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, Address ,&DataRW[0], 2 );
        }
        
        //bit-5 //Change occurred on MONITORING_STATUS_TRANS register (see POWER_STATUS register).
        //VALUE_TYPEC_MONITORING_STATUS_AL
        if ((DataR & VALUE_TYPEC_MONITORING_STATUS_AL) != 0)
        {
            DBGPRINTF("bit-5 Alarm TYPEC_MONITORING_STATUS, ");
            
            Address = STUSBADDR_TYPEC_MONITORING_STATUS_0; //[Read/Clear]
            Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit , Address ,&DataRW[0], 2 );
        }
        
        //bit-4 //Change occurred on HW_FAULT_STATUS_TRANS register (Hardware fault from analog logic)
        //CC_HW_FAULT_STATUS_AL
        if ((DataR & VALUE_CC_HW_FAULT_STATUS_AL) != 0)
        {
            
            DBGPRINTF("bit-4 Alarm C_HW_FAULT_STATUS, ");
            
            Address = STUSBADDR_CC_HW_FAULT_STATUS_0; //[Read/Clear]
            Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit , Address ,&DataRW[0], 2 );
        }
        
        
        //bit-3 //DPM layer alert
        //DPM_STATUS_AL
        if ((DataR & VALUE_DPM_STATUS_AL) != 0)
        {
            DBGPRINTF("bit-3 Alarm DPM_STATUS, ");
            
            Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,RX_HEADER ,&DataRW[0], 30 );
        }
        
        //bit-2 //Policy Engine layer alert (Reserved)
        if ((DataR & VALUE_PE_STATUS_AL) != 0)
        {
            DBGPRINTF("bit-2 PE_STATUS, ");
        }
        
        //bit-1 //Protocol layer alert
        //PRT_STATUS_AL
        if ((DataR & VALUE_PRT_STATUS_AL) != 0) //Protocol layer alert (for USB PD mode)
        {
            DBGPRINTF("bit-1 Alarm PRT_STATUS, ");
            
            //clear STATUS REGISTER
            Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,PRT_STATUS ,&DataRW[0], 1 );
        }
        
        //bit-0 //Physical layer alert
        //VALUE_PHY_STATUS_AL
        if ((DataR & VALUE_PHY_STATUS_AL) != 0)
        {
            DBGPRINTF("bit-0 Alarm PHY_STATUS, ");
            
            //clear STATUS REGISTER
            Address = STUSBADDR_PHY_STATUS; //[Read/Clear]
            Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit , Address ,&DataRW[0], 1 );
        }
        
        DBGPRINTF("\r\n");
    }
    
    //re-check all Alarms cleared
    Address = STUSBADDR_ALERT_STATUS_1;
    Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,Address ,&DataR, 1 );
    
    if(DataR != 0)
    {
        DBGPRINTF("Error, IRQ still pending \r\n");
    }
}
