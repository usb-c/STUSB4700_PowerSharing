

#include "USB_PD_user.h"
#include "USB_PD_core.h"
#include "i2c.h"
#include "I2C_ReadWrite.h"
#include "Hardware_configuration.h" //for I2CBUS_MAX, USBPORT_MAX

#include "USB_PD_defines_stusb4700.h"
#include "stusb4700_defines_2.h"
#include "global_var_IRQ.h"

#include "printf_debug.h"


extern I2C_HandleTypeDef *hi2c[USBPORT_MAX];	
//extern unsigned int Address;

extern USB_PD_StatusTypeDef PD_status[USBPORT_MAX] ;
extern USB_PD_CTRLTypeDef   PD_ctrl[USBPORT_MAX];

extern USB_PD_SRC_PDOTypeDef PDO_SRC[USBPORT_MAX][5];

extern USB_PD_SNK_PDOTypeDef PDO_FROM_SNK[USBPORT_MAX][3];
extern uint8_t PDO_FROM_SNK_Num[USBPORT_MAX];
extern USB_PD_SRC_PDOTypeDef PDO_FROM_SRC[USBPORT_MAX][7];
extern uint8_t PDO_FROM_SRC_Num[USBPORT_MAX];

extern USB_PD_Debug_FSM_TypeDef Debug_fsm[USBPORT_MAX];

extern STUSB_RDO_REG_STATUS_RegTypeDef STUSB4700Register_Nego_RDO[USBPORT_MAX];
extern STUSB_RDO_REG_STATUS_RegTypeDef STUSB4700Register_registered_RDO[USBPORT_MAX];

//extern volatile uint16_t Timer_Action_Flag[USBPORT_MAX] ;
extern uint8_t USB_PD_Status_change_flag[USBPORT_MAX] ;
extern uint8_t PortConnected_flag[USBPORT_MAX];
extern int TotalConnectionCount;

extern USB_PD_I2C_PORT STUSB47DeviceConf[USBPORT_MAX];
extern uint32_t timer_cnt;
static int Time_elapse=1;
int Flag_count = 0;
//extern int UsbPort_max;
extern int cpt;
extern uint8_t Cut[USBPORT_MAX];
extern int PB_press;

extern float PowerAllocated[USBPORT_MAX];

int ComputeNewPower_forUnusedPort(int UsbPort)
{
    //Pdo_nb = Check_Contract(UsbPort);
    PowerAllocated[UsbPort] = 0;
    Get_PD_Contract(UsbPort, &PowerAllocated[UsbPort]);
    
    
    //---------------------------------------------------------
    int TotalPortConnected = 0;
    for(int Port = 0; Port < USBPORT_MAX; Port++)
    {
        if (PD_status[Port].Port_status.bitfield.CC_ATTACH_STATE == 1) //attached
            TotalPortConnected++;
    }
    int TotalPortUnconnected = USBPORT_MAX - TotalPortConnected;
    
    
    if(TotalPortUnconnected <= 0) //no need to compute
        return 0; 
    //---------------------------------------------------------
    
    float TotalSystemPower = TOTAL_SYSTEM_POWER; //60W demo
    float TotalPowerAllocated = 0;
    for(int i = 0; i < USBPORT_MAX; i++)
    {
        TotalPowerAllocated += PowerAllocated[i];
    }
    float TotalPower_remaining = (int)(TotalSystemPower - TotalPowerAllocated);
    
    float Power_remaining = 0;    
    if(TotalPortUnconnected == 1)
    { 
        //give all the remaining power
        Power_remaining = TotalPower_remaining;
    }
    else if(TotalPortUnconnected >= 2)
    {
        int MinPowerRequiredForOtherPorts = (TotalPortUnconnected-1) * SMALLEST_POWER_PER_USBPORT;
        Power_remaining = TotalPower_remaining - MinPowerRequiredForOtherPorts;
    }
    
    if(Power_remaining < 0)
    {
        //error, one Port is taking too much power
        
#ifdef CONSOLE_PRINTF
        printf( "Error: Power_remaining=%f \r\n", Power_remaining);                   
#endif
        return -1;
    }
    
    float Power_MAX_remaining_PerPDO = 0;
    if(Power_remaining > MAX_POWER_PER_USBPORT) 
    {
        Power_MAX_remaining_PerPDO = MAX_POWER_PER_USBPORT;
    }
    else if(Power_remaining < SMALLEST_POWER_PER_USBPORT)
    {
        Power_MAX_remaining_PerPDO = SMALLEST_POWER_PER_USBPORT; //should not happen
    }
    else
    {
        Power_MAX_remaining_PerPDO = Power_remaining;
    }
    
    //---------------------------------------------------------
    
    for(int NextPort = 0; NextPort < USBPORT_MAX; NextPort++)
    {
        if(NextPort == UsbPort) continue; //skip the current UsbPort
        
        
        if (PD_status[NextPort].Port_status.bitfield.CC_ATTACH_STATE == 0) //OtherPort Not attached, so can be changed // port i is connected but not the other 
        { 
#ifdef CONSOLE_PRINTF
            printf("    / ");
            printf( "Evaluation of new PDO for UsbPort #%i \r\n", NextPort);                   
#endif
            
            
            //change other ports:
            //Power_remaining = (int)(TotalSystemPower - PowerAllocated[UsbPort]);
            float I_MIN_perPDO = MIN_CURRENT_PER_PDO;
            Update_all_PDO_with_new_budget(NextPort, Power_MAX_remaining_PerPDO, I_MIN_perPDO);
            
            Update_TypeC_resistor_with_new_budget(NextPort, Power_MAX_remaining_PerPDO);
            
        }
    }
    
#if 0    
#ifdef CONSOLE_PRINTF
    printf("\r\n---- Checking all PDOs status:");
    
    for (int p = 0; p < USBPORT_MAX; p++)
    {
        Print_PDO(p);
    }
#endif
#endif
    
    return 0;
    
}




//void Dual_TypeC_power_sharing(int UsbPort_max)
void MultiPort_PowerSharing_polling(int UsbPort)
{
    //int UsbPort;
    static unsigned char DataRW[4];
    unsigned int RDO ;
    uint8_t Pdo_nb;
    
    //for ( UsbPort = 0 ;UsbPort < UsbPort_max ; UsbPort++)
    {
        //I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,STUSB47ADDR_DEVICE_ID ,&Cut[UsbPort], 1 );
        //        if ((Cut[UsbPort] ==  0x8F ) ||(Cut[UsbPort] ==  0x93 ) )// 4710 or 4710A 
        //        ALARM_MANAGEMENT(UsbPort);  // Interrupt management
        
        
        if (PD_status[UsbPort].Port_status.bitfield.CC_ATTACH_STATE !=0)  //  check if Port is connected 
        {
            PortConnected_flag[UsbPort]++;
            
            if (PortConnected_flag[UsbPort]== 1) // first time loop 
            {
                TotalConnectionCount++;
                
                //HAL_Delay(1000);
                HAL_Delay(10); //wait for registers for STUSB4700Register_Nego_RDO[] to be ready
                
                
                I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,RDO_REG_STATUS ,(uint8_t *)&STUSB4700Register_Nego_RDO[UsbPort], 4 );
                //STUSB4700Register_Nego_RDO[UsbPort].d32;
#if 0 // code not used
                if (((Cut[UsbPort] >> 2) & 0x07) > 3 ) // check silicon version 
                    Pdo_nb = STUSB4700Register_Nego_RDO[UsbPort].bitfield.Object_Pos;
                else Pdo_nb = (uint8_t)(STUSB4700Register_Nego_RDO[UsbPort].d32 & 0x07);
#endif
                
                
                DBGPRINTF("\r\n----- Connection UsbPort # %i -----",UsbPort);
                DBGPRINTF("   (total=%i)", TotalConnectionCount);
                DBGPRINTF("\r\n");
                
                ComputeNewPower_forUnusedPort(UsbPort);
                
            }
            else  // polling PDO Changed without cable disconnection
            {
                // port connection already detected 
                PortConnected_flag[UsbPort]=2;
                
                
                //warning: potential register disynchornisation between PD_status[UsbPort].Port_status and real I2C register (I2cRead)
                //so wait PD comm is not busy
                { 
                    uint16_t Address = STUSBADDR_PE_FSM; 
                    I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, Address ,&DataRW[0], 1 ); 
                    
                    if( ((DataRW[0] != PE_SRC_READY) && (DataRW[0] != PE_SRC_DISABLED) && (DataRW[0] != PE_INIT)) )
                    {
#if 0 //non-blocking implementation
                        
                        DBGPRINTF("wait(PE=%X)/",DataRW[0]);
                        
                        //comm busy, come back later
                        return;
                        
#else //blocking implementation
                        {
                            //wait the new PDO negociated are ready to read
                            int Timeout = 0;
                            
                            //force wait PD communication finished
                            do
                            {
                                uint16_t Address = STUSBADDR_PE_FSM; 
                                I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, Address ,&DataRW[0], 1 ); 
                                
                                DBGPRINTF("PE:%X/",DataRW[0]);
                                HAL_Delay(50);
                            }
                            while( ((DataRW[0] != PE_SRC_READY) && (DataRW[0] != PE_SRC_DISABLED)) && (DataRW[0] != PE_INIT) && (Timeout == 0) );
                            
                            //clear register to be able to received a new IRQ (otherwise stuck)
                            //ForceClear_PRT_STATUS_AL(UsbPort);
                        }
#endif
                    }
                    
                }
                
                // check if change occures
                I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,RDO_REG_STATUS ,(uint8_t *)&STUSB4700Register_Nego_RDO[UsbPort], 4 );
                if (((Cut[UsbPort] >> 2) & 0x07) > 3 ) // check silicon version 
                    Pdo_nb = STUSB4700Register_Nego_RDO[UsbPort].bitfield.Object_Pos;
                else Pdo_nb = (uint8_t)(STUSB4700Register_Nego_RDO[UsbPort].d32 & 0x07);
                
                if (STUSB4700Register_registered_RDO[UsbPort].bitfield.Object_Pos != Pdo_nb)
                {
                    STUSB4700Register_registered_RDO[UsbPort].d32 =  STUSB4700Register_Nego_RDO[UsbPort].d32;
                    
                    DBGPRINTF("PDO change on UsbPort %i without re-connection",UsbPort);
                    DBGPRINTF("\r\n");
                    
                    ComputeNewPower_forUnusedPort(UsbPort);
                }
                else
                {
#if 0
                    DBGPRINTF("No need ComputeNewPower\r\n");
#endif
                }
            }
        }  
        else //UsbPort not connected
        {
            if ( PortConnected_flag[UsbPort]!=0 ) // first disconnection action port i 
            {
                PortConnected_flag[UsbPort]=0;
                TotalConnectionCount--;
                
                //printf("\r\n----- Disconnection UsbPort # %i -----\r\n",UsbPort);
                DBGPRINTF("\r\n---- Port #%i: Device Disconnection      ----", UsbPort);
                DBGPRINTF("   (total=%i)", TotalConnectionCount);
                DBGPRINTF("\r\n");
                
                ComputeNewPower_forUnusedPort(UsbPort);
                
                //put the Device IRQ in a clean state, when no cable
                Clear_All_pendingIRQ(UsbPort);
                
            }
            else //already disconnected
            {
                //do nothing
            }
        }
    }
}

void Single_TypeC_port(uint8_t UsbPort) //used to update the var: PortConnected_flag[UsbPort]
{
    static unsigned char DataRW[4];
    int Status;
    unsigned int RDO ;
    uint8_t Pdo_nb;
    
    // status is always given by ALERT_MANAGER()
    if (PD_status[UsbPort].Port_status.bitfield.CC_ATTACH_STATE !=0)  
    {
        I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,RDO_REG_STATUS ,(uint8_t *)&STUSB4700Register_Nego_RDO[UsbPort], 4 );
        
        if( STUSB4700Register_registered_RDO[UsbPort].d32 !=  STUSB4700Register_Nego_RDO[UsbPort].d32)
            PortConnected_flag[UsbPort]=0;
        PortConnected_flag[UsbPort]++;
        
        if (PortConnected_flag[UsbPort]== 1) // first time cable connection
        {
            Pdo_nb = Check_Contract(UsbPort);
        }
        else // polling PDO Changed without cable disconnection
        {
            PortConnected_flag[UsbPort]=2; 
        }
    }  
    else //Port Disconnected
    {
        if ( PortConnected_flag[UsbPort]!=0 )  //first disconnection action port i 
        {
            //Update_all_PDO_with_new_budget(OtherPort,45, 1.25);         
            DBGPRINTF("\r\nDisconnection UsbPort %i\r\n",UsbPort);
            
            PortConnected_flag[UsbPort]=0;
        } 
    }
    STUSB4700Register_registered_RDO[UsbPort].d32 =  STUSB4700Register_Nego_RDO[UsbPort].d32;
    
}

int push_button_Get_Sink_Cap(uint8_t UsbPort)
{
    //if (PD_status[UsbPort].Port_status.bitfield.CC_ATTACH_STATE !=0)  
    if(CheckCableAttached(UsbPort) >= 1) //USB-C cable attached
    {
        static unsigned char DataRW[4];
        int Status;
        int timeout = 0;
        DataRW[0] = 0x0C;
        Status = I2cWrite(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,0x1A ,&DataRW[0], 1 ); // unmask port status alarm 
        
        
        while (( PDO_FROM_SNK_Num[UsbPort] == 0 ) && (timeout != 1)); //wait message received from SINK during IRQ
        
        Print_PDO_FROM_SNK(UsbPort);
    }
    else
    {
        //Error, cable non attached.
        printf("Not Attached\r\n");
        return -1;
    }
    
    return 0; //OK
    
}

void push_button_TypeA_emulation(uint8_t UsbPort)
{
    DBGPRINTF("r\n--- User button pushed ---");	
    
    switch(PB_press )
    {
    case 1 :
        {
            Update_all_PDO_with_new_budget(UsbPort,23, 1.00);
            
            DBGPRINTF("\r\n--- STD-A port #1 : CONNECTION detected");
            DBGPRINTF("\r\n--- STD-A port #1 : charging  - 7.5W allocated");
            DBGPRINTF("\r\n--- STD-A port #2 : open      - 2.5W allocated");                      
            Print_PDO(UsbPort);
            
            PB_press = PB_press+1;
        } break;      
        
    case 2 :
        {
            Update_all_PDO_with_new_budget(UsbPort,15, 1.00);
            
            DBGPRINTF("\r\n--- STD-A port #2 : CONNECTION detected");
            DBGPRINTF("\r\n--- STD-A port #1 : charging - 7.5W allocated");
            DBGPRINTF("\r\n--- STD-A port #2 : charging - 7.5W allocated");  
            Print_PDO(UsbPort);
            
            PB_press = PB_press+1;        
        } break;
        
    case 3 :
        {
            Update_all_PDO_with_new_budget(UsbPort,23, 1.00);
            
            DBGPRINTF("\r\n--- STD-A port #1 : DISCONNECTION detected");
            DBGPRINTF("\r\n--- STD-A port #1 : open      - 2.5W allocated");
            DBGPRINTF("\r\n--- STD-A port #2 : charging  - 7.5W allocated"); 
            Print_PDO(UsbPort);
            
            PB_press = PB_press+1;
        } break;
        
    case 4 :
        {
            Update_all_PDO_with_new_budget(UsbPort,30, 1.00);
            
            DBGPRINTF("\r\n--- STD-A port #2 : DISCONNECTION detected");
            DBGPRINTF("\r\n--- STD-A port #1 : open      - 2.5W allocated");
            DBGPRINTF("\r\n--- STD-A port #2 : open      - 2.5W allocated");
            Print_PDO(UsbPort);
            
            PB_press = 1;
        } break;  
        
    }
    
}


void push_button_circular(uint8_t UsbPort)
{
    static int PB_press=1;
    
    DBGPRINTF("\r\n--- User button pushed circular ---\r\n");
    
    switch(PB_press )
    {
    case 0 :
        {
            Update_all_PDO_with_new_budget(UsbPort,45, 1.25);
            Print_PDO(UsbPort);
            PB_press = PB_press+1;        
        } break;      
        
    case 1 :
        {
            Update_all_PDO_with_new_budget(UsbPort,30, 1.25);
            Print_PDO(UsbPort);
            PB_press = PB_press+1;        
        } break;
        
    case 2 :
        {
            Update_all_PDO_with_new_budget(UsbPort,20, 1.25);
            Print_PDO(UsbPort);
            PB_press = PB_press+1;
        } break;
        
    case 3 :
        {
            Update_all_PDO_with_new_budget(UsbPort,15, 1.25);
            Print_PDO(UsbPort);
            PB_press = PB_press+1;
        } break;     
        
    case 4 :
        {
            Update_all_PDO_with_new_budget(UsbPort,10, 1);
            Print_PDO(UsbPort);
            PB_press = PB_press+1;
        } break;   
        
    case 5 :
        {
            Update_all_PDO_with_new_budget(UsbPort,60, 1.25);
            Print_PDO(UsbPort);
            PB_press = 0;
        } break;         
    }
    
}

void Timer_circular_ChangePdoNb(uint8_t UsbPort)
{
    int Status = 0;
    uint8_t Buffer[5];
    
    if (PD_status[UsbPort].Port_status.bitfield.CC_ATTACH_STATE !=0)
    {
        
        //    printf("\r\n--- Timer Action ---\r\n");
        
        switch(Time_elapse ) 
        {
        case 1:
            {
                /*update PDO num to 3 then Reset */
                Buffer[0] = 3 << 5 ;
                Status = I2cWrite(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,STUSBADDR_DPM_PDO_NUMB,&Buffer[0],1 );
                if (Flag_count)
                    Time_elapse ++ ;
                else 
                    Time_elapse -- ;  
                
            }
            break;
        case 2:
            {
                /*update PDO num to 2 then Reset */
                Buffer[0] = 2 << 5 ;
                Status = I2cWrite(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,STUSBADDR_DPM_PDO_NUMB,&Buffer[0],1 );
                if (Flag_count)
                    Time_elapse ++ ;
                else 
                    Time_elapse -- ;  
                
            }
            break; 
        case 3:
            {
                /*update PDO num to 2 then Reset */
                Buffer[0] = 1 << 5 ;
                Status = I2cWrite(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,STUSBADDR_DPM_PDO_NUMB,&Buffer[0],1 );
                if (Flag_count)
                    Time_elapse ++ ;
                else 
                    Time_elapse -- ;  
                
            }
            break; 
            
        default:
            {
                /*update PDO num to 2 then Reset */
                Buffer[0] = 3 << 5 ;
                Status = I2cWrite(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,STUSBADDR_DPM_PDO_NUMB,&Buffer[0],1 ); 
                Time_elapse = 1;
            }
            break;     
        }
        
        
        if ( Time_elapse == 1)
            Flag_count = 1 ;
        if ( Time_elapse == 3)
            Flag_count = 0 ;
        
        /* send cmd source cap update to force nego */
        Buffer[0] = SRC_capa_changed ;
        Status = I2cWrite(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,PD_COMMAND,&Buffer[0],1 ); 
    }
    
    //Timer_Action_Flag[UsbPort]=0;
    
}
