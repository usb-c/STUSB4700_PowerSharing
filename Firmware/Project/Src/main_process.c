
/* Includes ------------------------------------------------------------------*/
#include "main_process.h"
#include "stm32f0xx_hal.h" //for uint16_t
#include "I2C.h" //for hi2c1
#include "I2C_ReadWrite.h" //for I2cRead
#include "gpio_i2c_pullup.h"
#include "UART_process.h"
#include "timer_process.h"

#include "USB_PD_defines_stusb4700.h"
#include "stusb4700_defines_2.h"
#include "USB_PD_user.h" //for USB_PD_I2C_PORT
#include "utils.h"
#include "global_var_IRQ.h"

#include "Hardware_configuration.h" //for I2CBUS_MAX, USBPORT_MAX
#include "Software_configuration.h"
#include "printf_debug.h"




extern int UART_start_message(void);

int Print_STUSB47Version();
int Print_PDO_config();
int Write_New_PDO();
int PostProcess_PD_Messages(uint8_t UsbPort);

USB_PD_I2C_PORT STUSB47DeviceConf[USBPORT_MAX];

volatile uint16_t USB_PD_Interupt_Flag[USBPORT_MAX] = {0}; // all elements to 0
volatile uint8_t  USB_PD_Resquest_received_Flag[USBPORT_MAX] = {0};
volatile uint8_t  Timer_Action_Flag[USBPORT_MAX] = {0};
volatile uint8_t  PB_Action_Flag[USBPORT_MAX] = {0};

float PowerAllocated[USBPORT_MAX] = {0};

extern STUSB_RDO_REG_STATUS_RegTypeDef STUSB4700Register_Nego_RDO[USBPORT_MAX];
static int myDevicesFoundCount = 0;

extern volatile uint8_t DoActionAfterMsgReceived_GoodCrc[USBPORT_MAX];



int GetDeviceFoundCount(void)
{
    return myDevicesFoundCount;
}

void SetDeviceFoundCount(int Count)
{
    myDevicesFoundCount = Count;
}




int main_process(void)
{
    unsigned int i = 0;
    
    int Status;
    uint8_t UsbPort;
    int Pdo_nb;
    int I2cBus;
    
    unsigned char DataRW[10];
    
    //int myDevicesFoundCount = 0;
    uint8_t I2cDeviceID_7bit ;
    extern I2C_HandleTypeDef *hi2c[I2CBUS_MAX];
    
    
    hi2c[0]= &hi2c1;
    if(I2CBUS_MAX > 1) hi2c[1]= &hi2c2;
    
#ifdef CONSOLE_PRINTF
    UART_start_message();
#endif
    UART_Rx_init(); //for Terminal commands
    
    
    Timer_Start();
    
#ifdef CONSOLE_PRINTF
    printf("--- Power: \r\n");
    printf("# Total system power = %d W\r\n", TOTAL_SYSTEM_POWER);
    printf("# Max power per Usb Port = %d W\r\n", MAX_POWER_PER_USBPORT);
    printf("# Min power per Usb Port = %d W\r\n", SMALLEST_POWER_PER_USBPORT);
    printf("\r\n");
#endif //CONSOLE_PRINTF
    
    
    //Hardware-Reset STUSB4700
    HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(Reset_GPIO_Port, Reset_Pin, GPIO_PIN_RESET);
    HAL_Delay(100);
    
    
    DBGPRINTF("--- Checking External pull-up resistors for I2C bus...\r\n");
    for(I2cBus = 0; I2cBus < I2CBUS_MAX; I2cBus++)
    {
        Gpio_I2C_external_pullup_GetStatus(I2cBus);
    }
    DBGPRINTF("\r\n");
    
#ifdef CONSOLE_PRINTF
    
    printf("--- Scanning devices on I2C bus:\r\n");
    
    for(I2cBus = 0; I2cBus < I2CBUS_MAX; I2cBus++)
    {
        for (I2cDeviceID_7bit = 2 ;I2cDeviceID_7bit <= 128 ; I2cDeviceID_7bit++)
        {
            uint16_t I2cDeviceID_8bit = (I2cDeviceID_7bit << 1);
            uint32_t Trials = 1;
            if (HAL_I2C_IsDeviceReady(hi2c[I2cBus], I2cDeviceID_8bit, Trials, 1000) == HAL_OK)
            {
                STUSB47DeviceConf[myDevicesFoundCount].I2cBus = I2cBus ;
                STUSB47DeviceConf[myDevicesFoundCount].I2cDeviceID_7bit = I2cDeviceID_7bit;
                printf("Bus: %d,  ", I2cBus);
                printf("I2c_DeviceID_7bit: 0x%X \r\n", I2cDeviceID_7bit);
                
                myDevicesFoundCount++;	
            }
        }
    }
    
    printf("Nr Devices found: %d \r\n", myDevicesFoundCount);
    
#endif //CONSOLE_PRINTF
    
    if (myDevicesFoundCount == 0) //Error
    {
        printf("Error, no STUSB4700 IC detected\r\n");
        
        _Error_Handler(__FILE__, __LINE__);
    }
    
    if (myDevicesFoundCount > USBPORT_MAX) //Should not happen, but just in case
    {
        myDevicesFoundCount = USBPORT_MAX;
    }
    
    
    Print_STUSB47Version();
    //Print_PDO_config();
    
    
    int CableAttachedStatus = 0;
    
    {
        for(UsbPort = 0; UsbPort < myDevicesFoundCount; UsbPort++)
        {
            
            printf("\r\n=== CABLE: ");
            
            if(CableAttachedStatus = CheckCableAttached(UsbPort) >= 1) //USB-C cable attached
            {
                if( CableAttachedStatus == 1)
                {
                    printf("Attached [CC1] ");
                }
                else if( CableAttachedStatus == 2)
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
    }
    
    
    for(UsbPort = 0; UsbPort < myDevicesFoundCount; UsbPort++)
    {
        usb_pd_init(UsbPort);
        
        Print_PDO(UsbPort);
    }
    
    for(UsbPort = 0; UsbPort < myDevicesFoundCount; UsbPort++)
    {
        Pdo_nb = Check_Contract(UsbPort);
    }
    
    Write_New_PDO();
    
    for(UsbPort = 0; UsbPort < myDevicesFoundCount; UsbPort++)
    {
        PrintAlertStatusRegister(UsbPort);
    }
    
    
#if CONSOLE_PRINTF
    printf("\r\n");
    printf("--- MainLoop \r\n");
#endif
    
    /* Infinite loop */
    while (1)
    {
        UART_CheckReceivedMessage(); //to control the demo firmware from the terminal
        
#if CONSOLE_PRINTF
        if(HAL_Delay_nonblocking(5000) == 1)
        {
            //printf("~%d", i++); //running counter
            printf("~"); //running counter
        }
#endif
        

        for(UsbPort = 0; (UsbPort < myDevicesFoundCount) && (UsbPort < USBPORT_MAX) ; UsbPort++)
        {
            
#if defined(INTERRUPT_MODE)
            if (USB_PD_Interupt_Flag[UsbPort] != 0)
            {
                ALARM_MANAGEMENT(UsbPort); //it updates the STATUS variable
            }
            
#elif defined(POLLING_MODE)
            if(HAL_Delay_nonblocking3(100) == 1)
            {
                MultiPort_PowerSharing_polling(UsbPort);
            }
#endif
            
            
            
#if defined(DEMO_TIMER)
            
            if( Timer_Action_Flag[UsbPort] == 1) //Timer occurs every 2s
            {
                Single_TypeC_port(UsbPort); //polling of registers
                //Dual_TypeC_power_sharing(UsbPort);
                
                //Timer_circular_ChangePdoNb(UsbPort); //Change PDO number from SOURCE (SRC_Capa_Change)
                
                Timer_Action_Flag[UsbPort]=0;
            }
            
#elif defined(DEMO_PUSH_BUTTON)
            if( PB_Action_Flag[UsbPort] == 1)
            {
                //push_button_Get_Sink_Cap(UsbPort);
                
                push_button_circular(UsbPort);
                
                PB_Action_Flag[UsbPort]=0; 
            } 
#endif
            
            
            PostProcess_UsbEvents(UsbPort);
            
            PostProcess_PD_Messages(UsbPort);
            
#if DBG
            if(HAL_Delay_nonblocking2(2000) == 1)
            {
                for(UsbPort = 0; (UsbPort < myDevicesFoundCount) && (UsbPort < USBPORT_MAX) ; UsbPort++)
                {
                    uint16_t Address = STUSBADDR_ALERT_STATUS_1; 
                    uint8_t DataR[2];
                    static uint8_t DataR_saved = 0xFF;
                    
                    Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, Address, &DataR[0], 1 ); if(Status != I2C_OK) {printf("I2cRead error\r\n"); return I2C_ERR; };
                    Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, Address, &DataR[1], 1 ); if(Status != I2C_OK) {printf("I2cRead error\r\n"); return I2C_ERR; };
                    
                    
                    if(DataR[0] != DataR_saved)
                    {
                        printf("New status register: 0x%X\r\n", DataR[0]);
                        printf("New status register: 0x%X\r\n", DataR[1]);
                        DataR_saved = DataR[0];
                        
                        PrintAlertStatusRegister();
                    }
                }
            }
#endif  
            
        }
    }
    
}

int PostProcess_PD_Messages(uint8_t UsbPort)
{
    unsigned char DataRW[10];
    int Status;
    
#if 1
    if (USB_PD_Resquest_received_Flag[UsbPort] == 1 )
        /*read PE_state register to check transition */
        do 
        {
            uint16_t Address = STUSBADDR_PE_FSM; 
            I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, Address ,&DataRW[0], 1 );  if(Status != I2C_OK) return I2C_ERR;
            
            if ( DataRW[0] == PE_SRC_TRANSITION_SUPPLY_3 )
            {
                // here you can set the Current control put the code here 
#ifdef CONSOLE_PRINTF
                printf("\r\nnew contract set %i ,max current :%i", STUSB4700Register_Nego_RDO[UsbPort].bitfield.Object_Pos, STUSB4700Register_Nego_RDO[UsbPort].bitfield.MaxCurrent);
#endif                 
                USB_PD_Resquest_received_Flag[UsbPort] = 0;
                
            }
            if ( DataRW[0] == PE_SRC_READY )
            {
                // here we are in explicit contrat , Power nego is over 
                USB_PD_Resquest_received_Flag[UsbPort] = 0;
                
            }
            
        }while (USB_PD_Resquest_received_Flag[UsbPort] == 1 );
#endif
    
    
    
#if 0
    //if( UsbPortStatusChanged || PD_message_ongoing != 0)
    if(DoActionAfterMsgReceived_GoodCrc[UsbPort] != 0)
    {
        //wait the new PDO negociated are ready to read
        int Timeout = 0;
        
        //wait PD communication finished
        do
        {
            uint16_t Address = STUSBADDR_PE_FSM; 
            I2cRead(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, Address ,&DataRW[0], 1 ); 
        }
        while( ((DataRW[0] != PE_SRC_READY) && (DataRW[0] != PE_SRC_DISABLED)) && (DataRW[0] != PE_INIT) && (Timeout == 0) );
        
        if(Timeout == 1)
            printf("USbPort %d: Message Timeout \r\n", UsbPort);
        else
            printf("NEW PDO negociated on USbPort %d\r\n", UsbPort);
        
        int Pdo_nb = Get_PD_Contract(UsbPort, &PowerAllocated[UsbPort]);
        
        DoActionAfterMsgReceived_GoodCrc[UsbPort] = 0; //clear
    }
#endif
    
    return 0;
}

int Print_STUSB47Version()
{
    int Status;
    
#ifdef CONSOLE_PRINTF
    
    uint8_t SiliconCut;
    printf("\r\n");
    printf("--- Checking USB-PD device silicon version: \r\n");
    
    for(int UsbPort = 0; UsbPort < myDevicesFoundCount; UsbPort++)
    {
        //HAL_StatusTypeDef I2cRead(uint8_t Port, uint16_t I2cDeviceID_7bit, uint16_t Address, uint8_t *pDataR , uint16_t ByteCount)
        uint16_t ByteCount = 1;
        Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, STUSB47ADDR_DEVICE_ID, &SiliconCut, ByteCount );
        
        if(Status == HAL_OK)
        {
            char VersionString[32];
            
            switch (SiliconCut)
            {
            case 0x0F:
                sprintf(VersionString, "STUSB4700");
                break;
            case 0x8F:
                sprintf(VersionString, "STUSB4710");
                break;
            case 0x13:
                sprintf(VersionString, "STUSB4700-A");
                break;
            case 0x93:
                sprintf(VersionString, "STUSB4710-A");
                break;
            default:
                sprintf(VersionString, "UNKNOWN_Device");
                break;
            }
            
            printf("UsbPort %i, Cut=0x%X  -> %s \r\n",UsbPort, SiliconCut, VersionString);
        }
        else printf("UsbPort %i: I2c Error\r\n", UsbPort);
    }
    
    printf("\r\n");
    
    for(int UsbPort = 0; UsbPort < myDevicesFoundCount; UsbPort++)
    {
        uint16_t ByteCount = 1;
        uint8_t Data8;
        Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, STUSB47ADDR_DEVICE_ID, &Data8, ByteCount ); if(Status != I2C_OK) return I2C_ERR;
        
        if(Status == HAL_OK)
        {            
            uint8_t Value;
            uint16_t Family;
            
            Value = (Data8 >> STUSB47SHIF_4710_NOT_4700) & STUSB47MASK_4710_NOT_4700;           
            
            if(Value == 1)
            {
                Family = 4710;
            }
            else
            {
                Family = 4700;
            }
            
            printf("UsbPort %i, Family=%d_serie, ", UsbPort, Family);
            
            Value = (Data8 >> STUSB47SHIF_ID) & STUSB47MASK_ID;
            printf("ID_CUT=0x%X, ",Value);
            
            Value = (Data8 >> STUSB47SHIF_DEV_CUT) & STUSB47MASK_DEV_CUT;
            printf("DEV_CUT=0x%X \r\n",Value);
        }
        else printf("UsbPort %i: I2c Error\r\n", UsbPort);
    }
#endif //#ifdef CONSOLE_PRINTF
    
    return 0;
}

int Print_PDO_config()
{
    int Status;
    
    printf("\r\n");
    printf("--- Checking PDOs configuration: \r\n");
    
    for(int UsbPort = 0; UsbPort < myDevicesFoundCount; UsbPort++)
    {
        unsigned char Data8[4];
        unsigned long Data32;
        uint16_t ByteCount;
        int RegisterAddress;
        unsigned char SRC_PDO_Number = 0;
        
        RegisterAddress = STUSBADDR_DPM_PDO_NUMB;
        ByteCount = 1;
        Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, RegisterAddress, &Data8[0], ByteCount );
        if(Status == HAL_OK)
        {
            SRC_PDO_Number = (Data8[0] >> STUSB4700SHIFT_DPM_SRC_PDO_NUMB) & STUSB4700MASK_DPM_SRC_PDO_NUMB;
            printf("UsbPort %i, RegisterData: 0x%X  -> PDO Source number= %d \r\n", UsbPort, Data8[0], SRC_PDO_Number);
        }
        else printf("UsbPort %i: I2c Error\r\n", UsbPort);
        
        for(int PdoIdx=0; PdoIdx < SRC_PDO_Number; PdoIdx++)
        {
            ByteCount = 4;
            RegisterAddress = STUSBADDR_DPM_SRC_PDO1 + PdoIdx*ByteCount;
            
            Status = I2cRead(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit, RegisterAddress, &Data8[0], ByteCount );
            if(Status == HAL_OK)
            {
                Data32 = (Data8[3] << 24) | (Data8[2] << 16) | (Data8[1] << 8) | (Data8[0]);
                printf("    PDO%d: ", PdoIdx+1);
                //printf("Data32=0x%08X  -> ", Data32);
                
                int Voltage = (Data32 >> STUSB4700SHIFT_PDO_VOLTAGE) & STUSB4700MASK_PDO_VOLTAGE;
                printf("Voltage: %5d mV , ", Voltage*50);
                
                int Current = (Data32 >> STUSB4700SHIFT_PDO_MAXCURRENT) & STUSB4700MASK_PDO_MAXCURRENT;
                printf("Current: %4d mA ", Current*10);
                
                printf("\r\n");
            }
            else printf("UsbPort %i: I2c Error\r\n", UsbPort);
        }
    }
    
    return 0;
}

int Write_New_PDO()
{ 
    //Force default PDO setting at startup
    
    float PowerAllocated = 0;
    int Power_MAX_remaining = 0;
    float I_min_perPDO;
    float TotalSystemPower = TOTAL_SYSTEM_POWER; //60W demo
    
#ifdef CONSOLE_PRINTF
    printf("\n---- Overwritting default PDOs at startup...\r\n");
#endif
    
    //Power_MAX_remaining = (int)(TotalSystemPower - PowerAllocated); //45-15 = 30W
    I_min_perPDO = MIN_CURRENT_PER_PDO;
    
    for(int UsbPort = 0; (UsbPort < myDevicesFoundCount) && (UsbPort < USBPORT_MAX) ; UsbPort++)
    {
        Update_all_PDO_with_new_budget(UsbPort, MAX_POWER_PER_USBPORT, I_min_perPDO);
        
        //SetRpResistorValue(UsbPort, POWER_RP_LEGACY); //Type-C starts with 5V @ 100/500/900mA
        //SetRpResistorValue(UsbPort, POWER_RP_1_5A); //Type-C starts with 5V @ 1.5A
        SetRpResistorValue(UsbPort, POWER_RP_3A); //Type-C starts with 5V @ 3A
    }
    
#ifdef CONSOLE_PRINTF
    printf("\r\n---- Checking all PDOs:\r\n");
    
    for(int UsbPort = 0; (UsbPort < myDevicesFoundCount) && (UsbPort < USBPORT_MAX) ; UsbPort++)
    {
        Print_PDO(UsbPort);
    }
#endif
    
    return 0;
}
