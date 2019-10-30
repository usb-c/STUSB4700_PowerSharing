#include "Hardware_configuration.h" //for I2CBUS_MAX, USBPORT_MAX
#include "USB_PD_defines_stusb4700.h"
#include "stusb4700_defines_2.h"
#include "i2c.h"
#include "I2C_ReadWrite.h"

#include "USB_PD_core.h" //for USB_PD_I2C_PORT
#include "stm32f0xx_hal.h" //for HAL_Delay(100);


extern USB_PD_I2C_PORT STUSB47DeviceConf[USBPORT_MAX];

/*  ----------------- SEND_SW_Reset -------------------------------------------
------------------------------------------------------------------------------*/
void STUSB4700_SEND_SW_Reset(uint8_t UsbPort)
{
    uint8_t DataRW;
    int Status;
    uint16_t Address;
    
    //send Software RESET
    DataRW = RESET_SW_EN;
    Address = STUSBADDR_SW_RESET;
    Status = I2cWrite(STUSB47DeviceConf[UsbPort].I2cBus, STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,Address ,&DataRW, 1) ;
    
#ifdef CONSOLE_PRINTF
    printf("Software RESET: ON\r\n");
#endif 
    
    HAL_Delay(100);
    
    //Release Software RESET
    DataRW = RESET_SW_DIS;
    Address = STUSBADDR_SW_RESET;
    Status = I2cWrite(STUSB47DeviceConf[UsbPort].I2cBus,STUSB47DeviceConf[UsbPort].I2cDeviceID_7bit ,Address ,&DataRW, 1) ;
    
#ifdef CONSOLE_PRINTF
    printf("Software RESET: OFF \r\n");
#endif
    
    //  HAL_Delay(500);
}


#if 0
uint8_t PDO_SNK[USBPORT_MAX];

/*  ----------------- GET_SINK_CAP -------------------------------------------
------------------------------------------------------------------------------*/
void Get_sink_cap(uint8_t Port)
{
    int Status;
    static unsigned char New_CMD;
    static unsigned char Header[2];
    Header[0]= 0;
    static unsigned char DataRW[4];	
    DataRW[0]= 0;
    unsigned char Current_ADD_PDO;
    uint32_t PDO ;
    uint16_t Address;
    
    float OPVoltage, MaxVoltage, MinVoltage, OPCurrent, OPPower; 
    int i, j, nbofPDO; // nbofPDO is the number of PDOblects from sink capabilities
    
    // CHECK 
    I2cRead(STUSB47DeviceConf[Port].I2cBus ,STUSB47DeviceConf[Port].I2cDeviceID_7bit ,0x16 ,&Header[0], 1);
    //printf("\n\r PRL STATUS 0x%x \n\r", Header[0]);   // 10001100  
    
    // SEND GET_SINK_CAP command
    Address = PD_COMMAND;
    //New_CMD = (unsigned char) (SRC_get_sink_cap & 0x000000FF);
    New_CMD = 0x0C;
    
    Status = I2C_Write_USB_PD(STUSB47DeviceConf[Port].I2cBus ,STUSB47DeviceConf[Port].I2cDeviceID_7bit, Address ,&New_CMD, 1 ) ;
    if (Status != HAL_I2C_ERROR_NONE) return;              
    
    //I2cRead(STUSB47DeviceConf[Port].I2cBus ,STUSB47DeviceConf[Port].I2cDeviceID_7bit ,0x16 ,&Header[0], 1 );
    // DOES NOT REMOVE otherwise code doesn't work ??????????????????????????????????????????
    printf("\n\r WAIT TO BE DEBUGGED");
    
    // CHECK GET_SINK_CAP message sent
    do {
        I2cRead(STUSB47DeviceConf[Port].I2cBus ,STUSB47DeviceConf[Port].I2cDeviceID_7bit ,0x16 ,&Header[0], 1 );
    } while( (Header[0] & 0x08) != 0x08 );  
    printf("\n\r---- Port #%i: GET_SINK_CAP message sent ----", Port);
    
    // WAIT UNTIL MESSAGE RECEIVED
    if ((Header[0] & 0x04) != 0x04)
    {
        do {I2cRead(STUSB47DeviceConf[Port].I2cBus ,STUSB47DeviceConf[Port].I2cDeviceID_7bit ,0x16 ,&Header[0], 1 );}
        while( (Header[0] & 0x04) != 0x04 ); 
    }
#ifdef CONSOLE_PRINTF  
    printf("\n\r MESSAGE RECEIVED 0x%x", Header[0]);
#endif
    
    // READ HEADER
    Address = RX_HEADER ;
    I2cRead(STUSB47DeviceConf[Port].I2cBus ,STUSB47DeviceConf[Port].I2cDeviceID_7bit, Address ,&Header[0], 2 );
    
#ifdef CONSOLE_PRINTF
    {
        printf("\n\r READ HEADER0: 0x%x", Header[0]);
        printf("\n\r READ HEADER1: 0x%x", Header[1]);
    }
#endif
    
    
    if ( (Header[0] & 0xF) == 4) // test if data message of sink capabilities
    {
        nbofPDO = ((Header[1] & 0x70) >> 4); //number of PDobject
        
#ifdef PRINTF  
        printf("\n\r SINK capabilitites received: #%i",nbofPDO);
#endif        
        
        Current_ADD_PDO = RX_DATA_OBJ1;
        Address = Current_ADD_PDO;
        I2cRead(STUSB47DeviceConf[Port].I2cBus ,STUSB47DeviceConf[Port].I2cDeviceID_7bit, Address ,&DataRW[0], 4*nbofPDO );
        
        j=0;
        for (i = 0; i < nbofPDO; i++)
        {
            
            PDO = (uint32_t )( DataRW[j] +(DataRW[j+1]<<8)+(DataRW[j+2]<<16)+(DataRW[j+3]<<24));
            
            // STORE PDO(i)
            PDO_SNK[Port][i].supply_type = (PDO & 0xC0000000) >> 30 ;
            
#ifdef PRINTF  
            printf("\n\r PDO_SNK%i: ",i);
#endif           
            
            if (PDO_SNK[Port][i].supply_type == 0)  // Fixed Supply
            {
                PDO_SNK[Port][i].Role             = (PDO & 0x20000000) >> 29 ;
                PDO_SNK[Port][i].suspend          = (PDO & 0x10000000) >> 28 ;
                PDO_SNK[Port][i].externaly_powered= (PDO & 0x08000000) >> 27 ;
                PDO_SNK[Port][i].communication    = (PDO & 0x04000000) >> 26 ;
                PDO_SNK[Port][i].data_role_swap   = (PDO & 0x02000000) >> 25 ;
                PDO_SNK[Port][i].Max_Current      = (PDO & 0x00300000) >> 20  ;
                PDO_SNK[Port][i].Voltage          = (PDO & 0x000FFC00) >> 10 ;
                PDO_SNK[Port][i].Current          = (PDO & 0x000003FF) ;   
                
#ifdef PRINTF  
                OPVoltage = (PDO_SNK[Port][i].Voltage) * 0.05; // getting voltage to multiply by 50mV units
                OPCurrent = (PDO_SNK[Port][i].Current) * 0.01; // getting current to multiply to 10mA units        
                
                printf(" FIXED   : %.2fV; %.2fA",OPVoltage, OPCurrent);
                if (PDO_SNK[Port][i].Role ==1)
                { 
                    printf(" (DRP capable)");
                } 
#endif
            }
            else if (PDO_SNK[Port][i].supply_type == 1) // Battery Supply	
            {
                PDO_SNK[Port][i].Max_Voltage      = (PDO & 0x3FF00000) >> 20 ;
                PDO_SNK[Port][i].Min_Voltage      = (PDO & 0x000FFC00) >> 10 ;
                PDO_SNK[Port][i].Current          = (PDO & 0x000003FF) ; 
                
#ifdef PRINTF            
                MaxVoltage = (PDO_SNK[Port][i].Max_Voltage) * 0.05; // getting maximum voltage to multiply by 50mV units
                MinVoltage = (PDO_SNK[Port][i].Min_Voltage) * 0.05; // getting minimum voltage to multiply by 50mV units
                OPCurrent  = (PDO_SNK[Port][i].Current)  * 0.01;    // getting current to multiply to 10mA units
                printf(" BATTERY : %.2fV-%.2fV; %.2fA",MinVoltage, MaxVoltage, OPCurrent);
#endif            
            }
            else if (PDO_SNK[Port][i].supply_type == 2) // Variable Suply
            {
                PDO_SNK[Port][i].Max_Voltage      = (PDO & 0x3FF00000) >> 20 ;
                PDO_SNK[Port][i].Min_Voltage      = (PDO & 0x000FFC00) >> 10 ;
                PDO_SNK[Port][i].Power            = (PDO & 0x000003FF) ; 
                
#ifdef PRINTF            
                MaxVoltage = (PDO_SNK[Port][i].Max_Voltage) * 0.05; // getting maximum voltage to multiply by 50mV units
                MinVoltage = (PDO_SNK[Port][i].Min_Voltage) * 0.05; // getting minimum voltage to multiply by 50mV units
                OPPower    = (PDO_SNK[Port][i].Power) * 0.25;       // Getting Power to multiply by 250mW units
                printf(" VARIABLE: %.2fV-%.2fV; %iW",MinVoltage, MaxVoltage, OPCurrent);
#endif              
            }
            j=j+4;
        }    
    }
}

#endif
