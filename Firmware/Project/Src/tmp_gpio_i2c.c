
{
        uint16_t ByteCount;
        int RegisterAddress;
        unsigned char Data8[4];
        unsigned char DataW;
        UsbPort = 0;
        
        
        RegisterAddress = STUSB4700ADDR_STM_PASSWORD;
        ByteCount = 1;
        Status = STM32_I2cRead(STUSB47_DeviceConf[UsbPort].I2cBus, STUSB47_DeviceConf[UsbPort].I2cDeviceID_7bit, RegisterAddress, &Data8[0], ByteCount );
        if(Status == HAL_OK){ printf("UsbPort %i, Data: 0x%X \r\n", UsbPort, Data8[0]); }
        
        DataW = ST_TESTMODE;
        Status = STM32_I2cWrite(STUSB47_DeviceConf[UsbPort].I2cBus, STUSB47_DeviceConf[UsbPort].I2cDeviceID_7bit, RegisterAddress, &DataW, ByteCount );
        if(Status == HAL_OK){ printf("I2cWrite OK \r\n"); }
        
        Status = STM32_I2cRead(STUSB47_DeviceConf[UsbPort].I2cBus, STUSB47_DeviceConf[UsbPort].I2cDeviceID_7bit, RegisterAddress, &Data8[0], ByteCount );
        if(Status == HAL_OK){ printf("UsbPort %i, Data: 0x%X \r\n", UsbPort, Data8[0]); }
        
        //-------------------------------------
        
        RegisterAddress = STUSB4700ADDR_CTRL_GPIO;
        ByteCount = 1;
        Status = STM32_I2cRead(STUSB47_DeviceConf[UsbPort].I2cBus, STUSB47_DeviceConf[UsbPort].I2cDeviceID_7bit, RegisterAddress, &Data8[0], ByteCount );
        if(Status == HAL_OK){ printf("UsbPort %i, Data: 0x%X \r\n", UsbPort, Data8[0]); }
        
        
        DataW = 0x00;
        Status = STM32_I2cWrite(STUSB47_DeviceConf[UsbPort].I2cBus, STUSB47_DeviceConf[UsbPort].I2cDeviceID_7bit, RegisterAddress, &DataW, ByteCount );
        if(Status == HAL_OK){ printf("I2cWrite OK \r\n"); }
        
        Status = STM32_I2cRead(STUSB47_DeviceConf[UsbPort].I2cBus, STUSB47_DeviceConf[UsbPort].I2cDeviceID_7bit, RegisterAddress, &Data8[0], ByteCount );
        if(Status == HAL_OK){ printf("UsbPort %i, Data: 0x%X \r\n", UsbPort, Data8[0]); }
        
        
    }
	