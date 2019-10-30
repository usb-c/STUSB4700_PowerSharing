#include "i2c.h"

extern I2C_HandleTypeDef *hi2c[2];	
extern unsigned int I2cDeviceID_7bit;
extern unsigned int Address;
extern unsigned int AddressSize = I2C_MEMADD_SIZE_8BIT;

void CUST_EnterWriteMode(uint8_t I2cBus,unsigned char ErasedSector)
{
	unsigned char Buffer[10];



	Buffer[0]=FTP_CUST_PASSWORD;  // Set TM
	I2C_Write_USB_PD(I2cBus,FTP_CUST_PASSWORD_REG,1,Buffer);

	Buffer[0]= 0 ;   // this register must be NULL for Partial Errase feature
	I2C_Write_USB_PD(I2cBus,TX_DATA_OBJ1_0,1,Buffer);

	Buffer[0]=FTP_CUST_PWR | FTP_CUST_RST_N; //Set PWR and RST_N bits
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer);
	Buffer[0]=((ErasedSector << 3) & FTP_CUST_SER) | ( WRITE_SER & FTP_CUST_OPCODE) ;  // Load FF to erase all sectors of FTP
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_1,1,Buffer); 

	Buffer[0]=FTP_CUST_PWR | FTP_CUST_RST_N | FTP_CUST_REQ ;  // Load SER opcode //3.2µs
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer);
	do 
	{
	I2C_Read_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer); 
	}
	while(Buffer[0] & FTP_CUST_REQ); 
	Buffer[0]=  SOFT_PROG_SECTOR & FTP_CUST_OPCODE ;  // Load FF to erase all sectors of FTP
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_1,1,Buffer); 

	Buffer[0]=FTP_CUST_PWR | FTP_CUST_RST_N | FTP_CUST_REQ ;  // Load SER opcode //3.2µs
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer);

	do 
	{
	I2C_Read_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer); 
	}
	while(Buffer[0] & FTP_CUST_REQ);
	Buffer[0]= ERASE_SECTOR & FTP_CUST_OPCODE ;  // Load FF to erase all sectors of FTP
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_1,1,Buffer); 

	Buffer[0]=FTP_CUST_PWR | FTP_CUST_RST_N | FTP_CUST_REQ ;  // Load SER opcode //3.2µs
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer);

	do 
	{
	I2C_Read_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer); 
	}
	while(Buffer[0] & FTP_CUST_REQ);	



}

void CUST_EnterReadMode(uint8_t I2cBus)
{
	unsigned char Buffer[10];

	//UIP_I2cRead(1,  0, 8, 1, &Buffer ,8, NULL ); //cut1 related code

//	I2C_Read_USB_PD(I2cBus,0,1,Buffer); //cut1 related code
	Buffer[0]=FTP_CUST_PASSWORD;  // Set TM
	I2C_Write_USB_PD(I2cBus,FTP_CUST_PASSWORD_REG,1,Buffer);

//	Sleep(4); // wait

}

void CUST_ReadSector(uint8_t I2cBus,char SectorNum, unsigned char *SectorData)
{
	unsigned char Buffer[10];


	//read sector
	Buffer[0]= (SectorNum & FTP_CUST_SECT) | FTP_CUST_PWR |FTP_CUST_RST_N ;
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer);

	Buffer[0]= (READ & FTP_CUST_OPCODE);
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_1,1,Buffer);
	Buffer[0]= (SectorNum & FTP_CUST_SECT) |FTP_CUST_PWR |FTP_CUST_RST_N | FTP_CUST_REQ;
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer);
	do 
	{
	I2C_Read_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer); 
	}
	while(Buffer[0] & FTP_CUST_REQ);
	I2C_Read_USB_PD(I2cBus,TX_DATA_OBJ1_0,8,&SectorData[0]); 

	Buffer[0] = 0 ;
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer);

		
}

void CUST_WriteSector(uint8_t I2cBus,char SectorNum, unsigned char *SectorData)
{
	unsigned char Buffer[10];
//	int i;

	I2C_Read_USB_PD(I2cBus,TX_DATA_OBJ1_0,8,Buffer);

	I2C_Write_USB_PD(I2cBus,TX_DATA_OBJ1_0,8,SectorData);
	
	I2C_Read_USB_PD(I2cBus,TX_DATA_OBJ1_0,8,Buffer);
	Buffer[0]=FTP_CUST_PWR | FTP_CUST_RST_N; //Set PWR and RST_N bits
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer);
	Buffer[0]= (WRITE_PL & FTP_CUST_OPCODE); //Set Write to PL Opcode
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_1,1,Buffer);
	Buffer[0]=FTP_CUST_PWR |FTP_CUST_RST_N | FTP_CUST_REQ;
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer);

	do 
	{
	I2C_Read_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer); 
	}		 
	while(Buffer[0] & FTP_CUST_REQ) ;
	

	Buffer[0]= (PROG_SECTOR & FTP_CUST_OPCODE); //=0xCA;
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_1,1,Buffer);
	Buffer[0]=(SectorNum & FTP_CUST_SECT) |FTP_CUST_PWR |FTP_CUST_RST_N | FTP_CUST_REQ;
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer);
	do 
	{
	I2C_Read_USB_PD(I2cBus,FTP_CTRL_0,1,Buffer); 
	}
	while(Buffer[0] & FTP_CUST_REQ) ;

}

void CUST_ExitTestMode(uint8_t I2cBus)
{
	unsigned char Buffer[10];

	Buffer[0]= FTP_CUST_RST_N; Buffer[1]=0x00;  // clear register 45
	I2C_Write_USB_PD(I2cBus,FTP_CTRL_0,2,Buffer);
	Buffer[0]=0x00;  // Set TM
	I2C_Write_USB_PD(I2cBus,FTP_CUST_PASSWORD_REG,1,Buffer);

}



