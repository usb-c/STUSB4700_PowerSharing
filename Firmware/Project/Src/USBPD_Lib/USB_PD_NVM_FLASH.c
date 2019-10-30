

#include "USBPD_FLASH.h"


void nvm_flash(uint8_t Port)
{
	CUST_EnterWriteMode( SECTOR_0 |SECTOR_1  |SECTOR_2 |SECTOR_3  | SECTOR_4 );
	CUST_WriteSector(0,Sector0);
	CUST_WriteSector(1,Sector1);
	CUST_WriteSector(2,Sector2);
	CUST_WriteSector(3,Sector3);
	CUST_WriteSector(4,Sector4);

	CUST_ExitTestMode();
}