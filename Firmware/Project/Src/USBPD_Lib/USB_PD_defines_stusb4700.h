
// full Register MAP definition 

#ifndef __STUSB4700_DEFINES_H
#define __STUSB4700_DEFINES_H

#include <stdint.h> //for uint8_t, uint16_t, uint32_t


#define VENDOR_ID_LOW		0x00	// "USB-PD Structured Vendor ID. Assigned by the USB-IF to the Vendor."
#define VENDOR_ID_HIGH	   0x01	// "MUST be in the NVM programmed by customer because STMIcroelectronics does not want to keep track of the vendor."

#define PRODUCT_ID_LOW          0x02	//USB-PD Product ID identifying the product. 
#define PRODUCT_ID_HIGH         0x03	//MUST be in the NVM programmed by customer because STMIcroelectronics does not want to keep track of the vendor.

#define BCD_DEVICE_ID_LOW       0x04	//USB-PD bcdDevice ID: aversion number relevant to the release version of the product. 
#define BCD_DEVICE_ID_HIGH      0x05	//MUST be in the NVM programmed by customer because STMIcroelectronics does not want to keep track of the vendor.

#define BCD_TYPEC_REV_LOW       0x06	//USB-PD bcdDevice ID: a version number relevant to the release version of the product.
#define BCD_TYPEC_REV_HIGH      0x07

#define BCD_USBPD_REV_LOW       0x08
#define BCD_USBPD_REV_HIGH      0x09


#define STUSBADDR_ALERT_STATUS_1          0x0B // Interrupt register
typedef union
{
  uint8_t d8;
  struct
  {
    uint8_t PHY_STATUS_AL 		:       1; //bit-0 //Physical layer alert
    uint8_t PRT_STATUS_AL 		:       1; //bit-1 //Protocol layer alert
    uint8_t _Reserved_2 		:       1; //bit-2 //Policy Engine layer alert
    uint8_t PD_TYPEC_STATUS_AL 		:       1; //bit-3 //DPM layer alert
    uint8_t HW_FAULT_STATUS_AL 		:       1; //bit-4 //Change occurred on HW_FAULT_STATUS_TRANS register (Hardware fault from analog logic)
    uint8_t MONITORING_STATUS_AL 	:       1; //bit-5 //Change occurred on MONITORING_STATUS_TRANS register (see POWER_STATUS register).
    uint8_t PORT_CC_DETECTION_STATUS_AL 	:       1; //bit-6 //Change occurred on PORT_STATUS_TRANS register  (see CC_STATUS register).
    uint8_t HARD_RESET_AL 		:       1; //bit-7 //Hard Reset Message received
  } bitfield;
} STUSB_ALERT_STATUS_RegTypeDef;


#define STUSBADDR_ALERT_STATUS_MASK     0x0C // interrup MASK same mask as status shoud be used 
typedef union
{
  uint8_t d8;
  struct
  {
    uint8_t PHY_STATUS_AL_MASK          : 1; 
    uint8_t PRT_STATUS_AL_MASK          : 1; 
    uint8_t _Reserved_2                 : 1; 
    uint8_t PD_TYPEC_STATUS_AL_MASK     : 1;    
    uint8_t HW_FAULT_STATUS_AL_MASK     : 1;    
    uint8_t MONITORING_STATUS_AL_MASK   : 1;    
    uint8_t CC_DETECTION_STATUS_AL_MASK : 1;    
    uint8_t HARD_RESET_AL_MASK          : 1;
  } bitfield;
} STUSB_ALERT_STATUS_MASK_RegTypeDef;


#define STUSBADDR_PORT_STATUS_TRANS 0x0D
/*************************************************************************************************************
  * @brief:  	STUSB CC_DETECTION_STATUS_TRANS register Structure definition
  * @Address: 	0Dh
  * @Access: 	RC
  * @Note: 	This register indicates a bit value change has occurred in CC_DETECTION_STATUS register.
  ************************************************************************************************************/ 
typedef union
{
  uint8_t d8;
  struct
  {
    uint8_t ATTACH_STATE_TRANS 	        :       1; //Transition occurred on ATTACH_STATUS bit
    uint8_t _Reserved_1_7		:       7;
  } bitfield;
} STUSB_CC_DETECTION_STATUS_TRANS_RegTypeDef;


/*************************************************************************************************************
  * @brief:  	STUSB_CC_DETECTION_STATUS register Structure definition
  * @Address: 	0Eh
  * @Access: 	RO
  * @Note: 	This register provides current status of the connection detection and corresponding operation modes.
  ************************************************************************************************************/ 
#define STUSBADDR_PORT_STATUS 0x0E
typedef union
{
  uint8_t d8;
  struct
  {
    uint8_t CC_ATTACH_STATE			:       1;
    uint8_t CC_VCONN_SUPPLY_STATE 		:       1;    
    uint8_t CC_DATA_ROLE 			:       1;   
    uint8_t CC_POWER_ROLE 			:       1; 
    uint8_t START_UP_POWER_MODE 		:       1;    
    uint8_t CC_ATTACH_MODE 			:       3;
  } bitfield;
} STUSB_CC_DETECTION_STATUS_RegTypeDef;

#define                                 No_Devices 0
#define 				Debug_mode 3
#define 				Audio_mode 4
#define 				Distant_device_Sink 1 
#define 				Distant_device_Source 2
#define 				Powered_accessory 5


#define POWER_MODE 0x08
#define DATA_MODE 0x04
#define VCONN_MODE 0x02
#define ATTACH 0x01

/*************************************************************************************************************
  * @brief:  	STUSB_MONITORING_STATUS_TRANS register Structure definition
  * @Address: 	0Fh
  * @Access: 	RC
  * @Note: 	This register allows to: 
  *              - Alert about any change that occurs in MONITORING_STATUS register.
  *              - Manage specific USB PD Acknowledge commands
  *              - to manage Type-C state machine Acknowledge to USB PD Requests commands
  ************************************************************************************************************/ 

#define STUSBADDR_TYPEC_MONITORING_STATUS_0 0x0F
typedef union
{
  uint8_t d8;
  struct
  {    
    uint8_t VCONN_VALID_TRANS 	:       1;
    uint8_t VBUS_VALID_SNK_TRANS	:       1;
    uint8_t VBUS_VSAFE0V_TRANS 	        :       1;
    uint8_t VBUS_READY_TRANS 	        :       1;
    uint8_t HAND_CHECK		:       4;
  } bitfield;
} STUSB_MONITORING_STATUS_TRANS_RegTypeDef;

#define 	PD_TYPEC_HAND_CHECK 0xF0
#define 					cleared 0
#define 					PD_VCONN_POWERED_NOT_SUPPORTED_ACK 1
#define 					PD_PR_SWAP_RP_ASSERT_ACK 2
#define 					PD_PR_SWAP_RD_ASSERT_ACK 3
#define 					PD_DR_SWAP_PORT_CHANGE_2_DFP_ACK 4
#define 					PD_DR_SWAP_PORT_CHANGE_2_UFP_ACK 5
#define 					PD_VCONN_SWAP_TURN_ON_VCONN_ACK  6
#define 					PD_VCONN_SWAP_TURN_OFF_VCONN_ACK 7
#define 					PD_HARD_RESET_COMPLETE_ACK 8
#define 					PD_HARD_RESET_TURN_OFF_VCONN_ACK 9
#define                                         PD__HARD_RESET_PORT_CHANGE_2_DFP_ACK 0xA
#define                                         PD__HARD_RESET_PORT_CHANGE_2_UFP_ACK 0xB
#define 					PD_PR_SWAP_SNK_VBUS_OFF_ACK 0xC
#define 					PD_PR_SWAP_SRC_VBUS_OFF_ACK 0xD
#define 					PD_HARD_RESET_RECEIVED_ACK 0xE
#define 					PD_HARD_RESET_SEND_ACK 0xF

/*************************************************************************************************************
  * @brief:  	STUSB_MONITORING_STATUS register Structure definition
  * @Address: 	10h
  * @Access: 	RO
  * @Note: 	This register provides information on current status of the VBUS and VCONN voltages 
  *		monitoring done respectively on VBUS_SENSE input pin and VCONN input pin.
  ************************************************************************************************************/ 
#define STUSBADDR_TYPEC_MONITORING_STATUS_1 0x10
typedef union
{
  uint8_t d8;
  struct
  {  
    uint8_t VCONN_VALID	: 	1;
    uint8_t VBUS_VALID_SNK	: 	1;    
    uint8_t VBUS_VSAFE0V	: 	1;    
    uint8_t VBUS_READY		: 	1;   
    uint8_t _Reserved_4_7	:	4;    
  } bitfield;
} STUSB_MONITORING_STATUS_RegTypeDef;




#define	STUSBADDR_CC_STATUS 0x11
#define 				REVERSE 0x80 //1: CC2 is Attach.
#define					SNK_POWER_LEVEL 0x60	
#define Rp_Std 0		//"00: Rp standard current is connected;
#define Rp1500mA 1		//01: Rp 1.5A is connected;
#define Rp3000mA 2		//10: Rp 3A is connected;

#define STUSBADDR_TYPEC_FSM_STATE 0x0F
#define 		SNK_UNATTACHED        0x00
#define 		SNK_ATTACHWAIT        0x01
#define 		SNK_ATTACHED          0x02
#define 		SNK_2_SRC_PR_SWAP     0x06
#define 		SNK_TRYWAIT           0x07
#define 		SRC_UNATTACHED        0x08
#define 		SRC_ATTACHWAIT        0x09
#define 		SRC_ATTACHED          0x0A
#define 		SRC_2_SNK_PR_SWAP     0x0B
#define 		SRC_TRY               0x0C
#define 		ACCESSORY_UNATTACHED  0x0D
#define 		ACCESSORY_ATTACHWAIT  0x0E
#define 		ACCESSORY_AUDIO       0x0F
#define 		ACCESSORY_DEBUG       0x10
#define 		ACCESSORY_POWERED     0x11
#define 		ACCESSORY_UNSUPPORTED 0x12
#define 		ERRORRECOVERY         0x13





#define STUSBADDR_CC_HW_FAULT_STATUS_0     0x12
#define TH_145					0x80 // thermal alarm 
 	                        
#define CS_OVP_TRANS			0x20 // change in CS_OVP status 
#define VPU_VALID_TRANS			0x10 // change in VPUvalidity status 
#define	CC_HW_FAULT_STATUS_TRANS 0x07 	// 1 :bit change in CC_HW_FAULT_STATUS[2.0]

#define CC_HW_FAULT_STATUS_1     0x13
#define CS_OVP 0x20
#define VPU_VALID 0x10
#define STUSBADDR_CC_HW_FAULT_STATUS
#define					SW_OVP_FAULT_CC1_VDDH 0x1
#define					SW_OVP_FAULT_CC2_VDDH 0x2
#define					SW_OCP_FAULT_CC1_VDDH 0x3
#define					SW_OCP_FAULT_CC2_VDDH 0x4
#define					SW_DELTA_FAULT_CC1_VDDH 0x5
#define					SW_DELTA_FAULT_CC2_VDDH 0x6

typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t VCONN_SW_OVP_FAULT_CC2		:	1;
        uint8_t VCONN_SW_OVP_FAULT_CC1		:	1;
        uint8_t VCONN_SW_OCP_FAULT_CC2		:	1;
        uint8_t VCONN_SW_OCP_FAULT_CC1		:	1;
        uint8_t VCONN_SW_RVP_FAULT_CC2		:	1;
        uint8_t VCONN_SW_RVP_FAULT_CC1		:	1;
        uint8_t VPU_PRESENCE			:	1;
        uint8_t VPU_OVP_FAULT			:	1;
  } b;
} STUSB_HW_FAULT_STATUS_RegTypeDef;







#define PE_STATUS               0x15
#define PRT_STATUS 0x16  //HW Protocol Layer
/************************************************************************************************************
  * @brief:  	STUSB_PRT_STATUS register Structure definition
  * @Address: 	16h
  * @Access: 	RO
  * @Note: 	This register provides information on PRT status 
  *		
  ************************************************************************************************************/ 
typedef union
{
  uint8_t d8;
  struct
  {
        uint8_t HWRESET_RECEIVED	:	1;    
        uint8_t HWRESET_DONE		:	1;    
        uint8_t MSG_RECEIVED		:	1;    
        uint8_t MSG_SENT		:	1;    
        uint8_t BIST_RECEIVED		:	1;    
        uint8_t BIST_SENT		:	1;    
        uint8_t Reserved_6		:	1;
        uint8_t TX_ERROR		:	1;
  } bitfield;
} STUSB_PRT_STATUS_RegTypeDef;


#define PHY_STATUS 0x17
/************************************************************************************************************
  * @brief:  	STUSB_GEN1S PHY_STATUS register Structure definition
  * @Address: 	17h
  * @Access: 	RO
  * @Note: 	This register provides information on PHY status 
  *		
  ************************************************************************************************************/ 
typedef union
{
  uint8_t d8;
  struct
  {
       uint8_t TX_MSG_FAIL      :       1;
       uint8_t TX_MSG_DISC      :       1;
       uint8_t TX_MSG_SUCC	:	1;
       uint8_t IDLE		:	1; 
       uint8_t Reserved2	:	1; 
       uint8_t SOP_RX_Type	:	3; 

  } bitfield;
} STUSB_GEN1S_PHY_STATUS_RegTypeDef;


#define Received_SOP 1
#define Received_SOP1          2 
#define Received_SOP2          3
#define Received_SOP_DBG1      4
#define Received_SOP_DBG2      5 
#define Received_Cable_Reset   6 
// All others are reserved."
#define 			TX_MSG_SUCC_STATUS 0x04	//"0: Cleared;   
//1: Reset or SOP* message transmission successful. GoodCRC response received on SOP* message transmission. Transmit SOP* message buffer registers (41h..5Eh) is not full."
#define 			TX_MSG_DISC_STATUS 0x02	//"0: Cleared;           
//1: Reset or SOP* message transmission not sent due to incoming receive message. Transmit SOP* message buffer registers (41h..5Eh) is not full. "
#define 			TX_MSG_FAIL_STATUS 0x01	//"0: Cleared;           
//1: SOP* message transmission not successful, no GoodCRC response received on SOP* message transmission. Transmit SOP* message buffer registers (41h..5Eh) is not full. "

#define PD_ROLE_CNTRL           0x18 
#define 					RP_VALUE 0xC0
#define 					RP_VALUE_MSK 0x03
#define 					RP_VALUE_POSITION 6
#define											Rp_standard_current 0
#define											Rp_1_5A 1
#define											Rp_3A 2

#define DR_SWAP_EN 0x08 //	"0: Data role swap capability is disabled;
//1: Data role swap capability is enabled for Source, Sink and DRP ."
#define PR_SWAP_EN 0x04	//"0: Power role swap capability is disabled;
//1: Power role swap capability is enabled for Source, Sink and DRP ."
#define VCONN_SWAP_EN 0x02	//"0: VCONN  swap capability is disabled;
//1: VCONN swap capability is enabled for Source, Sink and DRP ."
#define VCONN_EN 0x01	//"0: VCONN  supply capability is disabled ;
//1: VCONN supply is capability enabled ."

#define PRT_TX_CTRL             0x19

#define PD_COMMAND              0x1A 
#define                                         SRC_Capa_Valid                     01  // 0x01
#define                                         SRC_capa_changed                   02  // 0x02
#define                                         CMD_Launch                         38  // 0x26

#define RX_MSG_DETECT           0x1B


#define HEADER_MSG_INFO         0x1C
#define DEVICE_CTRL             0x1D
#define PROTOCOL  0
#define POLICY 1
#define Semi_Auto 2
#define Autorun 3

#define ANALOG_CNTRL            0x1E
#define TYPEC_POWER_ROLE_CTRL   0x1F
#define                                 PD_HARD_RESET_TURN_OFF_VCONN_REQ        1
#define                                 PD_HARD_RESET_PORT_CHANGE_2_DFP_REQ     2
#define                                 PD_HARD_RESET_PORT_CHANGE_2_UFP_REQ     3
#define                                 PD_HARD_RESET_COMPLETE_REQ              4

#define                                 PD_PR_SWAP_SNK_VBUS_OFF_REQ             5
#define                                 PD_PR_SWAP_SRC_VBUS_OFF_REQ             6
#define                                 PD_PR_SWAP_RP_ASSERT_REQ                7
#define                                 PD_PR_SWAP_RD_ASSERT_REQ                8

#define                                 PD_DR_SWAP_PORT_CHANGE_2_DFP_REQ        9
#define                                 PD_DR_SWAP_PORT_CHANGE_2_UFP_REQ        0xA

#define                                 PD_VCONN_SWAP_TURN_ON_VCONN_REQ         0xB
#define                                 PD_VCONN_SWAP_TURN_OFF_VCONN_REQ        0xC
#define                                 PD_VCONN_POWERED_NOT_SUPPORTED_REQ      0xD
#define                                 PD_HARD_RESET_RECEIVED_REQ              0xE
#define                                 PD_HARD_RESET_SEND_REQ                  0xF


#define MONITORING_CTRL_0         0x20
#define VCONN MONITOR 0x80
#define VCONN_UV_SEL 0x40
#define OV_MONITOR 0x20
#define VBUS MONITORING_ENA 0x10


#define MONITORING_CTRL_1 0x21 
#define 		VSEL_PDO 0xFF
#define MONITORING_CTRL_2 
#define VSHIFT_HIGH	0xF0 		
#define VSHIFT_LOW 	0x0F 

#define STUSBADDR_SW_RESET 0x23
#define RESET_CTRL 0x23
              #define RESET_SW_DIS 0x00
              #define RESET_SW_EN 0x01

#define PE_FSM 0x29  //Policy Engine State Machine
              #define PE_INIT 0x00
              #define PE_SOFT_RESET 0x01
              #define PE_HARD_RESET 0x02  
              #define PE_SEND_SOFT_RESET 0x03
              #define C_BIST 0x04
              #define PE_SRC_STARTUP 0x05 
              #define PE_SRC_DISCOVERY 0x06
              #define PE_SRC_REQUEST_CAPABILITIES 0x07
              #define PE_SRC_SEND_CAPABILITIES 0x08
              #define PE_SRC_NEGOTIATE_CAPABILITIES 0x09
              #define PE_SRC_TRANSITION_SUPPLY 0x0A
              #define PE_SRC_TSINK_TRANSACTION 0x0B
              #define PE_SRC_TRANSITION_SUPPLY_2 0x0C
              #define PE_SRC_DISABLED 0x0D
              #define PE_SRC_READY 0x0E
              #define PE_SRC_READY_SENDING 0x0F
              #define PE_SRC_CAPABILITY_RESPONSE 0x10
              //#define PE_DB_CP_CHECK_FOR_VBUS 0x30
              #define PE_SRC_TRANSITION_SUPPLY_3 0x31


#define DISCHARGE_TIME_CTRL 0x25
#define DISCHARGE_TIME_TO_0V 0xF0
#define DISCHARGE_TIME_TRANSITION 0x0F

#define STUSB47ADDR_DEVICE_ID 0x2F
              #define STUSB47MASK_4710_NOT_4700 0x01
              #define STUSB47SHIF_4710_NOT_4700 7
              #define STUSB47MASK_ID 0x07 //bit-[2:0]
              #define STUSB47SHIF_ID 3
              #define STUSB47MASK_DEV_CUT 0x02 //bit-[1:0]
              #define STUSB47SHIF_DEV_CUT 0

#define EXTENTION_10  (uint8_t )0x80
#define ID_Reg (uint8_t )0x1C 
#define CUT (uint8_t )3<<2
#define CUT_A (uint8_t )4 << 2 

#define DEV_CUT (uint8_t )0x03
#define AUTO_READY (uint8_t )0x03


#define RX_BYTE_CNT           0x30
#define RX_HEADER             0x31 //RX message header (16bit)
#define RX_DATA_OBJ           0x33
#define RX_DATA_OBJ1          0x33  //(32bit)
#define RX_DATA_OBJ2          0x37  //(32bit)
#define RX_DATA_OBJ3          0x3B  //(32bit)
#define RX_DATA_OBJ4          0x3F  //(32bit)
#define RX_DATA_OBJ5          0x33  //(32bit)
#define RX_DATA_OBJ6          0x37  //(32bit)
#define RX_DATA_OBJ7          0x4B  //(32bit)

#define TX_BYTE_CNT            0x50
#define TX_HEADER_LOW          0x51
#define TX_DATA_OBJ1          0x53
#define TX_DATA_OBJ2          0x57
#define TX_DATA_OBJ3          0x5B
#define TX_DATA_OBJ4          0x5F
#define TX_DATA_OBJ5          0x63
#define TX_DATA_OBJ6          0x67
#define TX_DATA_OBJ7          0x6B

#define STUSBADDR_DPM_PDO_NUMB          0x70
#define         STUSB4700SHIFT_DPM_SRC_PDO_NUMB 5
#define         STUSB4700MASK_DPM_SRC_PDO_NUMB 0x07

#define STUSBADDR_DPM_SRC_PDO1          0x71
#define STUSBADDR_DPM_SRC_PDO2          0x75
#define STUSBADDR_DPM_SRC_PDO3          0x79
#define STUSBADDR_DPM_SRC_PDO4          0x7D
#define STUSBADDR_DPM_SRC_PDO5          0x81

#define STUSB4700SHIFT_PDO_VOLTAGE          10
#define STUSB4700MASK_PDO_VOLTAGE          0x03FF
#define STUSB4700SHIFT_PDO_MAXCURRENT       0
#define STUSB4700MASK_PDO_MAXCURRENT        0x03FF

#define DPM_SNK_PDO1          0x85
#define DPM_SNK_PDO2          0x89
#define DPM_SNK_PDO3          0x8D

#define DPM_REQ_RDO1          0x91 // RDO = Request Data Object
#define RDO_REG_STATUS 0x91 // RDO = Request Data Object
typedef union
{
  uint32_t d32;
  struct
  {     
        uint32_t MaxCurrent                     :       10;
        uint32_t OperatingCurrent               :       10;
        uint8_t reserved_22_20                  :       3;
        uint8_t UnchunkedMess_sup               :       1;
        uint8_t UsbSuspend                      :       1;
        uint8_t UsbComCap                       :       1;
        uint8_t CapaMismatch                    :       1;
        uint8_t GiveBack                        :       1;
        uint8_t Object_Pos                      :       3;
        uint8_t reserved_31		        :	1;
        
  } bitfield;
} STUSB_RDO_REG_STATUS_RegTypeDef;







#define FTP_CUST_PASSWORD_REG	0x95
#define FTP_CUST_PASSWORD		0x47
#define FTP_CTRL_0              0x96
	#define FTP_CUST_PWR	0x80 
	#define FTP_CUST_RST_N	0x40
	#define FTP_CUST_REQ	0x10
	#define FTP_CUST_SECT 0x07
#define FTP_CTRL_1              0x97
	#define FTP_CUST_SER 0xF8
	#define FTP_CUST_OPCODE 0x07

	
/*"000" then No Operation
"001" then Read 
"010" and FTP_ADR[2:0]="000" then Shift-In Write Bit Data (0x20-0x28). (to be done before Programming)
"010" and FTP_ADR[2:0]="001" then Shift-In Erase Sector Data (0x20). (to be done before Erasing)
"011" and FTP_ADR[2:0]="000" then Shift-Out Read Bit Data (0x20-0x28). (to be done after Reading) 
"011" and FTP_ADR[2:0]="001" then Shift-Out Erase Sector Data (0x20). (to be done after Erasing) 
"100" then Verify (to be done after Programming)
"101" then Erase
"110" then Program
"111" then Soft Programming (to be done after Erasing)*/
#define READ 0x00
#define WRITE_PL 0x01
#define WRITE_SER 0x02
#define READ_PL	0x03
#define READ_SER 0x04
#define ERASE_SECTOR 0x05
#define PROG_SECTOR 0x06
#define SOFT_PROG_SECTOR 0x07
        
        #define	SECTOR_0	0x01
	#define	SECTOR_1	0x02
	#define	SECTOR_2	0x04
	#define	SECTOR_3	0x08
	#define	SECTOR_4	0x10
	#define	SECTOR_5	0x20



	
	
#endif //#ifndef __STUSB4700_DEFINES_H



