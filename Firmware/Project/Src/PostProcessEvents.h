int PostProcess_UsbEvents(uint8_t UsbPort);
int Push_PD_MessageReceived(int UsbPort, char MessageType, char MessageValue);
int Pop_PD_MessageReceived(int UsbPort);
int Push_IrqReceived(int UsbPort, char MessageType);
int Pop_IrqReceived(int UsbPort);


#include "Hardware_configuration.h" //for I2CBUS_MAX, USBPORT_MAX

extern volatile int PostProcess_IrqReceived[USBPORT_MAX];
extern volatile int PostProcess_AttachTransition[USBPORT_MAX];
extern volatile int PostProcess_IrqHardreset;
extern volatile int PostProcess_PD_MessageReceived[USBPORT_MAX];
extern volatile int PostProcess_SRC_PDO_Received;
extern volatile int PostProcess_PSRDY_Received;
extern volatile int PostProcess_Msg_Accept;
extern volatile int PostProcess_Msg_Reject;
extern volatile int PostProcess_Msg_GoodCRC;

