
//Defined by the OEM, depending on hardware implementation

#define USBPORT_MAX 4
#define I2CBUS_MAX 2

#define TOTAL_SYSTEM_POWER 100 //Watt
#define MAX_POWER_PER_USBPORT 60 //Watt (e.g: 15V*3A = 45W, 20V*3A = 60W)
#define SMALLEST_POWER_PER_USBPORT 5 //Watt  (e.g: 5V*1A = 5W, 5V*1.5A = 7.5W)

#define PDO_NUMBER_MAX 5 //max PDO number per USB port
#define PDO_VOLTAGE_TABLE {5, 9, 12, 15, 20} //in V
#define MIN_CURRENT_PER_PDO 1 //(in A)
#define MAX_CURRENT_PER_PDO 3 //(in A)
