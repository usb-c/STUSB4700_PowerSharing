int UART_start_message(void);
int UART_Rx_init(void);
void Print_OK();
void UART_CheckReceivedMessage();
int uart_process(void);

void UART_CharReception_Callback(void);
void UART_Error_Callback(void);
