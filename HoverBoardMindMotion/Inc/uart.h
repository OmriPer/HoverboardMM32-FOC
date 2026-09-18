#ifdef TARGET_MM32SPIN25
#include "HAL_device.h"                 // Device header
#else
#include "mm32_device.h"                // Device header
#endif
#include  "stdio.h"
void UART_Send_Byte(u8 dat);
void UART_Send_Group(u8* buf, u16 len);
void UART_Send_GroupTo(UART_TypeDef* uart, u8* buf, u16 len);    //blocking, on a given UART
void UART_SendString(char* str);