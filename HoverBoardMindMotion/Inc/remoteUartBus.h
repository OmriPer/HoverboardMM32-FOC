#ifdef TARGET_MM32SPIN25
#include "HAL_device.h"                 // Device header
#else
#include "mm32_device.h"                // Device header
#endif

void AnswerMaster(void);
void RemoteUpdate(void);
void RelayRxByte(uint8_t cRead);    //master relay: byte received from the slave (UART2)
