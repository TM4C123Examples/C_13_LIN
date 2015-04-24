#ifndef __RETARGET_TM4C_H
#define __RETARGET_TM4C_H
#include "TM4C123.h"                    // Device header
#include "retarget.h"
	// U0Rx (VCP receive) connected to PA0
	// U0Tx (VCP transmit) connected to PA1
	void UART0_init(void);
	int UART0_kbhit(void);
#endif
