// U0Rx (VCP receive) connected to PA0
// U0Tx (VCP transmit) connected to PA1

#include "retarget_tm4c.h"

#define UART_FR_TXFF            0x00000020  // UART Transmit FIFO Full
#define UART_FR_RXFE            0x00000010  // UART Receive FIFO Empty
#define UART_LCRH_WLEN_8        0x00000060  // 8 bit word length
#define UART_LCRH_FEN           0x00000010  // UART Enable FIFOs
#define UART_CTL_UARTEN         0x00000001  // UART Enable
#define SYSCTL_RCGC1_UART0      0x00000001  // UART0 Clock Gating Control
#define SYSCTL_RCGC2_GPIOA      0x00000001  // port A Clock Gating Control


//------------UART_Init------------
// Wait for new serial port input
// Initialize the UART for 115,200 baud rate (assuming 50 MHz UART clock),
// 8 bit word length, no parity bits, one stop bit, FIFOs enabled
// Input: none
// Output: none
void UART0_init(void){
  SYSCTL->RCGC1|= SYSCTL_RCGC1_UART0; // activate UART0
  SYSCTL->RCGC2|= SYSCTL_RCGC2_GPIOA; // activate port A
  UART0->CTL &= ~UART_CTL_UARTEN;      // disable UART
  UART0->IBRD = 27;                    // IBRD = int(50,000,000 / (16 * 115200)) = int(27.12673611)
  UART0->FBRD= 8;                    // FBRD = round(0.12673611 * 64) = 8
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART0->LCRH = (UART_LCRH_WLEN_8|UART_LCRH_FEN);
  UART0->CTL|= UART_CTL_UARTEN;       // enable UART
  GPIOA->AFSEL |= 0x03;           // enable alt funct on PA1-0
  GPIOA->DEN |= 0x03;             // enable digital I/O on PA1-0
                                        // configure PA1-0 as UART
  GPIOA->PCTL = (GPIOA->PCTL&0xFFFFFF00)+0x00000011;
  GPIOA->AMSEL &= ~0x03;          // disable analog functionality on PA
}

int UART0_kbhit(void){
	if((UART0->FR&UART_FR_RXFE) != 0){
		return 0;
	}else{
	return 1;
	}
}

int getkey(void){
  while((UART0->FR&UART_FR_RXFE) != 0);
  return ((unsigned char)((UART0->DR)&0xFF));
}

void sendchar(int ch){
  while((UART0->FR&UART_FR_TXFF) != 0);
  UART0->DR =(ch&(0xFF));
}
