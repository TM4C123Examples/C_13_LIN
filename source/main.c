/********/
/*Este proyecto hace uso del Semaforo*/
#include "TM4C123.h"                    // Device header
#include "retarget_tm4c.h"
#include <stdio.h>

void timerConfig(void);
void set_timer(unsigned int, unsigned char);
void timerIRQConfig(unsigned int);
void timerEnable(unsigned int);
void configureLeds(void);

void lin_init(void);
int sendchar3(int ch);
void lin_sendBreak(void);

int contador=0;
volatile int BREAK_SENT=0;

int main(){
	char buffer[80];
	UART0_init();
	lin_init();
	printf("\nIngrese comando\n");
	//fprintf(&stream3,"\nLos comando se mostraran aqui\n");
	while(1){
		printf("$ ");
		gets(buffer);	
		printf("%s\n",buffer);
		lin_sendBreak();
		sendchar3(0x55);
		sendchar3(0x49);
		sendchar3(0x0);
		sendchar3(0x1);
		sendchar3(0x2);
		sendchar3(0x3);
	}
}

void lin_init(void){
	SYSCTL->RCGCUART|= (0x1<<3); // activate UART3
  SYSCTL->RCGCGPIO|= (0x1<<2); // activate port C
  //PC6 = Rx  PC7 = Tx  (UART3)
  GPIOC->AMSEL &= ~(0x1<<6)|(0x1<<7);// disable analog functionality on PC6 PC7
	GPIOC->DEN |=(0x1<<6)|(0x1<<7);  // enable digital I/O on PC6 PC7                                      
	GPIOC->PUR|=(0x1<<6);//Pull Up on PC6
	GPIOC->PCTL &= ~((0xF<<28)|(0xF<<24));//
	GPIOC->PCTL |=(0x1<<28)|(0x1<<24);// alternate fucntion 1 for PC7 and PC6 
	GPIOC->AFSEL |= (0x1<<6)|(0x1<<7);// enable alt funct on PC6 PC7
	
	UART3 ->CTL &= ~(0x1<<0);      // disable UART
  UART3->IBRD = 325;                    // IBRD = int(50,000,000 / (16 * 9600)) = int(325.52)
  UART3->FBRD= 33;                    // FBRD = round(0.52 * 64) = 8
                                        // 8 bit word length (no parity bits, one stop bit, FIFOs)
  UART3->LCRH = ((0x3<<5)| //(6:5) WLEN UART Word Length = 0x3 : 8 bits 
								(0x1<<4)); //(4) FEN  UART Enable FIFO = 1 RX and Tx FIFObuffers are enabled
  UART3->CTL|= (0x1<<0);       // enable UART
	UART3->ICR = (0x1<<9);
	UART3->IM |= (0x1<<9); //UART Break Error Interrupt Mask;
	NVIC_EnableIRQ(UART3_IRQn);
	timerConfig();
	set_timer(2604,1);//reset every (0+1)*1133 counts 
	timerIRQConfig(1);
};

int sendchar3(int ch){
  while((UART3->FR&(0x1<<5)));//while buffer not full
  UART3->DR =(ch&(0xFF));
	return ch;
}

void lin_sendBreak(void){
	while(!(UART3->FR&(0x1<<7)))//while buffer not empty
	BREAK_SENT=0;
	UART3->LCRH|=(0x1<<0);//Send break
	while(!BREAK_SENT){
	}
}

void UART3_Handler(void){
	if(UART3->MIS&(0x1<<9)){//break detected
		UART3->ICR = (0x1<<9);
		contador++;
		set_timer(2604,1);
		timerEnable(1);
	}
}


void configureLeds(){
		SYSCTL->RCGCGPIO|=(0x1<<5);//Enable GPIOF
		GPIOF->DEN|=(0x1<<1)|(0x1<<2)|(0x1<<3);//Set GPIOF[1..3] for digital operation
		GPIOF->DIR|=(0x1<<1)|(0x1<<2)|(0x1<<3);//Set  GPIOF[1..3] as outputs
}


void set_timer(unsigned int count, unsigned char prescale){
	timerEnable(0);
	TIMER0->TAILR=count;
	TIMER0->TAPR=prescale;
}

void timerEnable(unsigned int enable){
		if(enable){
				TIMER0->CTL|=0x01;
		}else{
			  TIMER0->CTL&=~((unsigned int) 0x01);
		}
}

void timerConfig(void){
		SYSCTL->RCGCTIMER|=0x1;//Enable peripherial timer0
	  TIMER0->CTL=0x00;//Disable timer during configuration 
	  TIMER0->CFG=0x4;// a/b 16 bit mode 
	  TIMER0->TAMR=(0x0<<4)|(0x02);//TAM as periodic timer/ down timer
	  TIMER0->TAILR=1023;//set top value as 1023
	  TIMER0->CTL=(0x1<<1);//Enable debug stall
}

void timerIRQConfig(unsigned int enable){
	  TIMER0->IMR=0x01;
	  if(enable){
			NVIC_EnableIRQ(TIMER0A_IRQn);
		}else{
			NVIC_DisableIRQ(TIMER0A_IRQn);
		}
}

void TIMER0A_Handler(){
	  TIMER0->ICR=0x01;//clear interrupt
		UART3->LCRH&=~(0x1<<0);//Clear Send break bit
	  timerEnable(0);
		BREAK_SENT=1;
}
