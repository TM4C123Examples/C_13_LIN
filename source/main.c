/********/
/*Este proyecto hace uso del Semaforo*/
#include "TM4C123.h"                    // Device header
#include "retarget_tm4c.h"
#include <stdio.h>

int main(){
	char buffer[80];
	UART0_init();
	printf("\nIngrese comando");
	while(1){
			printf("$ ");
			gets(buffer);
			printf("%s\n",buffer);
	}
}
