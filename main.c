#include "FreeRTOS.h"
#include "queue.h"
#include "task.h"
#include "semphr.h"
#include <stdint.h>
#include <stdbool.h>
#include "TM4C123GH6PM.h"
#include "inc\hw_timer.h"
#include "inc\hw_gpio.h"
#include "driverlib\timer.h"
#include "driverlib\gpio.h"
#include "driverlib\sysctl.h"
#include "inc\tm4c123gh6pm.h"
#include "inc/hw_types.h"
#include "driverlib/pin_map.h"
#include "driverlib/interrupt.h"
#include "inc/hw_ints.h"
#include "tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "driverlib/systick.h"
#include "driverlib/interrupt.h"
#include "inc/hw_types.h"
#include "inc/hw_timer.h"
#include "driverlib/timer.h"
#include "driverlib/sysctl.h"
#include "driverlib/GPIO.h"
#include "LCD.h"
#include "ADC.h"

QueueHandle_t xUARTQueue;
QueueHandle_t xLCDQueue;
QueueHandle_t xBuzzerQueue;

//Convert Temp and Setpoints to String datatype
void convertToString (char tim, char text []){
	//initialize text [0,0]
	for (int j =0; j<2; j++){
		text[j]='0';
	}
	//put numbers in char array
        int i = 2;
	while (tim != 0){
		i--;		
		text[i]=((tim%10)+'0');
		tim/=10;
	}
	text[2]='\0';//add null terminator
}

//Display characters on UART-Putty
void printchar(char c){
while((UART0_FR_R&(1<<5))!=0);
UART0_DR_R=c;
}
//Display strings on UART-Putty
void print(char *string){
  while(*string){
  printchar(*(string++));
  }
}

//Initialization of Ports A/B/D/E/F
void tiva_init(){
  //PORTF initialization
	SYSCTL_RCGCGPIO_R |= 0x00000020;      
  while((SYSCTL_PRGPIO_R&0x00000020) == 0){}
  GPIO_PORTF_LOCK_R = 0x4C4F434B;
  GPIO_PORTF_CR_R = 0x1F;
  GPIO_PORTF_DIR_R = 0x0E;
  GPIO_PORTF_DEN_R = 0x1F;
  GPIO_PORTF_PUR_R = 0x11;

	//PORTD initialization
	SYSCTL_RCGCGPIO_R |= 0x00000008;
  while((SYSCTL_PRGPIO_R&0x00000008) == 0){};
  GPIO_PORTD_LOCK_R = 0x4C4F434B;
  GPIO_PORTD_CR_R = 0xFF;
  GPIO_PORTD_DIR_R = 0xFF;
  GPIO_PORTD_DEN_R = 0xFF;
	
	//PORTE initialization
	SYSCTL_RCGCGPIO_R |= 0x00000010;
  while((SYSCTL_PRGPIO_R&0x00000010) == 0){};
  GPIO_PORTE_LOCK_R = 0x4C4F434B;
  GPIO_PORTE_CR_R = 0x2F;
  GPIO_PORTE_DIR_R = 0x2F;
  GPIO_PORTE_DEN_R = 0x2F;
	
	// PORTA initialization (UART)
	SYSCTL_RCGCUART_R|=0X0001;			
	SYSCTL_RCGCGPIO_R |= 0x00000001;
	UART0_CTL_R &= ~0x0001;
	UART0_CC_R=0X0;
	UART0_IBRD_R=104;
	UART0_FBRD_R=11;
	UART0_LCRH_R=(0x3<<5);
	GPIO_PORTA_AFSEL_R|=0X03;
	GPIO_PORTA_PCTL_R=0X011;
	GPIO_PORTA_DEN_R|=0X03;
	UART0_CTL_R=0x0301;
	
	//LCD initialization
	LCD_Init();
}

//Main Task (Task1)
void Task1(void *pvParameters){

	typedef struct Message{
	char Txt1[4]; 
	char Txt2[4];
	}AMessage;
	AMessage msg;

	char *on;
	char *off;
	on = 1; 		//buzzer on
	off = 0; 		//buzzer off
	unsigned char setpoint = 30; 						//initial setpoint
	unsigned AdcValue;
	unsigned char Temperature;
	float mV;
	unsigned const char AlarmValue = 50;  //initial alarm value
	adc_init() ;
  while(1)
		{
			xQueueReceive(xUARTQueue,	&setpoint, 0);	//Recieve setpoint entry from putty
			AdcValue =  adc_read();					  				//Read ADC voltage value
			mV =	147 - (247 * AdcValue) / 4096; 			//Calculate tempurature in Celsius
			Temperature = (int)mV;										//Convert tempurature to integer
			if(Temperature < setpoint){								//Check temperature compared to setpoint		
				GPIO_PORTE_DATA_R |= 0x02;							//Turn on green LED (Heater ON)
			}else{																		//Temperature is already higher		
				GPIO_PORTE_DATA_R &=~ 0x02;							//Turn off green LED (Heater OFF)
			}
			convertToString(Temperature, msg.Txt1);		//Current Temperature
			convertToString(setpoint, msg.Txt2);			//Entered Setpoint
			xQueueSend(xLCDQueue, &msg, 0);						//Pass temp and setpoints values to LCD task through xLCDQueue
			if(Temperature > AlarmValue) 							//Check temperature compared to Alarm value
				xQueueSend(xBuzzerQueue, &on, 0); 			//Turn on red LED and Buzzer
			else																			//Temperature is lower
				xQueueSend(xBuzzerQueue,&off,0);				//Turn off red LED and Buzzer
		}
}

//UART Task (Task2)
void Task2(void *pvParameters){
	unsigned N;
	unsigned AdcValue;
	unsigned char Total; 
  while(1){
		print("\n\r\nEnter Temperature Setpoint (Degrees): ");
		N=0;
		Total=0;
			while(1){
				while((UART0_FR_R&(1<<4))!=0);		//Read entered number
				N= UART0_DR_R;										//Echo the number
				print(&N);												//Display it on Putty terminal
				if(N=='\r') break;								//If Enter
				N=N-'0';													//Pure number
				Total=10*Total+N;									//Set total number to be passed as setpoint
			}
		xQueueSend(xUARTQueue,&Total,pdMS_TO_TICKS(10));	//Pass setpoint value to main task through xUARTQueue
		print("\n\rTemperature Setpoint changed...");	
	}
}

//LCD task (Task3)
void Task3(void *pvParameters){
	typedef struct Message
	{
		char Txt1[4];
		char Txt2[4];
	
	} AMessage;
	AMessage msg;
	LCD_Reset ();
	while(1){
		xQueueReceive(xLCDQueue,&msg,0);			//Recieve temp and setpoint values
		LCD_Row(1); 													//Select LCD's first row
		LCD_Show("Measured: ");   						//Print Temperature
		LCD_Show(msg.Txt1);  									//Print the measured value
		LCD_Row(2);														//Select LCD's second row
		LCD_Show("Setpoint: "); 							//Print Setpoint
		LCD_Show(msg.Txt2); 									//Print the entered value
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

//Buzzer Task (Task4)
void Task4(void *pvParameters){
unsigned char Buzzerstate;
	Buzzerstate=0;                    //initial buzzer state
    while(1){
			xQueueReceive(xBuzzerQueue,&Buzzerstate,0); //Recieve buzzer state from main task
			if(Buzzerstate==1){						//If sent value is on
			GPIO_PORTE_DATA_R|=0x05;			//Turn on red LED and Buzzer
			}else{												//If sent value is off
		 GPIO_PORTE_DATA_R&=~0x05;			//Turn on red LED and Buzzer
			}
    }
}


int main(void)
{
	//Initialize Queues to be used
	xUARTQueue = xQueueCreate(1,1);			//From Task 2 to Task 1
	xLCDQueue = xQueueCreate(1,8);			//From Task 1 to Task 3
	xBuzzerQueue = xQueueCreate(1,1);		//From Task 1 to Task 4
	
	//Initialize GPIO ports
  tiva_init();
	
	//Initialize 4 Tasks
	xTaskCreate(Task1,"Main Task",100,NULL, 1,0);
	xTaskCreate(Task2,"Uart Task",100,NULL, 1,0);
	xTaskCreate(Task3,"LCD Task",100,NULL, 1,0);
	xTaskCreate(Task4,"Buzzer Task",100,NULL, 1,0);
	vTaskStartScheduler();

}
