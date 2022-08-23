# Temperature-Control-Embedded-System
An ON-OFF temperature controller application on Arm Cortex M4 TivaC (TM4C123GH6PM) using FreeRTOS by measuring current temperature using temperature sensor and switch the heater on/off till it maintain the setpoint and display the measured temperature with the setpoint temp on the LCD.


*Implemented Functions:

Main:
Where we initialized queues, GPIO ports and the 4 tasks.

• Tiva_init( ):
Initialize GPIO Ports

• Task 1:
Receives etpoint entry from putty, read ADC value, calculate 
temperature in Celsis and converting temperature to integer 
to be read and checked comparable to the setpoint.
As well as turning on Green LED which implies turning the 
heater ON. If temreature is already higher then the green 
LED will be turned OFF.

• Task 2:
Reading the entered number and printing it to the UART as
well as displaying it to the putty terminal and set the total 
number to be passed as setpoint.
It passes the setpoint value to the main task through 
xUARTQueue.

• Task 3:
Receives the temperature and the setpoint values as well as 
selecting the LCDs first row.
Printing the temperature and the measured value as well as 
selecting LCD’s second row. Lastly print the setpoint and the 
entered value.

• Task 4:
Used to initialize the buzzer task and receives the buzzer 
srare from the main task. 
If the sent value is ON then we’ll turn on the RED LED and 
the Buzzer.