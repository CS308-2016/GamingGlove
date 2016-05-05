/*

* Author: Texas Instruments 

* Editted by: Saurav Shandilya, Vishwanathan Iyer 
	      ERTS Lab, CSE Department, IIT Bombay

* Description: This code will familiarize you with using GPIO on TMS4C123GXL microcontroller.

* Filename: lab-1.c

* Functions: setup(), ledPinConfig(), switchPinConfig(), main()

* Global Variables: none

*/

#include <stdint.h>
#include <stdbool.h>
#include "inc/tm4c123gh6pm.h"
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/gpio.h"
#include "driverlib/timer.h"


// LOCK_F and CR_F - used for unlocking PORTF pin 0
#define LOCK_F (*((volatile unsigned long *)0x40025520))
#define CR_F   (*((volatile unsigned long *)0x40025524))

/*
 ------ Global Variable Declaration
*/


/*

* Function Name: setup()

* Input: none

* Output: none

* Description: Set crystal frequency and enable GPIO Peripherals  

* Example Call: setup();

*/
void setup(void)
{
	SysCtlClockSet(SYSCTL_SYSDIV_4|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
}



void ledPinConfig(void)
{
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);  // Pin-1 of PORT F set as output. Modifiy this to use other 2 LEDs.
}


/*

* Function Name: switchPinConfig()

* Input: none

* Output: none

* Description: Set PORTF Pin 0 and Pin 4 as input. Note that Pin 0 is locked.

* Example Call: switchPinConfig();

*/
void switchPinConfig(void)
{
	// Following two line removes the lock from SW2 interfaced on PORTF Pin0 -- leave this unchanged
	LOCK_F=0x4C4F434BU;
	CR_F=GPIO_PIN_0|GPIO_PIN_4;

	// GPIO PORTF Pin 0 and Pin4
	GPIODirModeSet(GPIO_PORTF_BASE,GPIO_PIN_0 | GPIO_PIN_4 ,GPIO_DIR_MODE_IN); // Set Pin-4 of PORT F as Input. Modifiy this to use another switch
	GPIOPinTypeGPIOInput(GPIO_PORTF_BASE, GPIO_PIN_0 | GPIO_PIN_4);
	GPIOPadConfigSet(GPIO_PORTF_BASE,GPIO_PIN_0 | GPIO_PIN_4 ,GPIO_STRENGTH_12MA,GPIO_PIN_TYPE_STD_WPU);
}

int sw2_status = 0;
unsigned char led_col = 0x2;

int sw1_state = 0; // 0 is idle, 1 is press, 2 is release
int sw2_state = 0;


int main(void)
{


	setup();
	switchPinConfig();
	ledPinConfig();

	uint32_t pollfreq = 50;
	uint32_t ui32Period = (SysCtlClockGet() / pollfreq);
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);

	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();
	TimerEnable(TIMER0_BASE, TIMER_A);
	GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, led_col);



	while (1) {
	}


}


unsigned char detectKeyPress1() {
	// Handle sw1 automaton
		int val = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_4);
		unsigned char flag1 = 0;

		switch(sw1_state) {
		case 0: //idle
			if (!val) { //pressed
				sw1_state = 1;
			}
			break;
		case 1: // Press
			if (!val) { //pressed
				sw1_state = 2;
				flag1 = 1;
			} else {
				sw1_state = 0;
			}
			break;
		case 2: // Release
			if (val) { // released
				sw1_state = 0;
			}
			break;
		}
		return flag1;
}


unsigned char detectKeyPress2() {
	unsigned char flag2 = 0;
	int val = GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_0);

	switch(sw2_state) {
	case 0: //idle
		if (!val) { //pressed
			sw2_state = 1;
		}
		break;
	case 1: // Press
		if (!val) { //pressed
			sw2_state = 2;
			flag2 = 1;
		} else {
			sw2_state = 0;
		}
		break;
	case 2: // Release
		if (val) { // released
			sw2_state = 0;
		}
		break;
	}
	return flag2;
}


void Timer0IntHandler(void)
{

	// Clear the timer interrupt
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	if (detectKeyPress1()) {
		if (led_col == 2) {
			led_col = 8;
		} else if (led_col == 8) {
			led_col = 4;
		} else {
			led_col = 2;
		}
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3, led_col);

	}

	if (detectKeyPress2()) {
		sw2_status++;
	}
}
