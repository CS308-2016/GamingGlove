#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>
#include <math.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/pwm.h"
#include "driverlib/pin_map.h"
#include "inc/hw_gpio.h"
#include "driverlib/rom.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"


#define PWM_FREQUENCY 55


// r, g, b go from 0 to 240
void calcIntensities(uint32_t angle, uint32_t *r, uint32_t *g, uint32_t *b) {

	uint32_t angle1 = angle % 120;
	// Calc increasing val
	uint32_t val1 = 1 + ((255 - 1) * angle1) / 120;
	// Calc decreasing val
	uint32_t val2 = 255 - val1;
	*r = 1;
	*g = 1;
	*b = 1;

	if (angle < 120) {
		// Red green
		*r = val2;
		*g = val1;
	} else if (angle < 240) {
		*g = val2;
		*b = val1;
	} else {
		// Blue red
		*r = val1;
		*b = val2;
	}

	return;
}

uint32_t r, g, b;
volatile uint8_t colorWheelSpeed;

volatile uint16_t colorWheelAngle;

uint8_t mode;

int sw1_state = 0; // 0 is idle, 1 is press, 2 is release

uint8_t sw1_count;
uint8_t sw2_count;
uint8_t sw1_count1;
uint8_t cur_led;
uint32_t br, bg, bb;

int main(void)
{

	volatile uint32_t ui32Load;
	volatile uint32_t ui32PWMClock;
	// Init setup
	colorWheelAngle = 0;
	colorWheelSpeed = 2;
	mode = 'a';
	sw1_count = 0;
	sw2_count = 0;
	sw1_count1 = 0;
	cur_led = 'r';


	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	SysCtlPWMClockSet(SYSCTL_PWMDIV_64);

	SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);

	GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
	GPIOPinConfigure(GPIO_PF1_M1PWM5);
	GPIOPinConfigure(GPIO_PF2_M1PWM6);
	GPIOPinConfigure(GPIO_PF3_M1PWM7);


	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY;
	HWREG(GPIO_PORTF_BASE + GPIO_O_CR) |= 0x01;
	HWREG(GPIO_PORTF_BASE + GPIO_O_LOCK) = 0;
	GPIODirModeSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_DIR_MODE_IN);
	GPIOPadConfigSet(GPIO_PORTF_BASE, GPIO_PIN_4|GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);


	ui32PWMClock = SysCtlClockGet() / 64;
	ui32Load = (ui32PWMClock / PWM_FREQUENCY) - 1;
	PWMGenConfigure(PWM1_BASE, PWM_GEN_2, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_2, ui32Load);
	PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_DOWN);
	PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Load);

	calcIntensities(0, &r, &g, &b);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, r * ui32Load / 1000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, g * ui32Load / 1000);
	PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, b * ui32Load / 1000);
	PWMOutputState(PWM1_BASE, PWM_OUT_5_BIT | PWM_OUT_6_BIT | PWM_OUT_7_BIT, true);
	PWMGenEnable(PWM1_BASE, PWM_GEN_2);
	PWMGenEnable(PWM1_BASE, PWM_GEN_3);

	uint32_t pollfreq = 20;
	uint32_t ui32Period = (SysCtlClockGet() / pollfreq);
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);

	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();
	TimerEnable(TIMER0_BASE, TIMER_A);


	while(1)
	{
		if(mode == 'a') {
			colorWheelAngle = (colorWheelAngle + colorWheelSpeed - 1) % 360;
			calcIntensities(colorWheelAngle, &r, &g, &b);
		} else {

		}


		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_5, (uint8_t)r * ui32Load / 1000);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, (uint8_t)b * ui32Load / 1000);
		PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, (uint8_t)g * ui32Load / 1000);
		SysCtlDelay(400000);
	}

}


unsigned char detectKeyPress1(uint8_t val) {
	// Handle sw1 automaton

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
		return !flag1;
}


void fix(uint32_t *v) {
	if (*v < 1) {
		*v = 1;
	}

	if (*v > 254) {
		*v = 254;
	}
}

void Timer0IntHandler(void)
{

	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);



	if (mode == 'a') {
		uint8_t sw1 = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4);
		uint8_t sw2 = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0);
		if (sw1==0x0 && sw2==0x0) {
			mode = 'm';
		} else if (sw1==0x0) {
			colorWheelSpeed--;
			if (colorWheelSpeed < 1)
			{
				colorWheelSpeed = 1;
			}
		} else if (sw2 == 0x0) {
			colorWheelSpeed++;
			if (colorWheelSpeed > 61)
			{
				colorWheelSpeed = 61;
			}
		}
	} else {
		uint8_t long_thresh = 20;
		uint8_t sw2 = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_0);
		uint8_t sw1 = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4);
		uint8_t sw1v = detectKeyPress1(sw1);
		if (sw2 == 0x0) {
			if (sw2_count == 0) {
				br = r;
				bg = g;
				bb = b;
			}
			sw2_count = sw2_count + 1;
			if (sw2_count > long_thresh) {
				sw2_count = long_thresh;
			}
			if (sw1 == 0x0) {
				sw1_count1 = sw1_count1 + 1;
				if (sw1_count1 > long_thresh) {
					sw1_count1 = long_thresh;
				}
			}
			if (sw1v == 0x0) {
				sw1_count ++;
			}
		} else {
			if (sw2_count == long_thresh) { // 2 long press
				uint8_t past_state = cur_led;
				if (sw1_count1 == long_thresh && sw1_count == 1) {
					// 1 long press
					cur_led = 'g';
				} else if (sw1_count == 1) {
					cur_led = 'r';
				} else if (sw1_count == 2) {
					cur_led = 'b';
				} else {

				}
				if (past_state != cur_led) {
					r = br;
					g = bg;
					b = bb;
				}
			}
			sw1_count = 0;
			sw2_count = 0;
			sw1_count1 = 0;
		}

		int del = 0;
		if (sw1 == 0x0 && sw2 == 0x0) {
			del = 0;
		} else if (sw1 == 0x0) {
			del = -1;
		} else if (sw2 == 0x0) {
			del = 1;
		}

		switch (cur_led) {
		case 'r':
			r += del;
			fix(&r);
			break;
		case 'g':
			g += del;
			fix(&g);
			break;
		case 'b':
			b += del;
			fix(&b);
			break;

		}
	}
}

