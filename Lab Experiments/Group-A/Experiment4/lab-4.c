#define  UART_BUFFERED

#include<stdint.h>
#include<stdbool.h>
#include"inc/hw_memmap.h"
#include"inc/hw_types.h"
#include"driverlib/debug.h"
#include"driverlib/sysctl.h"
#include"driverlib/adc.h"
#include"inc/hw_ints.h"
#include"driverlib/interrupt.h"
#include"driverlib/gpio.h"
#include"driverlib/pin_map.h"
#include"driverlib/uart.h"
#include"utils/uartstdio.h"
#include "driverlib/timer.h"


volatile uint32_t ui32TempAvg;
volatile uint32_t ui32TempValueC;
volatile uint32_t ui32TempValueF;
volatile uint32_t ui32TempSetValueC;
uint32_t ui32ADC0Value[4];

void init_measurement_timer() {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	uint32_t pollfreq = 1;
	uint32_t ui32Period = (SysCtlClockGet() / pollfreq);
	TimerLoadSet(TIMER0_BASE, TIMER_A, ui32Period -1);
	IntEnable(INT_TIMER0A);
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
	IntMasterEnable();
	TimerEnable(TIMER0_BASE, TIMER_A);
}

void init_adc() {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //ADC pin
	ADCHardwareOversampleConfigure(ADC0_BASE, 4);
	ADCSequenceConfigure(ADC0_BASE, 1, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 0, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 1, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE, 1, 2, ADC_CTL_CH0);
	ADCSequenceStepConfigure(ADC0_BASE,1,3,ADC_CTL_CH0|ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 1);

}

void init_uart() {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);
	UARTClockSourceSet(UART0_BASE, UART_CLOCK_PIOSC);
	UARTStdioConfig(0, 115200, 16000000);
}

void init_led() {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_3);
}

void init() {
	SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_OSC_MAIN|SYSCTL_XTAL_16MHZ);
	init_measurement_timer();
	init_uart();
	init_adc();
	init_led();
}



char mode = 'm';

int tset;
int loc;

int main(void) {

	init();
	SysCtlDelay(1000);
	ADCIntClear(ADC0_BASE, 1);

	while(1) {
		if (mode == 'm') {
			char x = UARTgetc();
			if (x == 's' || x == 'S') {
				mode = 's';
			}
		} else if (mode == 's') {
			loc = 0;
			char buf[9];
			int i;
			for (i = 0; i < 9; ++i) {
				buf[i] = 0;
			}

			UARTprintf("Enter the temperature : ");
			while(1) {
				char c = UARTgetc();

				if (c == (char)127) {
					if (loc) {
						loc--;
					}
				} else if (c == '\r' || c == 'n') {
					UARTprintf("\n");
					break;
				} else {
					if (loc < 8) {
						buf[loc++] = c;
					}
				}
				UARTprintf("%c", c);
			}

			tset = 0;
			int cp = 0;
			while(cp < loc) {
				tset = tset*10;
				tset = tset + (buf[cp++] - '0');
			}
			mode = 'm';
			if (ui32TempValueC < tset) {
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3, 0x8);
			} else {
				GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3, 0x2);
			}
		}
	}

}

void Timer0IntHandler(void)
{
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	ADCProcessorTrigger(ADC0_BASE, 1);

	while(!ADCIntStatus(ADC0_BASE, 1, false))  { }
	ADCIntClear(ADC0_BASE, 1);
	ADCSequenceDataGet(ADC0_BASE, 1, ui32ADC0Value);
	ui32TempAvg = (ui32ADC0Value[0] + ui32ADC0Value[1] + ui32ADC0Value[2] + ui32ADC0Value[3] + 2)/4;
	if (ui32TempAvg > 3000) ui32TempAvg = 3000;
	if (ui32TempAvg < 300) ui32TempAvg = 300;
	ui32TempValueC = ((ui32TempAvg - 300)) / 2700.0f * 100 ;
	if (ui32TempValueC < tset) {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3, 0x8);
	} else {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_3, 0x2);
	}

	if (mode == 'm') {
		UARTprintf("Current temperature = %u %cC, Set Temp = %u %cC\n", ui32TempValueC, 176, tset, 176);
	}
}
