//*****************************************************************************
//
// usb_dev_mame.c - Main routines for a Mame control device.
//
// Copyright (c) 2011-2013 Texas Instruments Incorporated.  All rights reserved.
// Software License Agreement
// 
// Texas Instruments (TI) is supplying this software for use solely and
// exclusively on TI's microcontroller products. The software is owned by
// TI and/or its suppliers, and is protected under applicable copyright
// laws. You may not combine this software with "viral" open-source
// software in order to form a larger program.
// 
// THIS SOFTWARE IS PROVIDED "AS IS" AND WITH ALL FAULTS.
// NO WARRANTIES, WHETHER EXPRESS, IMPLIED OR STATUTORY, INCLUDING, BUT
// NOT LIMITED TO, IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE. TI SHALL NOT, UNDER ANY
// CIRCUMSTANCES, BE LIABLE FOR SPECIAL, INCIDENTAL, OR CONSEQUENTIAL
// DAMAGES, FOR ANY REASON WHATSOEVER.
// 
// This is part of revision 1.0 of the EK-LM4F232 Firmware Package.
//
//*****************************************************************************

#include <stdbool.h>
#include <stdint.h>
#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_gpio.h"
#include "inc/hw_sysctl.h"
#include "driverlib/debug.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "inc/hw_i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/i2c.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/qei.h"
#include "driverlib/interrupt.h"
#include "usblib/usblib.h"
#include "usblib/usbhid.h"
#include "usblib/device/usbdevice.h"
#include "usblib/device/usbdhid.h"
#include "usblib/device/usbdhidmame.h"
#include "usb_mame_structs.h"
#include "Mame_pins.h"
#include "utils/uartstdio.h"
#include"driverlib/adc.h"

//*****************************************************************************
//
//! \addtogroup example_list
//! <h1>USB HID Mame Control Device (usb_dev_mame)</h1>
//!
//! This example application turns the evaluation board into a USB volume control
//! supporting the Human Interface Device class.  When the left button is
//! pressed the volume is increased, when the right button is pressed the volume is decreased
//!
//! The device implemented by this application also supports USB remote wakeup
//! allowing it to request the host to reactivate a suspended bus.  If the bus
//! is suspended (as indicated on the application display), pressing the
//! push button will request a remote wakeup assuming the host has not
//! specifically disabled such requests.
//!
//
//*****************************************************************************

//*****************************************************************************
//
// The system tick timer period.
//
//*****************************************************************************
#define SYSTICKS_PER_SECOND     500  // 1ms systick rate
int g_ui32SysTickCount = 0;

//*****************************************************************************
//
// This global indicates whether or not we are connected to a USB host.
//
//*****************************************************************************
volatile bool g_bConnected = false;

//*****************************************************************************
//
// This global indicates whether or not the USB bus is currently in the suspend
// state.
//
//*****************************************************************************
volatile bool g_bSuspended = false;

//*****************************************************************************
//
// Global button arrays and ticks hold button data from each loop for debouncing
//
//*****************************************************************************
volatile signed char g_ui8Pad1[8];
volatile signed char g_ui8Pad2[3];

//*****************************************************************************
//
// Global variable indicating if the board is in programing or GPIO mode.
//
//*****************************************************************************
volatile bool g_bProgramMode;

//*****************************************************************************
//
// This enumeration holds the various states that the device can be in during
// normal operation.
//
//*****************************************************************************
volatile enum
{
    //
    // Unconfigured.
    //
    STATE_UNCONFIGURED,

    //
    // No keys to send and not waiting on data.
    //
    STATE_IDLE,

    //
    // Waiting on data to be sent out.
    //
    STATE_SENDING
}
g_eCustomHidState = STATE_UNCONFIGURED;

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
}
#endif

//*****************************************************************************
//
// Handles asynchronous events from the HID driver.
//
// \param pvCBData is the event callback pointer provided during
// USBDHIDCustomHidInit().  This is a pointer to our device structure
// (&g_sCustomHidDevice).
// \param ui32Event identifies the event we are being called back for.
// \param ui32MsgData is an event-specific value.
// \param pvMsgData is an event-specific pointer.
//
// This function is called by the HID driver to inform the application
// of particular asynchronous events related to operation of the HID
// device.
//
// \return Returns 0 in all cases.
//
//*****************************************************************************
uint32_t
CustomHidHandler(void *pvCBData, uint32_t ui32Event, uint32_t ui32MsgData,
                void *pvMsgData)
{
    switch (ui32Event)
    {
        //
        // The host has connected to us and configured the device.
        //
        case USB_EVENT_CONNECTED:
        {
            g_bConnected = true;
            g_bSuspended = false;
            break;
        }

        //
        // The host has disconnected from us.
        //
        case USB_EVENT_DISCONNECTED:
        {
            g_bConnected = false;
            break;
        }

        //
        // We receive this event every time the host acknowledges transmission
        // of a report. It is used here purely as a way of determining whether
        // the host is still talking to us or not.
        //
        case USB_EVENT_TX_COMPLETE:
        {
            //
            // Enter the idle state since we finished sending something.
            //
            g_eCustomHidState = STATE_IDLE;
            break;
        }

        //
        // This event indicates that the host has suspended the USB bus.
        //
        case USB_EVENT_SUSPEND:
        {
            g_bSuspended = true;
            break;
        }

        //
        // This event signals that the host has resumed signalling on the bus.
        //
        case USB_EVENT_RESUME:
        {
            g_bSuspended = false;
            break;
        }

        //
        // We ignore all other events.
        //
        default:
        {
            break;
        }
    }

    return(0);
}

//*****************************************************************************
//
// Send Data if necessary
//
//*****************************************************************************
void SendHIDReport(char ReportNum, signed char ReportData[])
{
	g_eCustomHidState = STATE_SENDING;
	USBDHIDCustomHidStateChange((void *)&g_sCustomHidDevice,ReportNum,ReportData);
	while(g_eCustomHidState != STATE_IDLE)
	{
	}
}

//read specified register on slave device
uint32_t I2CReceive(uint32_t slave_addr, uint8_t reg)
{
    //specify that we are writing (a register address) to the
    //slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, false);

    //specify register to be read
    I2CMasterDataPut(I2C0_BASE, reg);

    //send control byte and register address byte to slave device
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_BURST_SEND_START);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    //specify that we are going to read from slave device
    I2CMasterSlaveAddrSet(I2C0_BASE, slave_addr, true);

    //send control byte and read from the register we
    //specified
    I2CMasterControl(I2C0_BASE, I2C_MASTER_CMD_SINGLE_RECEIVE);

    //wait for MCU to finish transaction
    while(I2CMasterBusy(I2C0_BASE));

    //return data pulled from the specified register
    return I2CMasterDataGet(I2C0_BASE);
}

#define ACCEL_SLAVE_ADDR 0x68
#define XOUT8 0x3B
#define YOUT8 0x3D
#define ZOUT8 0x3F
#define GX8 0x43
#define GY8 0x45
#define GZ8 0x47

uint8_t ReadAccel(uint8_t reg)
{
    uint8_t accelData = I2CReceive(ACCEL_SLAVE_ADDR, reg);

    return accelData;
}


int32_t clip(int32_t v, int32_t lo) {
	if (v > 127) return 127;
	else if (v < lo) return lo;
	else return v;
}

float ADC_scaler[4] = {0.0389, 0.0378, 0.0364, 0.0571};
float ADC_callib_scaler[4] = {0.0609, 0.0592, 0.0570, 0.0895};
int32_t ADC_offset[4] = {809,1110,995,693};
uint32_t ADC_max[4] = {0,0,0,0};
uint32_t ADC_min[4] = {0,0,0,0};
#define DESIRED_MAX 127
#define DEADZONE 50
#define MAX_AVG_TICK 8

bool g_bCallibMode = false;
int F_PIN_4_PREV = 1;

//*****************************************************************************
//
// Store switch states in buffers
//
//*****************************************************************************
int8_t mpu_val = -1;
int32_t c = 0;
volatile uint32_t ui32ADC0Value[MAX_AVG_TICK][4];
volatile int32_t ui32MPUValue[MAX_AVG_TICK][3];

void
StoreSwitches(void)
{

	ADCProcessorTrigger(ADC0_BASE, 0);

	while(!ADCIntStatus(ADC0_BASE, 0, false))  { }
	ADCIntClear(ADC0_BASE, 0);
	ADCSequenceDataGet(ADC0_BASE, 0, ui32ADC0Value[g_ui32SysTickCount % MAX_AVG_TICK]);

	uint8_t tmp = 0x0;
	tmp = (~GPIOPinRead(GPIO_PORTB_BASE, GPIO_PIN_0 | GPIO_PIN_1) << 2) & 0x0C; // Two push buttons
	tmp = tmp | ((~GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5) >> 4) & 0x03); // Two more buttons
	g_ui8Pad1[7] = 0x0;
	g_ui8Pad1[7] |= (tmp & 0x2) >> 1;
	g_ui8Pad1[7] |= (tmp & 0x8) >> 2;
	g_ui8Pad1[7] |= (tmp & 0x4);
	g_ui8Pad1[7] |= (tmp & 0x1) << 3;

	ui32MPUValue[g_ui32SysTickCount % MAX_AVG_TICK][0] = (int8_t)ReadAccel(XOUT8);
	ui32MPUValue[g_ui32SysTickCount % MAX_AVG_TICK][1] = (int8_t)ReadAccel(ZOUT8);
	ui32MPUValue[g_ui32SysTickCount % MAX_AVG_TICK][2] = (int8_t)ReadAccel(GX8);

	c++;
}

//*****************************************************************************
//
// Debounce switches
//
//*****************************************************************************
void
DebounceSwitches(void)
{
	int i;
	for (i = 0; i < 4; ++i) {
		int j;
		uint32_t tmp = 0;
		for (j = 0; j < MAX_AVG_TICK; ++j) {
			tmp += ui32ADC0Value[j][i];
		}
		tmp /= MAX_AVG_TICK;
		g_ui8Pad1[i] = (uint8_t)clip(((int32_t)(((int32_t)tmp - ADC_offset[i]) * (g_bCallibMode ? ADC_callib_scaler[i] : ADC_scaler[i]))) - DEADZONE + (g_bCallibMode ? 0 : DESIRED_MAX / 2), g_bCallibMode ? 0 : (DESIRED_MAX / 2));
	}

	for (i = 0; i < 3; ++i) {
		int j;
		int32_t tmp = 0;
		for (j = 0; j < MAX_AVG_TICK; ++j) {
			tmp += ui32MPUValue[j][i];
		}
		tmp /= MAX_AVG_TICK;
		g_ui8Pad1[4 + i] = (i == 2 && tmp < 0) ? 0 : tmp;
	}
}

//*****************************************************************************
//
// Check buttons
//
//*****************************************************************************
void
CustomHidChangeHandler(void)
{

	// If the bus is suspended then resume it.
	//
	if(g_bSuspended)
	{
		USBDHIDCustomHidRemoteWakeupRequest((void *)&g_sCustomHidDevice);
	}

	//Get debounced switch states
	DebounceSwitches();


	//SendHIDReport(2,g_ui8Pad1);
	SendHIDReport(1,g_ui8Pad1);

}



//*****************************************************************************
//
// This is the interrupt handler for the SysTick interrupt.  It is used to
// update our local tick count and place the board in programming mode when appropriate
//
//*****************************************************************************
void
SysTickIntHandler(void)
{

	g_ui32SysTickCount++;
	StoreSwitches();

	int pinval = GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_4);
    //
    // If the left button has been pressed, and was previously not pressed,
    // start the process of changing the behavior of the JTAG pins.
    //
    if(!g_bCallibMode  && !pinval && F_PIN_4_PREV)
    {
		//
		// Change the LED to BLUE to indicate that the pins are in JTAG mode.
		//
		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 4);

		//
		// Mark device in programming mode
		//
		g_bCallibMode = true;
		F_PIN_4_PREV = pinval;
		return;
    }

    if (g_bCallibMode) {
    	if (!pinval && F_PIN_4_PREV) {
    		g_bCallibMode = false;
    		int i;
    		for (i = 0; i < 4; ++i) {
    			ADC_offset[i] = ADC_min[i];
    			ADC_callib_scaler[i] = (float)(DESIRED_MAX + DEADZONE) / (ADC_max[i] - ADC_min[i]);
    			ADC_scaler[i] = (float)(DESIRED_MAX / 2 + DEADZONE) / (ADC_max[i] - ADC_min[i]);
    			ADC_max[i] = 0;
    			ADC_min[i] = 0;
    		}
    		F_PIN_4_PREV = pinval;
    		GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 8);
    		return;
    	}

    	// Set the max and min during callibration
    	int i;
    	for (i = 0; i < 4; ++i) {
    		if (ADC_min[i] == 0) {
    			ADC_min[i] = ui32ADC0Value[0][i];
    		} else {
    			ADC_min[i] = (ADC_min[i] > ui32ADC0Value[0][i]) ? ui32ADC0Value[0][i] : ADC_min[i];
    		}

    		if (ADC_max[i] == 0) {
    			ADC_max[i] = ui32ADC0Value[0][i];
    		} else {
    			ADC_max[i] = (ADC_max[i] < ui32ADC0Value[0][i]) ? ui32ADC0Value[0][i] : ADC_max[i];
    		}
    	}

    }

    F_PIN_4_PREV = pinval;
}



void init_adc() {
	SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD); //ADC pin
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE); //ADC pin
	ADCHardwareOversampleConfigure(ADC0_BASE, 4);
	ADCSequenceConfigure(ADC0_BASE, 0, ADC_TRIGGER_PROCESSOR, 0);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 0, ADC_CTL_CH6);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 1, ADC_CTL_CH7);
	ADCSequenceStepConfigure(ADC0_BASE, 0, 2, ADC_CTL_CH5);
	ADCSequenceStepConfigure(ADC0_BASE,0,3,ADC_CTL_CH4|ADC_CTL_IE|ADC_CTL_END);
	ADCSequenceEnable(ADC0_BASE, 0);
	ADCIntClear(ADC0_BASE, 0);

}

//initialize I2C module 0
//Slightly modified version of TI's example code
void InitI2C0(void)
{
    //enable I2C module 0
    SysCtlPeripheralEnable(SYSCTL_PERIPH_I2C0);

    //reset module
    SysCtlPeripheralReset(SYSCTL_PERIPH_I2C0);

    // Configure the pin muxing for I2C0 functions on port B2 and B3.
    GPIOPinConfigure(GPIO_PB2_I2C0SCL);
    GPIOPinConfigure(GPIO_PB3_I2C0SDA);

    // Select the I2C function for these pins.
    GPIOPinTypeI2CSCL(GPIO_PORTB_BASE, GPIO_PIN_2);
    GPIOPinTypeI2C(GPIO_PORTB_BASE, GPIO_PIN_3);

    // Enable and initialize the I2C0 master module.  Use the system clock for
    // the I2C0 module.  The last parameter sets the I2C data transfer rate.
    // If false the data rate is set to 100kbps and if true the data rate will
    // be set to 400kbps.
    I2CMasterInitExpClk(I2C0_BASE, SysCtlClockGet(), false);

    //clear I2C FIFOs
    HWREG(I2C0_BASE + I2C_O_FIFOCTL) = 80008000;
}


//*****************************************************************************
//
// This is the main loop that runs the application.
//
//*****************************************************************************
int
main(void)
{
    bool bLastSuspend;

    //
    // Enable lazy stacking for interrupt handlers.  This allows floating-point
    // instructions to be used within interrupt handlers, but at the expense of
    // extra stack usage.
    //
    FPULazyStackingEnable();

    //
    // Set the clocking to run from the PLL at 50MHz.
    //
    SysCtlClockSet(SYSCTL_SYSDIV_4 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
                       SYSCTL_XTAL_16MHZ);


	// Initialize the inputs
    PortFunctionInit();

    init_adc();
    InitI2C0();

    // Set the system tick to control how often the buttons are polled.
	SysTickEnable();
	SysTickIntEnable();
	SysTickPeriodSet(SysCtlClockGet() / SYSTICKS_PER_SECOND);



	// Set initial LED Status to RED to indicate not connected
	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);



    //
    // Not configured initially.
    //
    g_bConnected = false;
    g_bSuspended = false;
    bLastSuspend = false;
    g_bProgramMode = false;
    g_ui8Pad1[0] = 0x00;
    g_ui8Pad1[1] = 0x00;
    g_ui8Pad1[2] = 0x00;
    g_ui8Pad1[3] = 0x00;
    g_ui8Pad1[4] = 0x00;
    g_ui8Pad1[5] = 0x00;
    g_ui8Pad1[6] = 0x00;
    g_ui8Pad1[7] = 0x00;

    //
    // Initialize the USB stack for device mode. (must use force on the Tiva launchpad since it doesn't have detection pins connected)
    //
    USBStackModeSet(0, eUSBModeForceDevice, 0);

    //
    // Pass our device information to the USB HID device class driver,
    // initialize the USB
    // controller and connect the device to the bus.
    //
    USBDHIDCustomHidInit(0, &g_sCustomHidDevice);


    IntMasterEnable();

    //
    // The main loop starts here.  We begin by waiting for a host connection
    // then drop into the main volume handling section.  If the host
    // disconnects, we return to the top and wait for a new connection.
    //
    while(1)
    {
        //
        // Wait here until USB device is connected to a host.
        //
        while(!g_bConnected)
        {

        	//Set the onboard LED to red when not connected
        	GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);
        }

        //
        // Update the status to green when connected.
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 8);

        //
        // Enter the idle state.
        //
        g_eCustomHidState = STATE_IDLE;

        //
        // Assume that the bus is not currently suspended if we have just been
        // configured.
        //
        bLastSuspend = false;

        //
        // Keep checking the volume buttons for as
        // long as we are connected to the host. This is a simple example
        //
        while(g_bConnected)
        {
            //
			// Has the suspend state changed since last time we checked?
			//
			if(bLastSuspend != g_bSuspended)
			{
				//
				// Yes - the state changed so update the display.
				//
				bLastSuspend = g_bSuspended;
				if(bLastSuspend)
				{
					//Set the onboard LED to red when not connected
					GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 2);
				}
				else
				{
				    // Update the status to green when connected.
					GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 8);
				}
			}

		    if(g_ui32SysTickCount>=MAX_AVG_TICK  && !g_bProgramMode)
		    {
		    	//Reset systick counter
		    	g_ui32SysTickCount = 0;

				//Check inputs and act accordingly
		    	CustomHidChangeHandler();
		    }
        }
    }
}
