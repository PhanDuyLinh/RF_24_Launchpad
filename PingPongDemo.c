/*
 Copyright (C) 2011 J. Coliz <maniacbug@ymail.com>
 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 version 2 as published by the Free Software Foundation.
 */

/**
 * Example for Getting Started with nRF24L01+ radios.
 *
 * This is an example of how to use the RF24 class.  Write this sketch to two
 * different nodes.  Put one of the nodes into 'transmit' mode by connecting
 * with the serial monitor and sending a 'T'.  The ping node sends the current
 * time to the pong node, which responds by sending the value back.  The ping
 * node can then see how long the whole cycle took.
 */

#include <stdint.h> // Variable definitions for the C99 standard.
#include <stdbool.h> // Boolean definitions for the C99 standard.
#include "inc/tm4c123gh6pm.h" // Definitions for the interrupt and register assignments.
#include "inc/hw_memmap.h" // Memory map definitions of the Tiva C Series device.
#include "inc/hw_types.h" // Definitions of common types and macros.
#include "inc/hw_gpio.h"
#include "driverlib/systick.h"
#include "driverlib/sysctl.h" // Definitions and macros for System Control API of DriverLib.
#include "driverlib/interrupt.h" // Defines and macros for NVIC Controller API of DriverLib.
#include "driverlib/gpio.h" // Definitions and macros for GPIO API of DriverLib.
#include "driverlib/timer.h" // Defines and macros for Timer API of DriverLib.
#include "driverlib/pin_map.h" //Mapping of peripherals to pins for all parts.
#include "driverlib/uart.h" // Definitions and macros for UART API of DriverLib.
#include "driverlib/ssi.h" // Prototypes for the SSI/SPI routines.
#include "utils/uartstdio.h" // Prototypes for the UART console functions.
							 // Needs to add "utils/uartstdio.c" through a relative link.
#include "RF24.h"

// function prototypes
void init_LEDs(void);
void init_timer(void);
void init_UART(void);
void Timer0_ISR(void);
void init_SPI(void);
void init_systick(void);

extern void UARTStdioIntHandler(void);

#define TIMER0_FREQ    2 // Freqency in Hz, heartbeat timer
#define RED_LED    GPIO_PIN_1
#define BLUE_LED    GPIO_PIN_2
#define GREEN_LED    GPIO_PIN_3
#define UART0_BAUDRATE    115200 // UART baudrate in bps
#define NUM_DISP_TEXT_LINE    4

extern uint8_t cur_LED;

RF24_data g_RF24_data =
{
	SYSCTL_PERIPH_GPIOA,	// 0xF0000800
	GPIO_PORTA_BASE,		// 0x40004000
	GPIO_PIN_6,				// 0x00000040
	SYSCTL_PERIPH_GPIOA,	// 0xF0000800
	GPIO_PORTA_BASE,		// 0x40004000
	GPIO_PIN_7,				// 0x00000080
	true,		// RF24_WB
	false,		// RF24_PV
	32,			// Payload Size (0..32)
	false,		// Ack Payload Available
	false,		// Dynamic Payloads Enabled
	0			// Pipe0 Reading Address
};

// The various roles supported by this sketch
typedef enum { role_ping_out = 1, role_pong_back } role_e;
// The role of the current running sketch
role_e role = role_pong_back;

// The debug-friendly names of those roles
const char* role_friendly_name[] = { "invalid", "Ping out", "Pong back"};

const char *disp_text[NUM_DISP_TEXT_LINE] = {
		"\n",
		"Ping Pong Demo\n",
		"H: help, T: transmit, R: receive\n",
		"> " };

// global variables
uint8_t cur_LED = RED_LED;

uint32_t sys_clock;

// Radio pipe addresses for the 2 nodes to communicate.
const uint64_t pipes[2] = { 0xF0F0F0F0E1LL, 0xF0F0F0F0D2LL };

int main()
{

		uint32_t i;
		unsigned char user_cmd;

		SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
			SYSCTL_XTAL_16MHZ);
//		SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN); // 40 MHz
		sys_clock = SysCtlClockGet();

		init_LEDs();
		init_UART();
		init_timer();
		init_SPI();
		init_systick();

		// Enable the processor to respond to interrupts.
		IntMasterEnable();
		SysTickEnable();

		// Start the timer by enabling operation of the timer module.
		TimerEnable(TIMER0_BASE, TIMER_A);

		UARTprintf("\r\n>Device Ready\r\n");

		//
		// Setup and configure rf radio
		//
		RF24_begin();

		// optionally, increase the delay between retries & # of retries
		RF24_setRetries( 15, 15 );

		// optionally, reduce the payload size.  seems to
		// improve reliability
//		RF24_setPayloadSize(8);

		//
		// Open pipes to other nodes for communication
		//

		// This simple sketch opens two pipes for these two nodes to communicate
		// back and forth.
		// Open 'our' pipe for writing
		// Open the 'other' pipe for reading, in position #1 (we can have up to 5 pipes open for reading)

		RF24_openWritingPipe(pipes[1]);
		RF24_openReadingPipe(1,pipes[0]);

		RF24_startListening();

		//
		// Dump the configuration of the rf unit for debugging
		//
		RF24_printDetails();

		// Initial display on terminal.
			for(i=0; i<NUM_DISP_TEXT_LINE; i++)
				UARTprintf(disp_text[i]);

    while(1)
    {


    	// Read user inputs from UART if available.
		if(UARTRxBytesAvail())
	        user_cmd = UARTgetc();
		else
			user_cmd = 0;

		switch(user_cmd){
		case '\r':
		case ' ':
		case 'H':
		case 'h':
			for(i=0; i<NUM_DISP_TEXT_LINE; i++)
				UARTprintf(disp_text[i]);
			break;
		case 'T':
		case 't':
			if (role ==role_pong_back){
				UARTprintf("\nNow changing to transmit mode...\n\r");
				cur_LED = GREEN_LED;
				role = role_ping_out;
				RF24_openWritingPipe(pipes[0]);
				RF24_openReadingPipe(1,pipes[1]);
				UARTprintf("\n> ");
			}
			break;
		case 'R':
		case 'r':
			if (role == role_ping_out){
				UARTprintf("\nNow changing to receive mode...\n\r");
				cur_LED = RED_LED;
				role = role_pong_back;
				RF24_openWritingPipe(pipes[1]);
				RF24_openReadingPipe(1,pipes[0]);
				UARTprintf("\n> ");
			}
			break;
		}


    	 if (role == role_ping_out)
    	  {
    	    // First, stop listening so we can talk.
    	    RF24_stopListening();

    	    // Take the time, and send it.  This will block until complete
    	    uint32_t time = millis();
    	    UARTprintf("Now sending %u...",time);
    	    bool ok = RF24_write( &time, sizeof(uint32_t) );

    	    if (ok)
    	      UARTprintf("ok...");
    	    else
    	      UARTprintf("failed.\n\r");

    	    // Now, continue listening
    	    RF24_startListening();

    	    // Wait here until we get a response, or timeout (250ms)
    	    uint32_t started_waiting_at = millis();
    	    bool timeout = false;
    	    while ( ! RF24_available() && ! timeout )
    	      if (millis() - started_waiting_at > 200 )
    	        timeout = true;

    	    // Describe the results
    	    if ( timeout )
    	    {
    	      UARTprintf("Failed, response timed out.\n\r");
    	    }
    	    else
    	    {
    	      // Grab the response, compare, and send to debugging spew
    	    	uint32_t got_time;
    	      RF24_read( &got_time, sizeof(uint32_t) );

    	      // Spew it
    	      UARTprintf("Got response %u, round-trip delay: %u\n\r",got_time,millis()-got_time);
    	    }

    	    // Try again 1s later
    	    delay(1000);
    	  }

    	  //
    	  // Pong back role.  Receive each packet, dump it out, and send it back
    	  //

    	  if ( role == role_pong_back )
    	  {
    	    // if there is data ready
    	    if ( RF24_available() )
    	    {
    	      // Dump the payloads until we've gotten everything
    	      uint32_t got_time;
    	      bool done = false;
    	      while (!done)
    	      {
    	        // Fetch the payload, and see if this was the last one.
    	        done = RF24_read( &got_time, sizeof(uint32_t) );

    	        // Spew it
    	        UARTprintf("Got payload %u...",got_time);

    		// Delay just a little bit to let the other unit
    		// make the transition to receiver
    		delay(50);
    	      }

    	      // First, stop listening so we can talk
    	      RF24_stopListening();

    	      // Send the final one back.
    	      RF24_write( &got_time, sizeof(uint32_t) );
    	      UARTprintf("Sent response.\n\r");

    	      // Now, resume listening so we catch the next packets.
    	      RF24_startListening();
    	    }
    	  }
    }
}

void init_LEDs(void)
{
	// Enable and configure LED peripheral.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF); // Enable GPIO Port F.
	// Three onboard LEDs, R:PF1, B:PF2, G:PF3.
	GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3);
}


void init_systick(void)
{
		SysTickPeriodSet(SysCtlClockGet()/1000);
		SysTickIntRegister(SysTickISR);
		SysTickIntEnable();
}


void init_SPI(void)
{
	uint32_t ui32DataRx;
	// Enable peripheral for SSI/SPI.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_SSI0);

	// Configure the muxing and GPIO settings to bring the SSI functions out to the pins
	GPIOPinConfigure(GPIO_PA2_SSI0CLK);
	GPIOPinConfigure(GPIO_PA3_SSI0FSS);
	GPIOPinConfigure(GPIO_PA4_SSI0RX);
	GPIOPinConfigure(GPIO_PA5_SSI0TX);

	GPIOPinTypeSSI(GPIO_PORTA_BASE, GPIO_PIN_5 | GPIO_PIN_4 | GPIO_PIN_3 | GPIO_PIN_2);

	// Configure and enable the SSI port.  Use SSI0, system clock, master mode, 1MHz SSI frequency, and 8-bit data
	SSIConfigSetExpClk(SSI0_BASE, SysCtlClockGet(), SSI_FRF_MOTO_MODE_0, SSI_MODE_MASTER, 1000000, 8);

	// Enable the SSI0 module.
	SSIEnable(SSI0_BASE);

	 // Read any residual data from the SSI port.  This makes sure the receive
	    // FIFOs are empty, so we don't read any unwanted junk.  This is done here
	    // because the TI SSI mode is full-duplex, which allows you to send and
	    // receive at the same time.  The SSIDataGetNonBlocking function returns
	    // "true" when data was returned, and "false" when no data was returned.
	    // The "non-blocking" function checks if there is any data in the receive
	    // FIFO and does not "hang" if there isn't.
	    //
	    while(SSIDataGetNonBlocking(SSI0_BASE, &ui32DataRx))
	    {
	    }
}


void init_timer(void)
{
	// Enable and configure Timer0 peripheral.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
	// Configure as a 32-bit timer in periodic mode.
	TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
	// Initialize timer load register.
	TimerLoadSet(TIMER0_BASE, TIMER_A, SysCtlClockGet()/TIMER0_FREQ -1);

	// Registers a function to be called when the interrupt occurs.
	IntRegister(INT_TIMER0A, Timer0_ISR);
	// The specified interrupt is enabled in the interrupt controller.
	IntEnable(INT_TIMER0A);
	// Enable the indicated timer interrupt source.
	TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
}


void init_UART(void)
{
	// Enable and configure UART0 for debugging printouts.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
	SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
	GPIOPinConfigure(GPIO_PA0_U0RX);
	GPIOPinConfigure(GPIO_PA1_U0TX);
	GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

	// Registers a function to be called when the interrupt occurs.
	IntRegister(INT_UART0, UARTStdioIntHandler);
	UARTStdioConfig(0, UART0_BAUDRATE, SysCtlClockGet());
}


// Timer0 interrupt service routine
void Timer0_ISR(void)
{
	// Clear the timer interrupt.
	TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

	// Blink LED. Read the current state of GPIO pins and write back the opposite state.
	if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3)) {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, 0);
	}
	else {
		GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3, cur_LED);
	}
}





