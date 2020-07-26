/* File    : EFM8LB1_Configurable_Logic_Biphase_Mark_Decoder.c
*  Author  : Mathieu Schnegg
*  Date    : 02.05.2020
*
* Description : Firmware of the sensors
*
* This file is based on the :
*  - EFM8LB1_Configurable_Logic_Biphase_Mark_Decoder
*  - EFM8LB1_I2C_Slave_FIFO
*
* Modifications : Date / Author / Purpose
*
* Platform : EFM8LB1
*/

// Copyright 2015 Silicon Laboratories, Inc.
// http://developer.silabs.com/legal/version/v11/Silicon_Labs_Software_License_Agreement.txt
//
// Program Description:
//
// This program demonstrates how to use the Configurable Logic to
// receive Biphase Mark coded data via the SPI module.  The Biphase
// Mark code is defined as follows:
//    '0' : no transition in the middle of a bit
//    '1' : a falling OR rising transition in the middle of a bit
// And there is always a transition at the end of the bit.
// BMC is used in protocols like the USB 3.1 CC protocol, S/PDIF.
//
// The incoming data rate is assumed to be 6 Mbps but the
// clock is recovered (or re-synchronized) at every bit transition.
//
//
// Resources:
//   SYSCLK - 72 MHz HFOSC1 / 1
//   SPI0   - operated as slave
//   Timer2 - Its timeout occurs 2 SYSCLK after Timer 4 and is used to generate the
//            leading edge of the SPI clock.
//   Timer4 - Used to timeout at 0.75 bit period to trigger decoding of current BMC bit
//          - Its opposite edge half a SYSCLK later is used to capture the logic
//          - of the current BMC logic level
//   Timer5 - Used to detect BMC idle condition - occurs when there is no transition in
//            BMC for more than 1 bit period
//   P1.1   - BMC encoded input - assumed to be at a rate of 300 kbps
//   P1.0   - SCK output from CLU (for debugging and observation)
//   P2.5   - MOSI output from CLU into SPI (for debugging and observation)
//   All 4 CLUs
//   Timer 3 is used to help simulate the BMC input by bit banging

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <SI_EFM8LB1_Register_Enums.h>                  // SFR declarations
#include "InitDevice.h"
#include "TS4231.h"
#include "EFM8LB1_I2C_Slave.h"

TS4231 device = {false};
uint8_t txbyte, j;
uint32_t i = 0;
uint8_t beam[6] = {0};
uint8_t SI_SEG_DATA buffer[6] = {0};
bool parity = false;
bool done = true;
bool ready = true;
uint8_t count = 0;

// I2C
uint8_t i2cReceivedData;               // Global holder for I2C data.
                                       // All receive data is written
                                       // here;

bool dataReady = 0;                    // Set to '1' by the I2C ISR
                                       // when a new data byte has been
                                       // received.

bool txDataReady = 1;                  // Set to '1' indicate that Tx data ready.
uint8_t sendDataValue = 0;             // Transmit the data value
uint8_t sendDataCnt = 0;               // Transmit data counter. Count the Tx data
                                       // in a I2C transaction.

//-----------------------------------------------------------------------------
// SiLabs_Startup() Routine
// ----------------------------------------------------------------------------
// This function is called immediately after reset, before the initialization
// code is run in SILABS_STARTUP.A51 (which runs before main() ). This is a
// useful place to disable the watchdog timer, which is enable by default
// and may trigger before main() in some instances.
//-----------------------------------------------------------------------------
void SiLabs_Startup (void)
{
  // Disable the watchdog here
}


SI_INTERRUPT(TIMER5_ISR, TIMER5_IRQn)
{
	count = 0;	// Reset count

	SFRPAGE = 0x10;
	TMR5CN0 &= ~TMR5CN0_TF5H__BMASK;			// Clear Timer5 interrupt-pending flag
}

//-----------------------------------------------------------------------------
// main() Routine
// ----------------------------------------------------------------------------
int main(void)
{
	// Call hardware initialization routine
	WDT_0_enter_DefaultMode_from_RESET();
	PORTS_0_enter_DefaultMode_from_RESET();
	PORTS_1_enter_DefaultMode_from_RESET();
	PORTS_2_enter_DefaultMode_from_RESET();
	PBCFG_0_enter_DefaultMode_from_RESET();
	CIP51_0_enter_DefaultMode_from_RESET();
	CLOCK_0_enter_DefaultMode_from_RESET();

	// Wait for TS4231 to detect light
	while(!waitForLight());

	// Wait for TS4231 to be configured
	while(configDevice(&device)!=CONFIG_PASS);

	TIMER16_2_enter_DefaultMode_from_RESET();
	TIMER16_4_enter_DefaultMode_from_RESET();
	TIMER16_5_enter_DefaultMode_from_RESET();
	TIMER_SETUP_0_enter_DefaultMode_from_RESET();
	SPI_0_enter_DefaultMode_from_RESET();

	CLU_0_enter_DefaultMode_from_RESET();
	CLU_1_enter_DefaultMode_from_RESET();
	CLU_2_enter_DefaultMode_from_RESET();
	CLU_3_enter_DefaultMode_from_RESET();
	CL_0_enter_DefaultMode_from_RESET();
	CLU_enable();

	I2CSLAVE_0_enter_DefaultMode_from_RESET();
	INTERRUPT_0_enter_DefaultMode_from_RESET();

	IE_EA = 0;
	SFRPAGE = PG3_PAGE;
	I2C0CN0 &= ~I2C0CN0_BUSY__BMASK;		// Clear BUSY bit
	I2C0FCN0 |= I2C0FCN0_RFLSH__FLUSH | I2C0FCN0_TFLSH__FLUSH;
	IE_EA = 1;

	// Program loop
	while(1)
	{
		// Signal idle detection
		if((P1_B1==1) && done)
			done = false;		// Ready for detection

		// Signal detected
		if((P1_B1==0) && !done && !(I2C0STAT & 0x40))
		{
			SFRPAGE = 0x10;
			TMR4CN0_TR4 = 1; 	// Start Timer 4
			TMR2CN0_TR2 = 1; 	// Start Timer 2
			TMR5CN0_TR5 = 1; 	// Start Timer 5

			SFRPAGE = 0x00;
			SPI0CFG |= 0x10;
			SPI0CFG &= ~0x20;
			SPI0FCN0 = SPI0FCN0 | 0x04;
			//SPI0CN0 |= 0x04;
			SPI0CN0_SPIEN = 1; 	// Enable SPI

			// Filter and read signal
			while((count<6) && (P1_B1==0))
			{
				while(!SPI0CN0_SPIF)
				{
					if(P1_B1)
						break;
				}

				SPI0CN0_SPIF = 0;	// Reset interrupt flag
				buffer[count++] = SPI0DAT;
			}

			SFRPAGE = 0x00;
			SPI0CN0_SPIEN = 0;	// Disable SPI
			SPI0CN0_SPIF = 0;	// Reset interrupt flag

			SFRPAGE = 0x10;
			TMR4CN0_TR4 = 0; 	// Stop Timer 4
			TMR2CN0_TR2 = 0; 	// Stop Timer 2
			TMR5CN0_TR5 = 0; 	// Stop Timer 5

			I2C0FCN0 |= I2C0FCN0_TFLSH__FLUSH;

			// Save data
			if(count<6)
			{
				beam[0]=0;
				beam[1]=0;
				beam[2]=0;
				beam[3]=0;
				beam[4]=0;
				beam[5]=0;
			}
			else
			{
				// Save data
				if(parity)
				{
					beam[0]=buffer[1];
					beam[1]=buffer[2];
					beam[2]=buffer[3];
				}
				else
				{
					beam[3]=buffer[1];
					beam[4]=buffer[2];
					beam[5]=buffer[3];
				}
			}

			// Toggle parity
			parity ^= 1;

			// Debounce envelope signal : ~100us
			for(i=0; i<100; i++)
				NOP();

			CLU_enable();
			count = 0;
			done = true;		// Ready for idle mode
		}
	}
}
