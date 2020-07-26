/* File       : ts4231.h
 * Author     : Mathieu Schnegg
 * Created on : Jun 18, 2020
 *
 * Description : This is an adapted version of the library for
 * 				 configuring the Triad Semiconductor TS4231 Light
 * 				 to Digital converter. See copyright below.
 *
 * Modifications : Date / Author / Reason
 *
 */
/*******************************************************************
 Copyright (C) 2017 Triad Semiconductor

 ts4231.h - Library for configuring the Triad Semiconductor
 TS4231 Light to Digital converter.
 Created by : John Seibel
 *******************************************************************/

//IMPORTANT NOTES:
//1)  If porting the TS4231 library code to a non-Arduino architecture,
//    be sure that the INPUT ports assigned to the E and D signals are configured as
//    floating inputs with NO pull-up or pull-down function.  Using a pull-up or
//    pull-down function on the inputs will cause the TS4231 to operate incorrectly.
//2)  The TS4231 library omits delays between E and D signal transitions when going
//    from S3_STATE to WATCH_STATE or SLEEP_STATE to WATCH_STATE in function
//    goToWatch() for the purpose of transitioning into WATCH_STATE as quickly as
//    possible.  If a microcontroller is being used that can change states on
//    the E and D outputs faster than approximately 100ns, the TS4231 datasheet
//    must be consulted to verify timing parameters are not being violated to
//    assure proper TS4231 operation.  A suitable solution would be to include
//    a short delay in function ts_digitalWrite() to allow enough time between
//    output pin signal changes to meet the TS4231 timing parameters as stated
//    in the datasheet.  See the ts_digitalWrite() function for more information.

#include <SI_EFM8LB1_Register_Enums.h>
#include "ts4231.h"
#include "InitDevice.h"


/*TS4231::TS4231(int device_E_pin, int device_D_pin)
{
	configured = false;
	E_PIN = device_E_pin;
	D_PIN = device_D_pin;
	ts_pinMode(E_PIN, INPUT);
	ts_pinMode(D_PIN, INPUT);
}*/

extern void ts_delayUs(uint32_t delay_val)
{
	//for (uint32_t i = delay_val; i; i--)
	uint32_t i = 0;
	/*TMR3CN0 &= ~(TMR3CN0_TR3__BMASK);
	TMR3H = (0xFF << TMR3H_TMR3H__SHIFT);
	TMR3L = (0xB8 << TMR3L_TMR3L__SHIFT);
	TMR3RLH = (0xFF << TMR3RLH_TMR3RLH__SHIFT);
	TMR3RLL = (0xB8 << TMR3RLL_TMR3RLL__SHIFT);
	TMR3CN0 |= TMR3CN0_TR3__RUN;*/
	if(delay_val==BUS_DRV_DLY)
		NOP();
	else if(delay_val==SLEEP_RECOVERY)
	{
		for(i=0; i<30; i++)
			NOP();
	}
	else if(delay_val==BUS_CHECK_DLY)
	{
		for(i=0; i<450; i++)
			NOP();
	}


	/*
	for (i = 0; i<delay_val; i++)
	{
		while (!(TMR3CN0 & TMR3CN0_TF3H__BMASK));
		TMR3CN0 &= ~TMR3CN0_TF3H__BMASK;
	}*/
}

extern void ts_pinMode(int pin, uint8_t mode)
{
	if(pin==E_PIN)
	{
		if(mode==INPUT)
		{
			P1MDOUT |= P1MDOUT_B1__OPEN_DRAIN;
		}
		else if(mode==OUTPUT)
		{
			P1MDOUT |= P1MDOUT_B1__PUSH_PULL;
		}
	}
	else if(pin==D_PIN)
	{
		if(mode==INPUT)
		{
			P1MDOUT |= P1MDOUT_B2__OPEN_DRAIN;
		}
		else if(mode==OUTPUT)
		{
			P1MDOUT |= P1MDOUT_B2__PUSH_PULL;
		}
	}
}

extern uint8_t ts_digitalRead(int pin)
{
	uint8_t read_val;

	if(pin==E_PIN)
	{
		read_val = P1_B1;
	}
	else if(pin==D_PIN)
	{
		read_val = P1_B2;
	}

	return read_val;
}

extern void ts_digitalWrite(int pin, uint8_t write_val)
{
	uint8_t i=0;
	if(pin==E_PIN)
	{
		P1_B1 = write_val;
	}
	else if(pin==D_PIN)
	{
		P1_B2 = write_val;
	}

	//for(i=0; i<10; i++)
		//NOP();
	//ts_delayUs(1);
	//for(int i=0; i<100; i++)
	//__asm__ __volatile__ ("nop\n\t");
	//delayMicroseconds(1);
	//A short delay function can be inserted here to extend the time between writes to
	//the E and D outputs if TS4231 timing parameters are being violated.  Consult
	//the TS4231 datasheet for more information on timing parameters.  It is recommended
	//that any added delay be no longer than approximately 1us.
}

/*extern unsigned long ts_millis()
{
	unsigned long current_time;

	current_time = millis();
	return current_time;
}*/

//Function waitForLight() should be executed after power-up and prior to
//configuring the device.  Upon power-up, D is a 0 and will output a 1
//when light is detected.  D will return to 0 at the end of light detection.
//This funciton looks for the falling edge of D to indicate that the end of
//light detection has occurred.
extern bool waitForLight(void)
{
	bool light = false;
	bool exit = false;

	if (checkBus() == S0_STATE)
	{
		ts_pinMode(E_PIN, INPUT);//
		ts_pinMode(D_PIN, INPUT);//

		//time0 = ts_millis();
		while (exit == false)
		{
			if (ts_digitalRead(D_PIN) > 0)
			{
				while (exit == false)
				{
					if (ts_digitalRead(D_PIN) == 0)
					{
						exit = true;
						light = true;
					}
					/*else if (ts_millis() > (time0 + light_timeout))
					{
						exit = true;
						light = false;
					}*/
					else
					{
						exit = false;
						light = false;
					}
				}
			}
			/*else if (ts_millis() > (time0 + light_timeout))
			{
				exit = true;
				light = false;
			}*/
			else
			{
				exit = false;
				light = false;
			}
		}
	}
	else light = true; //if not in state S0_state, light has already been detected

	return light;
}

extern bool goToSleep(TS4231 device)
{
	bool sleep_success;

	if (device.configured == false)
	sleep_success = false;
	else
	{
		switch (checkBus())
		{
			case S0_STATE:
			sleep_success = false;
			break;
			case SLEEP_STATE:
			sleep_success = true;
			break;
			case WATCH_STATE:
			ts_digitalWrite(E_PIN, LOW);
			ts_pinMode(E_PIN, OUTPUT);
			ts_delayUs(BUS_DRV_DLY);
			ts_pinMode(E_PIN, INPUT);
			ts_delayUs(BUS_DRV_DLY);
			if (checkBus() == SLEEP_STATE) sleep_success = true;
			else sleep_success = false;
			break;
			case S3_STATE:
			sleep_success = false;
			break;
			default:
			sleep_success = false;
			break;
		}
	}
	return sleep_success;
}

extern uint8_t configDevice(TS4231* device)
{
	uint8_t config_success = 0x00;
	uint16_t readback;

	device->configured = false;
	//ts_pinMode(D_PIN, INPUT);
	//ts_pinMode(E_PIN, INPUT);
	ts_pinMode(D_PIN, OUTPUT);//
	ts_pinMode(E_PIN, OUTPUT);//
	ts_digitalWrite(D_PIN, LOW);
	ts_digitalWrite(E_PIN, LOW);
	//ts_pinMode(E_PIN, OUTPUT);
	ts_delayUs(BUS_DRV_DLY);
	ts_digitalWrite(E_PIN, HIGH);
	ts_delayUs(BUS_DRV_DLY);
	ts_digitalWrite(E_PIN, LOW);
	ts_delayUs(BUS_DRV_DLY);
	ts_digitalWrite(E_PIN, HIGH);
	ts_delayUs(BUS_DRV_DLY);
	//ts_pinMode(D_PIN, OUTPUT);
	ts_delayUs(BUS_DRV_DLY);
	ts_digitalWrite(D_PIN, HIGH);
	ts_delayUs(BUS_DRV_DLY);
	//ts_pinMode(E_PIN, INPUT);
	//ts_pinMode(D_PIN, INPUT);
	if (checkBus() == S3_STATE)
	{
		writeConfig(CFG_WORD);
		readback = readConfig();
		if (readback == CFG_WORD)
		{
			device->configured = true;
			if (goToWatch(device))
			config_success = CONFIG_PASS;
			else
			config_success = WATCH_FAIL;
		}
		else
			config_success = VERIFY_FAIL;
	}
	else
		config_success = BUS_FAIL;

	return config_success;
}

extern void writeConfig(uint16_t config_val)
{
	uint8_t i = 0;
	ts_pinMode(E_PIN, OUTPUT);//
	ts_pinMode(D_PIN, OUTPUT);//
	ts_digitalWrite(E_PIN, HIGH);
	ts_digitalWrite(D_PIN, HIGH);
	//ts_pinMode(E_PIN, OUTPUT);
	//ts_pinMode(D_PIN, OUTPUT);
	ts_delayUs(BUS_DRV_DLY);
	ts_digitalWrite(D_PIN, LOW);
	ts_delayUs(BUS_DRV_DLY);
	ts_digitalWrite(E_PIN, LOW);
	ts_delayUs(BUS_DRV_DLY);
	for (i = 0; i < 15; i++)
	{
		config_val = config_val << 1;
		if ((config_val & 0x8000) > 0) ts_digitalWrite(D_PIN, HIGH);
		else ts_digitalWrite(D_PIN, LOW);
		ts_delayUs(BUS_DRV_DLY);
		ts_digitalWrite(E_PIN, HIGH);
		ts_delayUs(BUS_DRV_DLY);
		ts_digitalWrite(E_PIN, LOW);
		ts_delayUs(BUS_DRV_DLY);
	}
	ts_digitalWrite(D_PIN, LOW);
	ts_delayUs(BUS_DRV_DLY);
	ts_digitalWrite(E_PIN, HIGH);
	ts_delayUs(BUS_DRV_DLY);
	ts_digitalWrite(D_PIN, HIGH);
	ts_delayUs(BUS_DRV_DLY);
	//ts_pinMode(E_PIN, INPUT);
	//ts_pinMode(D_PIN, INPUT);
}

extern uint16_t readConfig(void)
{
	uint16_t readback;
	uint8_t i = 0;

	readback = 0x0000;
	ts_pinMode(E_PIN, OUTPUT);//
	ts_pinMode(D_PIN, OUTPUT);//
	ts_digitalWrite(E_PIN, HIGH);
	ts_digitalWrite(D_PIN, HIGH);
	//ts_pinMode(E_PIN, OUTPUT);
	//ts_pinMode(D_PIN, OUTPUT);
	ts_delayUs(BUS_DRV_DLY);
	ts_digitalWrite(D_PIN, LOW);
	ts_delayUs(BUS_DRV_DLY);
	ts_digitalWrite(E_PIN, LOW);
	ts_delayUs(BUS_DRV_DLY);
	ts_digitalWrite(D_PIN, HIGH);
	ts_delayUs(BUS_DRV_DLY);
	ts_digitalWrite(E_PIN, HIGH);
	ts_delayUs(BUS_DRV_DLY);
	ts_pinMode(D_PIN, INPUT);
	//P1 |= P1_B2__HIGH;
	PORTS_1_enter_DefaultMode_from_RESET();
	ts_pinMode(E_PIN, OUTPUT);//
	ts_delayUs(BUS_DRV_DLY);
	ts_digitalWrite(E_PIN, LOW);
	ts_delayUs(BUS_DRV_DLY);
	for (i = 0; i < 14; i++)
	{
		ts_digitalWrite(E_PIN, HIGH);
		ts_delayUs(BUS_DRV_DLY);
		readback = (readback << 1) | (ts_digitalRead(D_PIN) & 0x0001);
		ts_digitalWrite(E_PIN, LOW);
		ts_delayUs(BUS_DRV_DLY);
	}
	ts_pinMode(D_PIN, OUTPUT);//
	ts_digitalWrite(D_PIN, LOW);
	//ts_pinMode(D_PIN, OUTPUT);
	ts_delayUs(BUS_DRV_DLY);
	ts_digitalWrite(E_PIN, HIGH);
	ts_delayUs(BUS_DRV_DLY);
	ts_digitalWrite(D_PIN, HIGH);
	ts_delayUs(BUS_DRV_DLY);
	//ts_pinMode(E_PIN, INPUT);
	//ts_pinMode(D_PIN, INPUT);
	return readback;
}

//checkBus() performs a voting function where the bus is sampled 3 times
//to find 2 identical results.  This is necessary since light detection is
//asynchronous and can indicate a false state.
extern uint8_t checkBus(void)
{
	uint8_t state;
	uint8_t E_state;
	uint8_t D_state;
	uint8_t S0_count = 0;
	uint8_t SLEEP_count = 0;
	uint8_t WATCH_count = 0;
	uint8_t S3_count = 0;
	uint8_t i=0;

	ts_pinMode(E_PIN, INPUT);//
	ts_pinMode(D_PIN, INPUT);//

	for (i=0; i<3; i++)
	{
		E_state = ts_digitalRead(E_PIN);
		D_state = ts_digitalRead(D_PIN);
		if (D_state == HIGH)
		{
			if (E_state == HIGH) S3_count++;
			else SLEEP_count++;
		}
		else
		{
			if (E_state == HIGH) WATCH_count++;
			else S0_count++;
		}
		ts_delayUs(BUS_CHECK_DLY);
	}
	if (SLEEP_count >= 2) state = SLEEP_STATE;
	else if (WATCH_count >= 2) state = WATCH_STATE;
	else if (S3_count >= 2) state = S3_STATE;
	else if (S0_count >= 2) state = S0_STATE;
	else state = UNKNOWN_STATE;

	return state;
}

extern bool goToWatch(TS4231* device)
{
	bool watch_success;

	if (device->configured == false) watch_success = false;
	else
	{
		switch (checkBus())
		{
			case S0_STATE:
			watch_success = false;
			break;
			case SLEEP_STATE:
			ts_digitalWrite(D_PIN, HIGH);
			ts_pinMode(D_PIN, OUTPUT);
			ts_digitalWrite(E_PIN, LOW);
			ts_pinMode(E_PIN, OUTPUT);
			ts_digitalWrite(D_PIN, LOW);
			ts_pinMode(D_PIN, INPUT);
			ts_digitalWrite(E_PIN, HIGH);
			ts_pinMode(E_PIN, INPUT);
			ts_delayUs(SLEEP_RECOVERY);
			if (checkBus() == WATCH_STATE) watch_success = true;
			else watch_success = false;
			break;
			case WATCH_STATE:
			watch_success = true;
			break;
			case S3_STATE:
			ts_digitalWrite(E_PIN, HIGH);
			ts_pinMode(E_PIN, OUTPUT);
			ts_digitalWrite(D_PIN, HIGH);
			ts_pinMode(D_PIN, OUTPUT);
			ts_digitalWrite(E_PIN, LOW);
			ts_digitalWrite(D_PIN, LOW);
			//ts_pinMode(D_PIN, INPUT);
			ts_digitalWrite(E_PIN, HIGH);
			//ts_pinMode(E_PIN, INPUT);

			PORTS_1_enter_DefaultMode_from_RESET();//

			ts_delayUs(SLEEP_RECOVERY);
			if (checkBus() == WATCH_STATE) watch_success = true;
			else watch_success = false;
			break;
			default:
			watch_success = false;
			break;
		}
	}
	return watch_success;
}
