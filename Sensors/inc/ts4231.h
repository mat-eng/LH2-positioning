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

#ifndef INC_TS4231_H_
#define INC_TS4231_H_

#define BUS_DRV_DLY     1       //delay in microseconds between bus level changes
#define BUS_CHECK_DLY   500     //delay in microseconds for the checkBus() function
#define SLEEP_RECOVERY  100     //delay in microseconds for analog wake-up after exiting SLEEP mode
#define UNKNOWN_STATE   0x04    //checkBus() function state
#define S3_STATE        0x03    //checkBus() function state
#define WATCH_STATE     0x02    //checkBus() function state
#define SLEEP_STATE     0x01    //checkBus() function state
#define S0_STATE        0x00    //checkBus() function state
#define CFG_WORD        0x392B  //configuration value
#define BUS_FAIL        0x01    //configDevice() function status return value
#define VERIFY_FAIL     0x02    //configDevice() function status return value
#define WATCH_FAIL      0x03    //configDevice() function status return value
#define CONFIG_PASS     0x04    //configDevice() function status return value

#define E_PIN			0
#define D_PIN			1
#define INPUT			0
#define OUTPUT			1
#define LOW				0
#define HIGH			1

typedef struct TS4231 TS4231;
struct TS4231
{
	uint8_t configured;
};

extern bool waitForLight(void);  //timeout in milliseconds
extern bool goToSleep(TS4231 device);
extern uint8_t configDevice(TS4231* device);
extern bool goToWatch(TS4231* device);

extern uint8_t checkBus(void);
extern void ts_delayUs(uint32_t delay_val);  //delay in microseconds
extern void ts_pinMode(int pin, uint8_t mode);
extern uint8_t ts_digitalRead(int pin);
extern void ts_digitalWrite(int pin, uint8_t write_val);
extern void writeConfig(uint16_t config_val);
extern uint16_t readConfig(void);

#endif /* INC_TS4231_H_ */
