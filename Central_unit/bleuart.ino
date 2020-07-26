/* File    : bleuart.ino
*  Author  : Mathieu Schnegg
*  Date    : 02.05.2020
*
* Description : Firmware of the central unit
*
* This file is based on the bleuart.ino example
*
* Modifications : Date / Author / Purpose
*
* Platform : nRF52840
*/
#include <Adafruit_BNO055.h>
#include <Adafruit_LittleFS.h>
#include <Adafruit_Sensor.h>
#include <Arduino.h>
#include <bluefruit.h>
#include <InternalFileSystem.h>
#include <utility/imumaths.h>
#include <Wire.h>

/* Debug modes : comment unwanted ones */
#define IMU
#define SENSORS
#define SERIAL
//#undef IMU
//#undef SENSORS
//#undef SERIAL

#define LOOP_SAMPLERATE   (20)            // Delay between every loop in ms   
#define BNO055_SAMPLERATE (5)             // Delay between fresh IMU samples (LOOP_SAMPLERATE periode)
#define BAT_SAMPLERATE    (1500)          // Delay between fresh battery reading (LOOP_SAMPLERATE periode)
#define BUTTON_SAMPLERATE (50)            // Delay between two button readings
#define VBAT_MV_PER_LSB   (0.73242188F)   // 3.0V ADC range and 12-bit ADC resolution = 3000mV/4096
#define VBAT_DIVIDER      (0.5F)          // 100K + 100K voltage divider on VBAT
#define VBAT_DIVIDER_COMP (2.0F)          // Compensation factor for the VBAT divider
#define REAL_VBAT_MV_PER_LSB (VBAT_DIVIDER_COMP * VBAT_MV_PER_LSB)
#define PWR_LOCK          (5)             // D5 Output to lock the step-down
#define BUTTON            (A0)            // A0 Input (analog) to read user button state
#define NBOOT_LOAD        (9)             // D9 BNO055 bootloader mode select
#define BNO055_INT        (10)            // D10 BNO055 interrupt input pin
#define SLAVE_NB          (19)            // Number of sensors
#define ADDR_OFFSET       (0x30)          // Offset for the I2C slave addresses
#define SEND_BUFFER_SIZE  (30)            // Size of output (BLE) buffer
#define BLUE_LED          (19)            // Blue LED pin (D19)

// BLE Service
BLEDfu  bledfu;   // OTA DFU service
BLEDis  bledis;   // Device information
BLEUart bleuart;  // Uart over ble
BLEBas  blebas;   // Battery

uint8_t sendBuffer[SEND_BUFFER_SIZE]={10}; // Buffer for sending data

// I2C
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);  // IMU BNO055
int slaveNb = 0;          // Number of slave (I2C) found
char buffer[6]={0};       // Stock the incoming bytes from I2C 

// Battery pin
uint32_t vbat_pin = PIN_VBAT;

// Sensors
uint8_t sensorValue[SLAVE_NB][6] = {0};

// Functions prototypes
void startAdv(void);
void connect_callback(uint16_t conn_handle);
void disconnect_callback(uint16_t conn_handle, uint8_t reason);
void displaySensorDetails(void);
void displaySensorStatus(void);
void displayCalStatus(void);
float readVBAT(void);
uint8_t mvToPercent(float mvolts);

/*********************************************************************
 * Setup of the program, ran once
 * Initialize :
 * - BLE
 * - I2C and the available sensors
 * - Serial port for debug purpose
 ********************************************************************/
void setup()
{
  // Input/Output
  pinMode(PWR_LOCK, OUTPUT);
  pinMode(NBOOT_LOAD, OUTPUT);
  pinMode(BNO055_INT, INPUT);

  // Output default state
  digitalWrite(PWR_LOCK, HIGH);   // Lock the power supply
  digitalWrite(NBOOT_LOAD, HIGH);

  // End marker
  sendBuffer[SEND_BUFFER_SIZE-1] = 0xFF;  

  // Serial for debug purpose
  Serial.begin(115200); 

  #ifdef SENSORS
    // I2C
    Wire.begin();        // join i2c bus (address optional for master)

    // I2C scan for at least one sensor
    // (address from 0 to 18)
    while(slaveNb<SLAVE_NB)
    {
      slaveNb = 0;
      for (int addr = 0; addr < SLAVE_NB; addr++)
      {
        Wire.beginTransmission(addr+ADDR_OFFSET);
        if (Wire.endTransmission()==0)
        {
          slaveNb++;
          #ifdef SERIAL
            Serial.println(addr+ADDR_OFFSET);
          #endif
        }
      }
      delay(200); // Delay between each scan
      #ifdef SERIAL
        Serial.println("...");
      #endif
    }
  #endif

  #ifdef IMU
    // I2C scan for BNO055 IMU
    while(!bno.begin())
    bno.setExtCrystalUse(true);
  #endif

  // Get a single ADC sample and throw it away
  readVBAT();

  // Read battery level
  sendBuffer[0]=mvToPercent(readVBAT());  

  // Setup the BLE LED to be enabled on CONNECT
  // Note: This is actually the default behaviour, but provided
  // here in case you want to control this LED manually via PIN 19
  Bluefruit.autoConnLed(true);

  // Config the peripheral connection with maximum bandwidth 
  // more SRAM required by SoftDevice
  // Note: All config***() function must be called before begin()
  Bluefruit.configPrphBandwidth(BANDWIDTH_MAX);

  Bluefruit.begin();
  Bluefruit.setTxPower(4);    // Check bluefruit.h for supported values
  Bluefruit.setName("Traker 6DoF");
  //Bluefruit.setName(getMcuUniqueID()); // useful testing with multiple central connections
  Bluefruit.Periph.setConnectCallback(connect_callback);
  Bluefruit.Periph.setDisconnectCallback(disconnect_callback);

  // To be consistent OTA DFU should be added first if it exists
  bledfu.begin();

  // Configure and Start Device Information Service
  bledis.setManufacturer("Nordic");
  bledis.setModel("nRF52840");
  bledis.begin();

  // Configure and Start BLE Uart Service
  bleuart.begin();

  // Start BLE Battery Service
  blebas.begin();

  // Set up and start advertising
  startAdv();
}

/*********************************************************************
 * Main loop
 ********************************************************************/
void loop()
{
  // Local variables
  unsigned long currentTime = 0;
  static uint8_t bnoRefreshCount = 0;
  static uint16_t batRefreshCount = 0;
  int i,j = 0;
  int newData = 0;

  // Execute every 20ms
  currentTime = millis();
  bnoRefreshCount++;
  batRefreshCount++;

  #ifdef SENSORS
    // Request each connected slave some data
    for(i=0; i<SLAVE_NB; i++)
    { 
      // Reset buffer
      for(j=0; j<6; j++)
        buffer[j]=0;
      j = 0;

      Wire.requestFrom((i+ADDR_OFFSET), 6);  // Request 6 bytes from slave device
      while(Wire.available())             // Slave may send less than requested
      { 
        buffer[j] = Wire.read(); // Read receive byte

        #ifdef SERIAL
          // Display received data
          Serial.print(buffer[j],HEX);
          Serial.print(" ");
        #endif

        // Number of received byte management
        j++;
        if(j==6)
        {
          #ifdef SERIAL
            Serial.println("");
          #endif
          break;
        }
      }

      if(j==6)  // Right amount of data received
      {
        // Different data received ?
        if((buffer[0]!=sensorValue[i][0])|(buffer[1]!=sensorValue[i][1])|
          (buffer[2]!=sensorValue[i][2])|(buffer[3]!=sensorValue[i][3])|
          (buffer[4]!=sensorValue[i][4])|(buffer[5]!=sensorValue[i][5]))
        {
          if(newData<3)
          {
            // Update and load sensor value
            sendBuffer[(newData*7)+8]=i;
            sendBuffer[(newData*7)+9]=sensorValue[i][0]=buffer[0];
            sendBuffer[(newData*7)+10]=sensorValue[i][1]=buffer[1];
            sendBuffer[(newData*7)+11]=sensorValue[i][2]=buffer[2];
            sendBuffer[(newData*7)+12]=sensorValue[i][3]=buffer[3];
            sendBuffer[(newData*7)+13]=sensorValue[i][4]=buffer[4];
            sendBuffer[(newData*7)+14]=sensorValue[i][5]=buffer[5];

            newData++;
          }
          else
          {
            // Update and load sensor value
            sensorValue[i][0]=buffer[0];
            sensorValue[i][1]=buffer[1];
            sensorValue[i][2]=buffer[2];
            sensorValue[i][3]=buffer[3];
            sensorValue[i][4]=buffer[4];
            sensorValue[i][5]=buffer[5];
          }
        }
      }
    }
  #endif
  
  
  #ifdef IMU
    // Read IMU samples every 100ms
    if(bnoRefreshCount>=BNO055_SAMPLERATE)
    {
      bnoRefreshCount=0;            // Reset counter
      sendBuffer[1]=bno.getTemp();  // Load temp in the sending buffer

      // Get a new sensor event
      sensors_event_t event;
      bno.getEvent(&event);

      // Load data in the sending buffer
      sendBuffer[2]=(uint16_t(event.orientation.x*100))&0x00FF;
      //Serial.print(sendBuffer[2]);
      //Serial.print(' ');
      sendBuffer[3]=((uint16_t(event.orientation.x*100))&0xFF00)>>8;
      //Serial.println(sendBuffer[3]);
      sendBuffer[4]=(uint16_t((event.orientation.y+180)*100))&0x00FF;
      //Serial.print(sendBuffer[4]);
      //Serial.print(' ');
      sendBuffer[5]=((uint16_t((event.orientation.y+180)*100))&0xFF00)>>8;
      //Serial.println(sendBuffer[5]);
      sendBuffer[6]=(uint16_t((event.orientation.z+180)*100))&0x00FF;
      //Serial.print(sendBuffer[6]);
      //Serial.print(' ');
      sendBuffer[7]=((uint16_t((event.orientation.z+180)*100))&0xFF00)>>8;
      //Serial.println(sendBuffer[7]);

      #ifdef SERIAL
        // Display the floating point data
        Serial.print("X: ");
        Serial.print(event.orientation.x, 4);
        Serial.print("\tY: ");
        Serial.print(event.orientation.y, 4);
        Serial.print("\tZ: ");
        Serial.println(event.orientation.z, 4);
      #endif
    }
  #endif

  // Read battery level every 60s
  if(batRefreshCount>=BAT_SAMPLERATE)
  {
    batRefreshCount = 0;                    // Reset counter
    sendBuffer[0]=mvToPercent(readVBAT());  // Read battery level
  }

  // Send data over BLE
  bleuart.write(sendBuffer,SEND_BUFFER_SIZE);
  
  // Wait before next loop
  while((currentTime+LOOP_SAMPLERATE)>millis());
}

/*********************************************************************
 * startAdv
 * Start the BLE advertising
 ********************************************************************/
void startAdv(void)
{
  // Advertising packet
  Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  Bluefruit.ScanResponse.addName();
  
  /* Start Advertising
   * - Enable auto advertising if disconnected
   * - Interval:  fast mode = 20 ms, slow mode = 152.5 ms
   * - Timeout for fast mode is 30 seconds
   * - Start(timeout) with timeout = 0 will advertise forever (until connected)
   * 
   * For recommended advertising interval
   * https://developer.apple.com/library/content/qa/qa1931/_index.html   
   */
  Bluefruit.Advertising.restartOnDisconnect(true);
  Bluefruit.Advertising.setInterval(32, 244);    // in unit of 0.625 ms
  Bluefruit.Advertising.setFastTimeout(30);      // number of seconds in fast mode
  Bluefruit.Advertising.start(0);                // 0 = Don't stop advertising after n seconds  
}

/*********************************************************************
 * connect_callback
 * Callback invoked when central connects
 * @param conn_handle connection where this event happens
 ********************************************************************/
void connect_callback(uint16_t conn_handle)
{
  // Get the reference to current connection
  BLEConnection* connection = Bluefruit.Connection(conn_handle);

  char central_name[32] = { 0 };
  connection->getPeerName(central_name, sizeof(central_name));

  Serial.print("Connected to ");
  Serial.println(central_name);
}

/*********************************************************************
 * disconnect_callback
 * Callback invoked when a connection is dropped
 * @param conn_handle connection where this event happens
 * @param reason is a BLE_HCI_STATUS_CODE found in ble_hci.h
 ********************************************************************/
void disconnect_callback(uint16_t conn_handle, uint8_t reason)
{
  (void) conn_handle;
  (void) reason;

  Serial.println();
  Serial.print("Disconnected, reason = 0x"); Serial.println(reason, HEX);

  digitalWrite(PWR_LOCK, LOW);  // Power off
}

/*********************************************************************
 * displaySensorDetails
 * Displays some basic information on this sensor from the unified
 * sensor API sensor_t type (see Adafruit_Sensor for more information)
 ********************************************************************/
void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/*********************************************************************
 * displaySensorStatus
 * Display some basic info about the sensor status
 ********************************************************************/
void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

/*********************************************************************
 * displayCalStatus
 * Display sensor calibration status
 ********************************************************************/
void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

/*********************************************************************
 * readVBAT
 * Return the battery voltage level in mV
 ********************************************************************/
float readVBAT(void) 
{
  float raw;

  // Set the analog reference to 3.0V (default = 3.6V)
  analogReference(AR_INTERNAL_3_0);

  // Set the resolution to 12-bit (0..4095)
  analogReadResolution(12); // Can be 8, 10, 12 or 14

  // Let the ADC settle
  delay(1);

  // Get the raw 12-bit, 0..3000mV ADC value
  raw = analogRead(vbat_pin);

  // Set the ADC back to the default settings
  analogReference(AR_DEFAULT);
  analogReadResolution(10);

  // Convert the raw value to compensated mv, taking the resistor-
  // divider into account (providing the actual LIPO voltage)
  // ADC range is 0..3000mV and resolution is 12-bit (0..4095)
  return raw * REAL_VBAT_MV_PER_LSB;
}

/*********************************************************************
 * mvToPercent
 * Return the percentage battery level from mV level
 ********************************************************************/
uint8_t mvToPercent(float mvolts) 
{
  if(mvolts<3300)
    return 0;

  if(mvolts <3600) {
    mvolts -= 3300;
    return mvolts/30;
  }

  mvolts -= 3600;
  return 10 + (mvolts * 0.15F );  // thats mvolts /6.66666666
}