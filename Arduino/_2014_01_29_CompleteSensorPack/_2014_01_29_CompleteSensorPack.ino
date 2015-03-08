/*
MethaneMonitoring sketch
Author  : Michael van den Bossche
Version : 2.0
Date    : 2014-02-03

Reads data from SHT15 rH&T sensor, TGS2600 gas sensor, and BMP180 pressure sensor.
Reports battery status.
Saves data to SD, with timestamp. Goes to sleep between meas. cycles.

Based on the following example sketches: 
  'now' and 'adjust' (DS3231),
  'StalkerV21_DataLogger_5min',
  'SFE_BMP180_Example',
  battery 'example' and 'tropo'.

Last updated 2014.01.31
*/

//include libraries
#include <Sensirion.h>   // for T & RH sensors
#include <Fat16.h>       // for SD disk
#include <Fat16util.h>   // for SD disk
#include <Wire.h>        // for RTC timestamp
#include <DS3231.h>      // for RTC timestamp
#include <avr/sleep.h>   // for sleep function
#include <avr/power.h>   // for sleep function
#include <Battery.h>     // for battery status
#include <SFE_BMP180.h>  // for pressure sensor

// define time constants, in millis
// theat = 60000;        // time that the active heater is on (3min)
// int tSHT = 5000;      // time between SHT measurements (5 seconds)
// int tSHTMeas = 400;   // time it takes to do one SHT measurement (400ms)
int tcycle = 600;        // time for one meas cycle (in s, 10 minutes). tcycle > theat + tmeas


// filename
char name[] = "TESTMB77.CSV";

// define digital pins for SHT sensor
const uint8_t dataPinSHT15  =  7;
const uint8_t clockPinSHT15 =  6;
Sensirion SHT15 = Sensirion(dataPinSHT15, clockPinSHT15);

// define variables for SHT sensor
float tempSHT1;
float humSHT1;
float dewSHT1;
float tempSHT2;
float humSHT2;
float dewSHT2;
float tempSHT3;
float humSHT3;
float dewSHT3;

// From 'tropo' sketch from book 'atmospheric modeling with Arduino'
// define pins and variable for the gas sensor
const int gasPin1 = A3; // analog pin that reads the gas sensor
const int heaterPin1 = 8; // digital pin that controls the active heater
int gasVal1 = 0; // value read from the sensor at #A3

// create instance of BMP180 pressure sensor, set altitude:
SFE_BMP180 pressure;
#define ALTITUDE 191.0  // elevation in m for Cville, VA

// create instances of the SD card, battery and pressure sensor.
SdCard card;         // For SD card
Fat16 file;          // For SD card
Battery battery;     // LiPO battery

DS3231 RTC;
//static uint8_t prevSecond=0; 
static DateTime interruptTime;
DateTime dt(__DATE__,__TIME__); // define date, time as compilation date & time

//The following code is taken from sleep.h as Arduino Software v22 (avrgcc) in w32 does not have the latest sleep.h file
//From StalkerV21_DataLogger_5min example. This function further reduces power consumption when in sleep mode.
#define sleep_bod_disable() \
{ \
  uint8_t tempreg; \
  __asm__ __volatile__("in %[tempreg], %[mcucr]" "\n\t" \
                       "ori %[tempreg], %[bods_bodse]" "\n\t" \
                       "out %[mcucr], %[tempreg]" "\n\t" \
                       "andi %[tempreg], %[not_bodse]" "\n\t" \
                       "out %[mcucr], %[tempreg]" \
                       : [tempreg] "=&d" (tempreg) \
                       : [mcucr] "I" _SFR_IO_ADDR(MCUCR), \
                         [bods_bodse] "i" (_BV(BODS) | _BV(BODSE)), \
                         [not_bodse] "i" (~_BV(BODSE))); \
}

// store error strings in flash to save RAM
// from Stalkerv21_DataLogger_5min example
#define error(s) error_P(PSTR(s))

void error_P(const char* str) {
  PgmPrint("error: ");
  SerialPrintln_P(str);
  if (card.errorCode) {
    PgmPrint("SD error: ");
    Serial.println(card.errorCode, HEX);
  }
  while(1);
}

void setup()
{
  /*Initialize INT0 pin for accepting interrupts */
  //From StalkerV21_DataLogger_5min example
  PORTD |= 0x04; 
  DDRD &=~ 0x04;
  pinMode(4,INPUT);    // Use pin 4 to wake up the processor.
  
  Serial.begin(9600);  // Start communication on COM3
  Wire.begin();
  RTC.begin();
  RTC.adjust(dt);      // Adjust date-time as defined 'dt' above
  
  //From StalkerV21_DataLogger_5min example
  attachInterrupt(0, INT0_ISR, LOW);   // Only LOW level interrupt can wake up from PWR_DOWN
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Selects Power Down as sleep mode.
 
  //Enable Interrupt 
  DateTime  start = RTC.now();
  interruptTime = DateTime(start.get() + tcycle); //Adds SleepInt (in sec) to start time

  // set digital pin 5 as output to the gas sensor's heater.
  pinMode(heaterPin1, OUTPUT);

  // from SFE_BMP180_example
  // Initialize the pressure sensor
  if (pressure.begin())
  {
    Serial.println("BMP180 init success");
    Serial.print("provided altitude: ");
    Serial.print(ALTITUDE,0);
    Serial.println(" meters, ");
    Serial.println();
  }  
  else
  {
    Serial.println("BMP180 init fail\n\n");
    while(1); // Pause forever.
  }
  
  // from Stalkerv21_DataLogger_5min example
  // initialize the SD card
  if (!card.init()) error("card.init");
   
  // initialize a FAT16 volume
  if (!Fat16::init(&card)) error("Fat16::init");
  
  // clear write error
  file.writeError = false;
  
  // O_CREAT - create the file if it does not exist
  // O_APPEND - seek to the end of the file prior to each write
  // O_WRITE - open for write
  if (!file.open(name, O_CREAT | O_APPEND | O_WRITE))
      error("error opening file");
  
  // logging header
  file.println("date,time,julian,batt voltage [V],charge [%],status,temp1 [C],rH1 [%],temp2 [C],rH2 [%],temp3 [C],rH3 [%],Vout CH4 [mV],pAbs [mb],pSea [mb]");

    if (!file.close()) 
      error("error closing file");
  }

void loop()
{
  digitalWrite(heaterPin1, HIGH);      // turn gas sensor heater on
  Serial.println("Heater On");
  Serial.println();
  delay(175000);
    
  //SHT sensor readout 1
  SHT15.measure(&tempSHT1, &humSHT1, &dewSHT1);
     
  Serial.print("SHT15  Temp: ");
  Serial.print(tempSHT1);
  Serial.print(" C, RH: ");
  Serial.print(humSHT1);
  Serial.println(" %");
  Serial.println();
  
  delay(4600);
  
  DateTime now = RTC.now(); //get the current date-time
  
  Serial.print(now.year(), DEC);
  Serial.print('-');
  Serial.print(now.month(), DEC);
  Serial.print('-');
  Serial.print(now.date(), DEC);
  Serial.print(' ');
  Serial.print(now.hour(), DEC);
  Serial.print(':');
  Serial.print(now.minute(), DEC);
  Serial.print(':');
  Serial.print(now.second(), DEC);
  Serial.print(' ');
  Serial.println();
  Serial.print("Seconds since 1/1/2000: "); 
  Serial.print(now.get(),DEC);
  Serial.println();
  
  //SHT sensor readout 2
  SHT15.measure(&tempSHT2, &humSHT2, &dewSHT2);
     
  Serial.print("SHT15  Temp: ");
  Serial.print(tempSHT2);
  Serial.print(" C, RH: ");
  Serial.print(humSHT2);
  Serial.println(" %");
  Serial.println();
  
  // gas sensor measurement
  gasVal1 = analogRead(gasPin1);       // read the analog in value
  digitalWrite(heaterPin1, LOW);       // shut off the heater  
  
  Serial.print("gas sensor reading: ");
  Serial.println(gasVal1);
  Serial.println();
  
  delay(4600);
  
  //SHT sensor readout
  SHT15.measure(&tempSHT3, &humSHT3, &dewSHT3);
     
  Serial.print("SHT15  Temp: ");
  Serial.print(tempSHT3);
  Serial.print(" C, RH: ");
  Serial.print(humSHT3);
  Serial.println(" %");
  Serial.println();
  delay(10);
  
  //from SFE_BMP180_example
  char status;
  double T,P,p0,a;

  status = pressure.startTemperature();  // start temperature measurement
  if (status != 0)                       // if successful, nr of ms to wait is returned as 'status'
  {
    delay(status);                       // Wait for the measurement to complete:    

    status = pressure.getTemperature(T); // retrieve the temperature
    if (status != 0)                     // Function returns 1 if successful, 0 if failure
    {
      // Print out the measurement:
      Serial.print("temperature: ");
      Serial.print(T,2);
      Serial.print(" deg C, ");
      Serial.println();
            
      status = pressure.startPressure(3);  // Start a pressure measurement
      // The parameter is the oversampling setting, from 0 to 3 (highest res, longest wait).
      if (status != 0)                     // If request is successful, the number of ms to wait is returned.
      {
        delay(status);                     // Wait for the measurement to complete

        status = pressure.getPressure(P,T); // Retrieve the completed pressure measurement
        if (status != 0)                    // Function returns 1 if successful, 0 if failure.
        {
          // Print out the measurement:
          p0 = pressure.sealevel(P,ALTITUDE); // p0 = sea-level compensated pressure in mb, using altitude in m.
          Serial.print("relative (sea-level) pressure: ");
          Serial.print(p0,2);
          Serial.println(" mb, ");
          Serial.println();
        }
        else Serial.println("error retrieving pressure measurement\n");
      }
      else Serial.println("error starting pressure measurement\n");
    }
    else Serial.println("error retrieving temperature measurement\n");
  }
  else Serial.println("error starting temperature measurement\n");

  // From battery example sketch
  battery.update();
  float voltage = battery.getVoltage();
  int percentage = battery.getPercentage();
  char* CS = battery.getChStatus();
      
  Serial.print("battery: ");
  Serial.print(voltage);
  Serial.print("V  -> ");
  Serial.print(percentage);
  Serial.print("%     Charge Status: ");
  Serial.println(CS);
  Serial.println();
  delay(10);
  
  // from Stalkerv21_DataLogger_5min example
  // initialize the SD card
  if (!card.init()) error("card.init");
   
  // initialize a FAT16 volume
  if (!Fat16::init(&card)) error("Fat16::init");
  
  // clear write error
  file.writeError = false;
  
  // O_CREAT - create the file if it does not exist
  // O_APPEND - seek to the end of the file prior to each write
  // O_WRITE - open for write
  if (!file.open(name, O_CREAT | O_APPEND | O_WRITE))
      error("error opening file");
  
  
  // logging timestamp
  // from 'now' example sketch
  file.print(now.year(), DEC);
  file.print('-');
  file.print(now.month(), DEC);
  file.print('-');
  file.print(now.date(), DEC);
  file.print(',');
  file.print(now.hour(), DEC);
  file.print(':');
  file.print(now.minute(), DEC);
  file.print(':');
  file.print(now.second(), DEC);
  file.print(',');
  file.print(now.get(),DEC);
  file.print(',');
  
  // logging battery status
  file.print(voltage);
  file.print(',');
  file.print(percentage);
  file.print(',');
  file.print(CS);
  file.print(',');
  
  // logging SHT sensor info
  file.print(tempSHT1);
  file.print(",");
  file.print(humSHT1);
  file.print(",");
  file.print(tempSHT2);
  file.print(",");
  file.print(humSHT2);
  file.print(",");
  file.print(tempSHT3);
  file.print(",");
  file.print(humSHT3);
  file.print(",");

  // logging gas sensor info
  file.print(gasVal1);
  file.print(",");

  //logging pressure info
  file.print(P,2); //absolute pressure
  file.print(",");
  file.print(p0,2); //sealevel pressure
  file.println();
         
  if (!file.close()) 
      error("error closing file");
      
  //Everything below from StalkerV21_DataLogger_5min example
  RTC.clearINTStatus(); //This function call is  a must to bring /INT pin HIGH after an interrupt.
  RTC.enableInterrupts(interruptTime.hour(),interruptTime.minute(),interruptTime.second());    // set the interrupt at (h,m,s)
  attachInterrupt(0, INT0_ISR, LOW);  //Enable INT0 interrupt (as ISR disables interrupt). This strategy is required to handle LEVEL triggered interrupt
   
  //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Down routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\
            
  //Power Down routines
  cli(); 
  sleep_enable();      // Set sleep enable bit
  sleep_bod_disable(); // Disable brown out detection during sleep. Saves more power
  sei();
       
  Serial.print("\nSleeping");
  Serial.println();
  delay(20); //This delay is required to allow print to complete
  //Shut down all peripherals like ADC before sleep. Refer Atmega328 manual
  power_all_disable(); //This shuts down ADC, TWI, SPI, Timers and USART
  sleep_cpu();         // Sleep the CPU as per the mode set earlier(power down)  
  sleep_disable();     // Wakes up sleep and clears enable bit. Before this ISR would have executed
  power_all_enable();  //This shuts enables ADC, TWI, SPI, Timers and USART
  delay(10); //This delay is required to allow CPU to stabilize
  Serial.print("Awake from sleep");
  Serial.println();  
    
  //\/\/\/\/\/\/\/\/\/\/\/\/Sleep Mode and Power Saver routines\/\/\/\/\/\/\/\/\/\/\/\/\/\/\/\

} 
//Interrupt service routine for external interrupt on INT0 pin conntected to DS3231 /INT
void INT0_ISR()
{
 //Keep this as short as possible. Possibly avoid using function calls
   detachInterrupt(0); 
   interruptTime = DateTime(interruptTime.get() + tcycle);  //decide the time for next interrupt, configure next interrupt  
}