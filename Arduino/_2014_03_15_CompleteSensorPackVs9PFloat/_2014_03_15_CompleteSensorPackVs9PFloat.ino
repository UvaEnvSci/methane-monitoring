/*
 * Reads data from SHT15 rH&T sensor, TGS2600 gas sensor, and BMP180 pressure sensor.
 * Reports battery status.
 * Saves data to SD, with timestamp. Goes to sleep between meas. cycles.
 * 
 * Based on the following example sketches: 'now' and 'adjust' (DS3231), 'StalkerV21_DataLogger_5min', 'SFE_BMP180_Example', battery 'example' and 'tropo'.
 * by Michael van den Bossche.
 * Last updated 2014.03.15
 * Used Bosch Pressure Sensor BMP085 / BMP180 readout routine by Leo Nutz        
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
#include <MemoryFree.h>  // check RAM

//declare global variables
int ID                    =  1;              // ID of the sensor. This is sensor assembly 1.
int tcycle                = 600;             // time for one meas cycle (in s, 10 minutes).
int i                     = 5;               // nr of cycles
int j                     = 0;               // counter for cycles
//define variables for SHT sensor
float measT, measrH, measTd;                 // used to store measured values
float AvT, AvrH;                             // average T and rH for XBee
int   AvTInt, AvrHInt;                       // cast floats into ints before sending
//define variable for gas sensor reading
int   gasVal = 0;                            // value read from the sensor at #A3
//define variables for battery readings
float voltage;
int   voltageInt;                            // cast float into int before sending
char* CS;
//define variables for atmospheric pressure sensor
#define ADDRESS_SENSOR 0x77                  // Sensor address
int16_t  ac1, ac2, ac3, b1, b2, mb, mc, md; // Store sensor PROM values from BMP180
uint16_t ac4, ac5, ac6;                     // Store sensor PROM values from BMP180
const uint8_t oss = 3;                      // Set oversampling setting to high resolution
const uint8_t osd = 26;                     // with corresponding oversampling delay 
float Tpres,P;
long  PLong;                                // cast float into long
int   PInt,PInt2;                           // cast long into 2 integers before sending
//define strings for XBee URL messages
char message [88];
//define filename
char name [13] = "TESTMB99.CSV";

// define Seeeduino pins
const int WakeMP            =  4;            // microprocessor interrupt
const uint8_t clockPinSHT15 =  6;            // SHT sensor
const uint8_t dataPinSHT15  =  7;            // SHT sensor
const int AREFControl       =  8;            // AREF control
// const int heaterPin1     =  8;            // gas sensor heater power - always on.
const int XBee              =  9;            // XBee control
const int gasPin1           = A3;            // analog pin that reads the gas sensor

// create instance of SHT15, SD card, battery and RTC.
Sensirion SHT15 = Sensirion(dataPinSHT15, clockPinSHT15);
SdCard card;         
Fat16 file;         
Battery battery;
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
  pinMode(WakeMP,INPUT);    
  
  //Initialize XBee
  pinMode(XBee,INPUT);    
  digitalWrite(XBee,HIGH);      // use as pull-up resistor to set to sleep
  Serial.begin(38400);          // Baud rate for XBee is 38400
  
  //Initialize RTC time
  Wire.begin();
  RTC.begin();
  RTC.adjust(dt);      // Adjust date-time as defined 'dt' above
  
  //From StalkerV21_DataLogger_5min example
  attachInterrupt(0, INT0_ISR, LOW);   // Only LOW level interrupt can wake up from PWR_DOWN
  set_sleep_mode(SLEEP_MODE_PWR_DOWN); // Selects Power Down as sleep mode.
 
  //Enable Interrupt 
  DateTime  start = RTC.now();
  interruptTime = DateTime(start.get() + tcycle); //Adds SleepInt (in sec) to start time

  //pinMode(heaterPin1, OUTPUT); // set digital pin 8 as output to control gas sensor heater

  // Initialize the pressure sensor
  init_SENSOR();
  delay(100);
  
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
  file.print("date,time,julian,temp1 [C],rH1 [%],temp2 [C],rH2 [%],temp3 [C],rH3 [%],Vout CH4 [dig],batt voltage [V],charge [%],status,pAbs [mb],pSea [mb],");
    
  if (!file.close()) 
      error("error closing file");
}

void loop()
{
  gasVal = analogRead(gasPin1);  // read out gasVal to prevent wrong readings - 'warm-up' analog port.  
  //digitalWrite(heaterPin1, HIGH); // gas sensor heater is on all the time;
  pinMode(AREFControl, OUTPUT);
  digitalWrite(AREFControl, HIGH);
  delay(2000);
  analogReference(EXTERNAL); // use AREF for reference voltage (3.3)
  delay(173000);
  
  // from Stalkerv21_DataLogger_5min example: open file on SD
  if (!card.init()) error("card.init");                // initialize the SD card
  if (!Fat16::init(&card)) error("Fat16::init");       // initialize a FAT16 volume
  file.writeError = false;                             // clear write error
  if (!file.open(name, O_CREAT | O_APPEND | O_WRITE))
      error("error opening file");
  // O_CREAT - create the file if it does not exist
  // O_APPEND - seek to the end of the file prior to each write
  // O_WRITE - open for write
  
  for (j=0;j<i;j++)
  {  
    //SHT sensor readout 1
    DateTime now = RTC.now(); //get the current date-time  
    SHT15.measure(&measT, &measrH, &measTd);
    
    // logging timestamp
    // from 'now' example sketch
    file.println();
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
    // logging SHT sensor info
    file.print(measT);
    file.print(",");
    file.println(measrH);
    AvT=measT;
    AvrH=measrH;
    delay(4600);
  
    //SHT sensor readout 2 & gas sensor measurement
    now = RTC.now();                               //get current date&time  
    SHT15.measure(&measT, &measrH, &measTd);       //get rH&T
    gasVal = analogRead(gasPin1);                  //get gas sensor read-out
    
    // logging timestamp
    // from 'now' example sketch
    file.println();
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
    //logging SHT sensor info
    file.print(measT);
    file.print(",");
    file.println(measrH);
    // logging gas sensor info
    file.print(gasVal);
    file.print(",");
    AvT=AvT+measT;
    AvrH=AvrH+measrH;

    delay(4600);
  
    //SHT sensor readout 3
    now = RTC.now();                               //get the current date-time  
    SHT15.measure(&measT, &measrH, &measTd);
    
    // logging timestamp
    // from 'now' example sketch
    file.println();
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
    // logging SHT sensor info
    file.print(measT);
    file.print(",");
    file.println(measrH);
    AvT=(AvT+measT)/3.0;
    AvrH=(AvrH+measrH)/3.0;
    delay(20400);
     
    //XBee write over ethernet
    AvTInt=(int) (AvT*100);
    AvrHInt=(int) (AvrH*100);
    voltageInt=(int) (voltage*100);
    pinMode(XBee,OUTPUT);
    digitalWrite(XBee,LOW); // wake up Xbee
    delay(50);           // allow connect XBee to network.
    snprintf ( message, 88, "http://methanemonitoring.appspot.com/log?ID=%d&T=%d&rH=%d&CH4=%d&P=%d.%d&Batt=%d", ID, AvTInt, AvrHInt, gasVal, PInt, PInt2, voltageInt);
    Serial.println(message);
    pinMode(XBee,INPUT);
    digitalWrite(XBee,HIGH);
    }

  //digitalWrite(heaterPin1, LOW); //shut off heater to save power - always on now.
  // From battery example sketch
  analogReference(INTERNAL);
  pinMode(AREFControl, INPUT);
  delay(5000);
  battery.update();
  voltage = battery.getVoltage();
  CS = battery.getChStatus();
  delay(2000);
  battery.update();
  voltage = battery.getVoltage();
  CS = battery.getChStatus();
      
  Serial.print("battery: ");
  Serial.print(voltage);
  Serial.print("V  -> Charge Status: ");
  Serial.println(CS);
  Serial.println();
  
  // logging battery status
  file.print(voltage);
  file.print(',');
  file.print(CS);
  file.print(',');
  delay(10);

  //SFE_BMP180
  int32_t b5;
 
  b5 = temperature();                       // Read and calculate temperature (T) 
  P = pressure(b5);                         // Read and calculate pressure (P) 

  //Print out the measurement:
  Serial.print("Pressure: ");
  Serial.print(P,2);
  Serial.println(" mb, ");

  //logging absolute pressure info
  file.print(P,2);
  file.print(",");
  
  if (!file.close()) 
      error("error closing file");
  
  //The following three lines convert the P float into two integers to send with XBee    
  PLong=(long) (100*P);
  PInt2=PLong%100;
  PInt=PLong/100;
 
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

  //BMP180 routines
/**********************************************
  Initialize sensor variables
 **********************************************/
void init_SENSOR()
{
  ac1 = read_2_bytes(0xAA);
  ac2 = read_2_bytes(0xAC);
  ac3 = read_2_bytes(0xAE);
  ac4 = read_2_bytes(0xB0);
  ac5 = read_2_bytes(0xB2);
  ac6 = read_2_bytes(0xB4);
  b1  = read_2_bytes(0xB6);
  b2  = read_2_bytes(0xB8);
  mb  = read_2_bytes(0xBA);
  mc  = read_2_bytes(0xBC);
  md  = read_2_bytes(0xBE);
}

/**********************************************
  Calculate pressure readings
 **********************************************/
float pressure(int32_t b5)
{
  int32_t x1, x2, x3, b3, b6, p, UP;
  uint32_t b4, b7; 

  UP = read_pressure();                         // Read raw pressure

  b6 = b5 - 4000;
  x1 = (b2 * (b6 * b6 >> 12)) >> 11; 
  x2 = ac2 * b6 >> 11;
  x3 = x1 + x2;
  b3 = (((ac1 * 4 + x3) << oss) + 2) >> 2;
  x1 = ac3 * b6 >> 13;
  x2 = (b1 * (b6 * b6 >> 12)) >> 16;
  x3 = ((x1 + x2) + 2) >> 2;
  b4 = (ac4 * (uint32_t)(x3 + 32768)) >> 15;
  b7 = ((uint32_t)UP - b3) * (50000 >> oss);
  if(b7 < 0x80000000) { p = (b7 << 1) / b4; } else { p = (b7 / b4) << 1; } // or p = b7 < 0x80000000 ? (b7 * 2) / b4 : (b7 / b4) * 2;
  x1 = (p >> 8) * (p >> 8);
  x1 = (x1 * 3038) >> 16;
  x2 = (-7357 * p) >> 16;
  return (p + ((x1 + x2 + 3791) >> 4)) / 100.0f; // Return pressure in mbar
}

/**********************************************
  Read uncompensated temperature
 **********************************************/
int32_t temperature()
{
  int32_t x1, x2, b5, UT;

  Wire.beginTransmission(ADDRESS_SENSOR); // Start transmission to device 
  Wire.write(0xf4);                       // Sends register address
  Wire.write(0x2e);                       // Write data
  Wire.endTransmission();                 // End transmission
  delay(5);                               // Datasheet suggests 4.5 ms
  
  UT = read_2_bytes(0xf6);                // Read uncompensated TEMPERATURE value

  // Calculate true temperature
  x1 = (UT - (int32_t)ac6) * (int32_t)ac5 >> 15;
  x2 = ((int32_t)mc << 11) / (x1 + (int32_t)md);
  b5 = x1 + x2;
  Tpres  = (b5 + 8) >> 4;
  Tpres = Tpres / 10.0;                     // Temperature in celsius 
  return b5;  
}

/**********************************************
  Read uncompensated pressure value
 **********************************************/
int32_t read_pressure()
{
  int32_t value; 
  Wire.beginTransmission(ADDRESS_SENSOR);   // Start transmission to device 
  Wire.write(0xf4);                         // Sends register address to read from
  Wire.write(0x34 + (oss << 6));            // Write data
  Wire.endTransmission();                   // SEd transmission
  delay(osd);                               // Oversampling setting delay
  Wire.beginTransmission(ADDRESS_SENSOR);
  Wire.write(0xf6);                         // Register to read
  Wire.endTransmission();
  Wire.requestFrom(ADDRESS_SENSOR, 3);      // Request three bytes
  if(Wire.available() >= 3)
  {
    value = (((int32_t)Wire.read() << 16) | ((int32_t)Wire.read() << 8) | ((int32_t)Wire.read())) >> (8 - oss);
  }
  return value;                             // Return value
}

/**********************************************
  Read 1 byte from the BMP sensor
 **********************************************/
uint8_t read_1_byte(uint8_t code)
{
  uint8_t value;
  Wire.beginTransmission(ADDRESS_SENSOR);         // Start transmission to device 
  Wire.write(code);                               // Sends register address to read from
  Wire.endTransmission();                         // End transmission
  Wire.requestFrom(ADDRESS_SENSOR, 1);            // Request data for 1 byte to be read
  if(Wire.available() >= 1)
  {
    value = Wire.read();                          // Get 1 byte of data
  }
  return value;                                   // Return value
}

/**********************************************
  Read 2 bytes from the BMP sensor
 **********************************************/
uint16_t read_2_bytes(uint8_t code)
{
  uint16_t value;
  Wire.beginTransmission(ADDRESS_SENSOR);         // Start transmission to device 
  Wire.write(code);                               // Sends register address to read from
  Wire.endTransmission();                         // End transmission
  Wire.requestFrom(ADDRESS_SENSOR, 2);            // Request 2 bytes from device
  if(Wire.available() >= 2)
  {
    value = (Wire.read() << 8) | Wire.read();     // Get 2 bytes of data
  }
  return value;                                   // Return value
}  
  
