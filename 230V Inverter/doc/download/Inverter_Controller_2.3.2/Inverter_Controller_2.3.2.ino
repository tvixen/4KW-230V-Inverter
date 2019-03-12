  
////////////////////////////////////////////////////////////////////////////////////////////
// Voltage and Temperature Controller for 230Volt 4Kw Inverter                            //  
// Version: Arduino IDE 1.8.5                                                             //
//                                                                                        //    
// Code written by Tim Milgart                                                            //
//                                                                                        //
// Start Date: 29.10.2018                                                                 //
// This code is supposed to work with a OLED and a 16bit ADC chip called ADS1115          //
// The code will fit into a Arduoni Nano, but the code is compiled for a Uno because      // 
// it will give us some more space, as the bootsection is loaded with Optiboot.           //
//                                                                                        // 
// Update 29/10-2018 f                                                                    //
// Added a watchdog and setup up calls                                                    // 
// Update 29/10-2018                                                                      //  
// Version 1.1 added CheckBattVoltage, ReadADC, AnalogRead                                //
// Update 30/10-2018                                                                      // 
// Version 1.2 added temperature,fanspeed and shutdown                                    //
// Update 1/11-2018                                                                       //
// Added temperature calculation in ReadTemp()                                            //
//                                                                                        //
// Update 15/11-2018                                                                      //
// Added more temperature calculation in ReadTemp()                                       //
// Update 15/11-2018                                                                      //  
// Changed port 8 to 2 and Version to 1.6                                                 // 
// Update 16/11-2018                                                                      //
// Added more all variables from sensors to LCD panel                                     //
// New version 1.7                                                                        //
// Update 21/11-2018                                                                      //
// Added a ADS1115 more on the Relay print. To messure the DC current from the battery    //
// So now theres 2 ADS1115 to address.                                                    //
// Version 1.8                                                                            // 
// Update 24/11-2018                                                                      //
// Added current meassurement for ADS1115                                                 //
// Version 1.9 3:17                                                                       // 
// Update 2/12-2018                                                                       //
// Version 2.0                                                                            // 
// Updated timers for wire                                                                // 
// added test program Simulatedata(), and finished the transmission part to display       //
// added 16bit function (TotalPower) to the I2C communication Simulation is finished      //
// Display layout is finished                                                             //
// New name Inverter Controller 2.0                                                       //
// New name Inverter Controller 2.1                                                       //
// Added TotalInputPower og TotalOutputPower                                              //
// Update 26/12-2018                                                                      //
// New name Inverter Controller 2.2                                                       //
// Added Temperature sensor for Case and new voltage regulator for back fan.              //
// Front fan has build in temp sensor, and will run on 12V all time.                      //
// Back fan have a seperate 0-30volt regulatur controlled by up and down, by the Nano     //
// controller pin A10 & A11.                                                              //
// Update 29/12-2018 f                                                                    //
// Added "F" in WriteDataToLCD() with Read_ToroidOutput230Current, so the AC current is   //
// now transmitted to display. And Battery is now transmitted as a float.                 //
// Update 3/1-2019                                                                        //
// changed communications procedure                                                       // 
// Update 3/1-2019   Code not 100% ready.                                                 //
////////////////////////////////////////////////////////////////////////////////////////////

#include <EEPROM.h>
#include <avr/sleep.h>   // til sleep og interrupt
#include <avr/wdt.h>     // watchdog
#include <Wire.h>        // kan tale med ADS1115 print
#include <Adafruit_ADS1015.h>    // ADC 1115 


//Default i2c SLAVE address/Unconfigurated (used for auto provision of address)
#define DEFAULT_SLAVE_ADDR                    21


typedef struct cell_module2 {int volt; byte id;} records; records celledata[3]; 


//HERE goes all the Slave variables ----------------------
Adafruit_ADS1115 ads1115VoltModule(0x48);     // construct an ads1115 at address 
Adafruit_ADS1115 ads1115CurrentModule(0x49);  // construct an ads1115 at address 
/* ADC BUS addresses:
 * 0x4A = SDA  No 1
 * 0x49 = 5V   No 2
 * 0x48 = GND  No 3
 * 0x54 = SLC  No 4
 */

// LCD_adr on I2C bus
#define LCD_ADDRESS 8            // LCD address
 
float ADCvalue   =         4.201; //value from ADC
uint16_t results =          0;  //mellem resultat fra ADC

float Current    =          0.201; //value from Current ADC



//Where in EEPROM do we store the configuration
#define EEPROM_CHECKSUM_ADDRESS           0
#define EEPROM_CONFIG_ADDRESS            20

// Battery min&max voltage
#define MIN_BATT_VOLTAGE            21200U
#define MAX_BATT_VOLTAGE            29600U
#define MIN_Control_5VOLTAGE         4800U
#define MAX_Control_5VOLTAGE         5500U
#define MIN_Control_12VOLTAGE      115000U
#define MAX_Control_12VOLTAGE      135000U
#define MIN_CapVOLTAGE             220000U
#define MAX_CapVOLTAGE             290000U

// Temperature Min&Max degrees 
#define MAX_HeatsinkTemp              60
#define MAX_ToroidTemp                70
#define MAX_CaseTemp                  50
#define Mid_CaseTemp                  40
#define Min_CaseTemp                  30
#define Low_CaseTemp                  20
#define Fan_Max_100                  255
#define Fan_Mid_75                   192
#define Fan_Low_50                   128
#define Fan_Min_25                    64
#define Fan_Off_0                      0
 
// Timers
#define MIN_TimeOnLowVoltage          5    //How many seconds below MIN_BATT_VOLTAGE till shout down is triggered
#define RestartTime                  10    //How many minutes to restart if battery volts recover above MIN_BATT_VOLTAGE

// Analog Port define 
#define TemperatureForHeatsinkPortA0 A0   // Temperature from Power board
#define TemperatureForToroidPortA1   A1   // Temperature from Toroid
#define TemperatureForCasePortA2     A2   // Temperature from inside the case
#define SineWave12VoltPortA3         A3   // direct to 12 volt on Controller Board
//   in use                          A4   // For communication SDA 
//   in use                          A5   // For communication SCL
#define SineWave5VoltPortA6          A6   // direct to 5 volt on Controller Board

// Digital Port define 
#define StatusLedPin                 13   // Green led
#define FanUp                        10   // Fanblower for Heatsink. Fan1
#define FanDown                      11   // Fanblower for Toroid.   Fan2
#define PowerRelayT1                  4   // Realy T1
#define CapRelayT2                    5   // Realy T2
#define WaveOnSW1                     6   // Realy SW1 
#define DischargeRelaySW2             7   // Realy SW2
#define Beeper                        2   // Error beeper

// for PCB 6 outputs 5 analog inputs, 2 analog outputs, 2 data lines 

//Variables 
uint16_t last_raw_adc              =    0;
uint16_t TemperatureForHeatsink    =    0;
uint16_t TemperatureForToroid      =    0;
uint16_t TemperatureForCase        =    0;
uint8_t PacketNo                   =    0;
uint16_t FanVolt                   =    0;
bool SineWaveOk                    =    false;
bool WatchdogIsAlive               =    false; // If this triggers, shutdown will appear right after
uint16_t TotalOutputPower          =    0;     // Calc of the Outputpower
uint16_t TotalInputPower           =    0;     // Calc of the Inputpower
int incomingByte                   =   53; // Serial read,  53=ascii 5 tal


boolean PowerRelayT1B =false;                     // Realy T1
boolean CapRelayT2B =false;                       // Realy T2
boolean WaveOnSW1B =false ;                        // Realy SW1 
boolean DischargeRelaySW2B =false;                // Realy SW2
boolean Fan1B = false;
boolean Fan2B = false;
byte ControllerStatus=0;



// ADC singleEnded input
uint16_t Read_MainPowerVoltageIN1       = 0; // direct to battery
uint16_t Read_CapPowerVoltageIN2        = 0; // direct to Big cap 47.000
uint16_t Read_PowerBoardVoltageIN3      = 0; // direct to incoming voltage to the powerboard
uint16_t Read_ToroidOutput230VoltageIN4 = 0; // direct to little transformer behind Toroid 230volt side


// ADC Current input from A0-A1 on the relay print
uint16_t Read_BatteryPowerCurrent       = 0;  // To current Shunt 20A = 75mv
uint16_t Read_ToroidOutput230Current    = 0;  // From little coil around the output 230v ac wire


// Analog input
uint16_t Read_5VSineWaveVoltageIN5      = 0; // Analog read from A0
uint16_t Read_12VSineWaveVoltageIN6     = 0; // Analog read from A1


//Default values
struct cell_module_config 
{
  // 7 bit slave I2C address
  uint8_t SLAVE_ADDR = DEFAULT_SLAVE_ADDR;
  // Den målte værdi fra ADC
  uint16_t ADC_VCC_Value = 4201;
  // Den målte værdi fra Tempproben
  uint16_t TemperatureProbe = 10;
};

static cell_module_config myConfig;

// For communications purpose
uint8_t Part1, Part2          = 0;
byte    EfficiencyInByte      = 0;
uint8_t CommandFromDisplay    = 0;
//float BatteryVoltage_Float = 0;
//------------------------------------------------ SETUP -----------------------------------------------------------------------
void setup() 
{
   /* short beep for turn on
   BeepOn();
   delay(500);
   BeepOff();
  */
  Serial.begin(115200);           // start serial for output
  wdt_disable();                  // not yet
  Serial.println("STARTING UP. WAIT.......");
  pinMode(StatusLedPin,      OUTPUT);  //Greenled
  pinMode(PowerRelayT1,      OUTPUT);  //Power relay
  pinMode(CapRelayT2,        OUTPUT);  //Capasitor relay
  pinMode(WaveOnSW1,         OUTPUT);  //Waveform on relay
  pinMode(DischargeRelaySW2, OUTPUT);  //Discharge capasitor relay
  pinMode(Beeper,            OUTPUT);  //Error beeper
  pinMode(FanUp,              OUTPUT);  //Fan1
  pinMode(FanDown,            OUTPUT);  //Fan2

  
  pinMode(TemperatureForHeatsinkPortA0, INPUT);
  pinMode(TemperatureForToroidPortA1,   INPUT);
  pinMode(TemperatureForCasePortA2,     INPUT);
  pinMode(SineWave12VoltPortA3,         INPUT);
  pinMode(SineWave5VoltPortA6,          INPUT);
  
  digitalWrite(PowerRelayT1,      LOW); //Turn off Power from battery
  digitalWrite(CapRelayT2,        LOW); //Turn off Power to Big Capasitor
  digitalWrite(DischargeRelaySW2, LOW); //Turn off Resistor to Big Capasitor  
  digitalWrite(WaveOnSW1,         LOW); //Turn off Waveform in Control Board
  digitalWrite(StatusLedPin,      LOW); //Turn off status led  
  digitalWrite(Beeper,            LOW); //Turn off beeper

  digitalWrite(FanDown, HIGH); //Skru helt ned for powersupply til blæser til 1volt. Det laveste den kan.
  Serial.print("Powersupply is beeing initialized.");
  for (byte i=0; i<6; i++)
  {
//    delay(1000);
    Serial.print(".");
  }
  digitalWrite(FanDown, LOW);  
  FanVolt          =    0; //Nul stiller tæller 
  Serial.println("Powersupply initialized DONE.");

  
  Wire.begin(); //SDA/SCL         // I2C setup  A4, A5
//  Wire.setTimeout(1000);          //1000ms timeout
  Wire.setClock(100000);          //100khz on the bus

/*
   // 3 short beep for turn on
   BeepOn();
   delay(200);
   BeepOn();
   delay(200);
   BeepOn();
   delay(200);
   BeepOff();
 */ 
    Serial.println("Checking EEprom");  
   //Load our EEPROM configuration, if bad write something
   if (!LoadConfigFromEEPROM()) 
     {
       Serial.println("No Config in EEprom. Module number = 21");  
       for (int led=0; led <=24; led++)
         {
           BeepOn();
           delay(50);
           BeepOn();
           delay(50);
         }
     }


   //Load our EEPROM configuration, and put it over in our global variables
   PacketNo=myConfig.SLAVE_ADDR;
   last_raw_adc=myConfig.ADC_VCC_Value;
   TemperatureForCase=myConfig.TemperatureProbe;


   //cli();   // disable all interrupts (including timer interrupts, serial interrupts, etc.).


//  initOled();          // Oled display startup
/* virker ikke under test
  initADC();           // call ads1115.begin();
  ReadTemp();          // read temperatur
  ReadVoltADC();           // læs spænding
  ReadCurrentADC();           // læs spænding
  AnalogReadVolt();        // read analog values 
  WriteDataToLCD();    // write data to LCD touch displauy
  
 */ 
  Serial.println();
  Serial.println("START UP. Setup done. Inverter Controller Software Version 2.3.2  3/1-2019");
  Serial.println();

  //init_i2c();
  wdt_enable(WDTO_4S); // Watchdog set to 4 sec
//  initTimer1();      // Timer for interrupt of OLED
  interrupts();        // or sei(); same same, will enable all interrupts
  
  // virker ikke under test
  // CheckBattVoltage();  // Check voltage turn GreenLed if ok
  
//  if(SineWaveOk) digitalWrite(WaveOnSW1, HIGH); // Turn SineWave for 230v on  
}

//----------------------------------------------EEPROM READ/WRITE-------------------------------------------------------
uint32_t calculateCRC32(const uint8_t *data, size_t length) // used in write & load configtoEEprom
{
  uint32_t crc = 0xffffffff;
  while (length--) {
    uint8_t c = *data++;
    for (uint32_t i = 0x80; i > 0; i >>= 1) {
      bool bit = crc & 0x80000000;
      if (c & i) {
        bit = !bit;
      }
      crc <<= 1;
      if (bit) {
        crc ^= 0x04c11db7;
      }
    }
  }
  return crc;
}

void WriteConfigToEEPROM() {
  EEPROM.put(EEPROM_CONFIG_ADDRESS, myConfig);
  EEPROM.put(EEPROM_CHECKSUM_ADDRESS, calculateCRC32((uint8_t*)&myConfig, sizeof(cell_module_config)));
}

bool LoadConfigFromEEPROM() {
  cell_module_config restoredConfig;
  uint32_t existingChecksum;

  EEPROM.get(EEPROM_CONFIG_ADDRESS, restoredConfig);
  EEPROM.get(EEPROM_CHECKSUM_ADDRESS, existingChecksum);

  // Calculate the checksum of an entire buffer at once.
  uint32_t checksum = calculateCRC32((uint8_t*)&restoredConfig, sizeof(cell_module_config));

  if (checksum == existingChecksum) {
    //Clone the config into our global variable and return all OK
    memcpy(&myConfig, &restoredConfig, sizeof(cell_module_config));
    return true;
  }

  //Config is not configured or gone bad, return FALSE
  return false;
}

void factory_default() {
  EEPROM.put(EEPROM_CHECKSUM_ADDRESS, 0);
}
//----------------------------------------- Beeper -------------------------------------------------------------------
void BeepOn() 
{ 
  tone(Beeper, 3000, 100); 
}
void BeepOff() 
{ 
  noTone(Beeper); 
}
//---------------------------------------------------------------------------------------------------------------------------------
void Shutdown(int ErrorValue)
{
  switch(ErrorValue)
  {
    case 10: {  Serial.print("Battery voltage to high or to low. ErrorValue: "); Serial.println(ErrorValue);} break; 
    case 11: {  Serial.print("5V control voltage to high or to low. ErrorValue: "); Serial.println(ErrorValue);} break; 
    case 12: {  Serial.print("12V control voltage to high or to low. ErrorValue: "); Serial.println(ErrorValue);} break; 
    case 13: {  Serial.print("Timeout for Big cap to charge up. ErrorValue: "); Serial.println(ErrorValue);} break; 
    case 14: {  Serial.print("Low voltage from toroid. Check coil and controller. ErrorValue: "); Serial.println(ErrorValue);} break;
    case 15: {  Serial.print("High voltage from toroid. Check coil and controller. ErrorValue: "); Serial.println(ErrorValue);} break;  
    case 16: {  Serial.print("Watchdog was triggered. ErrorValue: "); Serial.println(ErrorValue);} break;  
    case 17: {  Serial.print("Temperature on heatsink too high. ErrorValue: "); Serial.println(ErrorValue);} break;  
    case 18: {  Serial.print("Temperature on toroid too high. ErrorValue: "); Serial.println(ErrorValue);} break;  
    case 19: {  Serial.print("Temperature in case too high. ErrorValue: "); Serial.println(ErrorValue);} break;  
  default: break;
  }

  digitalWrite(PowerRelayT1, LOW);         //Turn off Power from battery
  delay(500);
  digitalWrite(CapRelayT2, LOW);           //Turn off Power to Big Capasitor
  delay(500);
  digitalWrite(DischargeRelaySW2, HIGH);   //Turn on the discharge resistor
  delay(2000);
  digitalWrite(DischargeRelaySW2, LOW);    //Turn off the discharge resistor again

  while(true)  //Blink for error. Hardware reset required
  {
    
    BeepOn();
    delay(50);
    
    BeepOff();
    delay(50);
    wdt_reset();           // Watchdog reset.
  }
}
//---------------------------------------Temperature on Battery---------------------------------------------------------------------
void ReadTemp() //Read temp from PTC sensor
{
   /////////////////////////////////////////////////////////////////////////
   // TemperatureForHeatsinkPortA0 = A0  Temperature from Power board     //
   // TemperatureForToroidPortA1   = A1  Temperature from Toroid          //
   // TemperatureForCasePortA2     = A2  Temperature from inside the case //
   /////////////////////////////////////////////////////////////////////////

  //http://www.circuitbasics.com/arduino-thermistor-temperature-sensor-tutorial/ (NTC thermistor)(købt på ebay:NTC-MF52-103 / 3435 10Kohm1%)
  //int ThermistorPin = 0;
  int Vo =0;
  float R1 = 10000;                //R1= 10k fast modstand
  float logR2, R2, T, Tc;          //R2= Termistor R2
  float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;


   
  for(int TemperaturePort=0;TemperaturePort<3;TemperaturePort++)
  {
    uint16_t value = 0;  
    for(int i=0;i<10;i++)         // taking 10 samples from sensors with a inerval of 2msec and then average the samples data collected
     {
       value   += analogRead(TemperaturePort); //read the voltage from Pin A0 to A2
       delay(2);
     }
    value=value/10;              //average of the 10 measurements

    Vo = value;
    R2 = R1 * (1023.0 / (float)Vo - 1.0);
    logR2 = log(R2);
    T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
    Tc = T - 273.15;

    
    switch (TemperaturePort)
     {
       case 0: TemperatureForHeatsink=Tc; //over in global variable break;
       case 1: TemperatureForToroid=Tc;   //over in global variable break;
       case 2: TemperatureForCase=Tc;     //over in global variable break;
     }  
  }  

/* Original code
  Vo = analogRead(ThermistorPin);
  R2 = R1 * (1023.0 / (float)Vo - 1.0);
  logR2 = log(R2);
  T = (1.0 / (c1 + c2*logR2 + c3*logR2*logR2*logR2));
  Tc = T - 273.15;
*/  

//  Serial.print("Heatsink Temperature: "); Serial.print(TemperatureForHeatsink); Serial.println(" C");   
//  Serial.print("Toroid Temperature: "); Serial.print(TemperatureForToroid); Serial.println(" C");   
  Serial.print("Case Temperature: "); Serial.print(TemperatureForCase); Serial.println(" C");   
  
}
//-----------------------------------------------------------------------------------------------------------
//***************************************//
//check all the voltages before powerup. //
// && = AND                              //  
// || = OR                               //
//***************************************//
void CheckBattVoltage()    
{
 bool Power5v_ok  = false; 
 bool Power12v_ok = false;

 if((Read_MainPowerVoltageIN1 > MIN_BATT_VOLTAGE)&&(Read_MainPowerVoltageIN1 < MAX_BATT_VOLTAGE))
  {
    digitalWrite(PowerRelayT1, HIGH);  //Turn on Power from battery
  }
 else
  Shutdown(10); //10 means mainpower to high or to low

 if((Read_5VSineWaveVoltageIN5 <= MAX_Control_5VOLTAGE)&&(Read_5VSineWaveVoltageIN5 >= MIN_Control_5VOLTAGE))
  {
    Power5v_ok=true;
    WriteDataToLCD();      // Write ADCvalue and analog value to LCD
  }
 else
  Shutdown(11); //11 means 5V power to high of to low

 if((Read_12VSineWaveVoltageIN6 <= MAX_Control_5VOLTAGE)&&(Read_12VSineWaveVoltageIN6 >= MIN_Control_5VOLTAGE)) 
  {
    if (Power5v_ok)
      Power12v_ok=true;
    WriteDataToLCD();      // Write ADCvalue and analog value to LCD
  }
 else
  Shutdown(12); //12 means 12V power to high of to low

 for (uint8_t WaitforChargeUp=0; WaitforChargeUp <=6; WaitforChargeUp++)
 {
   wdt_reset();           // Watchdog reset
   delay(1000);           // wait for charge up loop
   if((Read_CapPowerVoltageIN2 < MIN_CapVOLTAGE)&&(Read_CapPowerVoltageIN2 < MIN_CapVOLTAGE)) 
    {
      if ((Power12v_ok)&&(Power5v_ok))
        digitalWrite(CapRelayT2, HIGH);  //Turn on Power from battery
      WriteDataToLCD();      // Write ADCvalue and analog value to LCD 
    }
    
  if (WaitforChargeUp >5)  
   Shutdown(13); //13 means timeout for charge up of big cap
 }
 

 SineWaveOk=true;   // OK to power up the sinewave from the Control board
 WriteDataToLCD();  // Write ADCvalue and analog value to LCD
}
//------------------------------------------------------------------------------------------------------------
//***************************************//
//check the voltages of the toroid       //
//                                       //  
//                                       //
//***************************************//
void ReadToroidVoltage() 
{
  if (Read_ToroidOutput230VoltageIN4 < 190 )
    Shutdown(14); //14 means to low voltage from toroid. But the controller should detect this first. 
  if (Read_ToroidOutput230VoltageIN4 > 235 )
    Shutdown(15); //15 means to high voltage from toroid. But the controller should detect this first. 
}
//------------------------------------------------------------------------------------------------------------
//***************************************//
//check the temperature of the probes    //
// FanUp   A10   Fanblower for Heatsink  //
// FanDown A11   Fanblower for Toroid    //
//***************************************//
void ReadTemperature() 
{
  //Serial.print("FanVolt Temperature: ");Serial.println(FanVolt);

  if ((TemperatureForCase >=Low_CaseTemp)&&(TemperatureForCase< Min_CaseTemp))   //mellem 20 of 30C
   FanStepUp(25);
  if ((TemperatureForCase >=Min_CaseTemp)&&(TemperatureForCase< Mid_CaseTemp))  //mellem 31 of 40C
   FanStepUp(50);
  if ((TemperatureForCase >=Mid_CaseTemp)&&(TemperatureForCase< MAX_CaseTemp))  //mellem 41 of 50C
   FanStepUp(75);
  if (TemperatureForCase >=MAX_CaseTemp)  //over 50C
   FanStepUp(100);




  
/*
  if(TemperatureForHeatsink > MAX_HeatsinkTemp)
    Shutdown(17); //17 Temperature on heatsink too high.

  if(TemperatureForToroid > MAX_ToroidTemp)
    Shutdown(18); //18 Temperature on toroid too high.

  if(TemperatureForCase > MAX_CaseTemp)
    Shutdown(19); //19 Temperature in case too high.
*/
}
//------------------------------------------------------------------------------------------------------------
void FanUp1() // Manuelt 1 volt op
{
   digitalWrite(FanUp, HIGH);  
   delay(600);
   digitalWrite(FanUp, LOW); 
   Serial.println("Up done");
}

void FanUp3() // Manuelt 3 volt op
{
   digitalWrite(FanUp, HIGH);  
   delay(1600);
   digitalWrite(FanUp, LOW); 
   Serial.println("3v Up done");
}

void FanDown1()// Manuelt 1 volt ned
{
   digitalWrite(FanDown, HIGH);  
   delay(600);
   digitalWrite(FanDown, LOW);  
   Serial.println("Down done");
  
}
//------------------------------------------------------------------------------------------------------------
//******************************************//
//Control the powersupply through D10 & D11 //
// FanUp   D10                              //
// FanDown D11                              //
// FanVolt is global, and represent Fanspeed//
//******************************************//
void FanStepUp(byte FanSpeed) // 1 volt op
{
//  Serial.print("FanVolt = ");Serial.println(FanVolt);
//  Serial.print("FanSpeed = ");Serial.println(FanSpeed);
    wdt_reset();           // Watchdog reset
  if (FanVolt!=FanSpeed)
    {
      if (FanVolt<FanSpeed)
      {tone(Beeper, 2000, 100);
        switch (FanSpeed)
         { 
          case 5:  {digitalWrite(FanUp, HIGH); delay(700);  digitalWrite(FanUp, LOW); FanVolt=FanSpeed;break;}  //   5%  Idle. Den kører altid
          case 25: {digitalWrite(FanUp, HIGH); delay(1800); digitalWrite(FanUp, LOW); FanVolt=FanSpeed;break;}  //  25%  
          case 50: {digitalWrite(FanUp, HIGH); delay(1200); digitalWrite(FanUp, LOW); FanVolt=FanSpeed;break;}  //  50%  
          case 75: {digitalWrite(FanUp, HIGH); delay(1200); digitalWrite(FanUp, LOW); FanVolt=FanSpeed;break;}  //  75%  
          case 90: {digitalWrite(FanUp, HIGH); delay(1200); digitalWrite(FanUp, LOW); FanVolt=FanSpeed;break;}  //  90%  
          case 100:{digitalWrite(FanUp, HIGH); delay(1800); digitalWrite(FanUp, LOW); FanVolt=FanSpeed;break;}  // 100%  
         }         
      }

      if (FanVolt>FanSpeed)
      {tone(Beeper, 800, 100);
        switch (FanSpeed)
         { 
          case 5:  {digitalWrite(FanDown, HIGH); delay(700);  digitalWrite(FanDown, LOW); FanVolt=FanSpeed;break;}  //   5%  
          case 25: {digitalWrite(FanDown, HIGH); delay(1800); digitalWrite(FanDown, LOW); FanVolt=FanSpeed;break;}  //  25%  
          case 50: {digitalWrite(FanDown, HIGH); delay(1200); digitalWrite(FanDown, LOW); FanVolt=FanSpeed;break;}  //  50%  
          case 75: {digitalWrite(FanDown, HIGH); delay(1200); digitalWrite(FanDown, LOW); FanVolt=FanSpeed;break;}  //  75%  
          case 90: {digitalWrite(FanDown, HIGH); delay(1200); digitalWrite(FanDown, LOW); FanVolt=FanSpeed;break;}  //  90%  
          case 100:{digitalWrite(FanDown, HIGH); delay(1800); digitalWrite(FanDown, LOW); FanVolt=FanSpeed;break;}  // 100%  
         }         
      }

         
    }
  if (FanVolt>100)
    {Serial.print("100 Down. = ");  digitalWrite(FanDown, HIGH); delay(1800); digitalWrite(FanDown, LOW); FanVolt=FanSpeed; } //  50%  

}
//------------------------------------------------------------------------------------------------------------
void initADC()
{
  Serial.println("Getting single-ended readings from AIN0..3");
  Serial.println("ADC Range: +/- 6.144V (1 bit = 3mV)");

  ads1115VoltModule.begin();                 // start of ADC ADS1115
  ads1115CurrentModule.begin();              // start of Current ADS1115
  
  ads1115CurrentModule.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV  0.0078125mV
  ads1115VoltModule.setGain(GAIN_TWOTHIRDS);     // 2/3x gain +/- 6.144V  1 bit = 3mV (default)
  
}
//------------------------------------------------------------------------------------------------------------
// Read all voltages from ads1115 when called
void ReadVoltADC()
{
   float multiplier = 0.125F;           // // 16x gain  +/- 0.256V  1 bit = 0.125mV
    
// ads1115.setGain(GAIN_TWOTHIRDS);  // 2/3x gain +/- 6.144V  1 bit = 3mV (default)
// ads1115.setGain(GAIN_ONE);     // 1x gain   +/- 4.096V  1 bit = 2mV
// ads1115.setGain(GAIN_TWO);     // 2x gain   +/- 2.048V  1 bit = 1mV
// ads1115.setGain(GAIN_FOUR);    // 4x gain   +/- 1.024V  1 bit = 0.5mV
// ads1115.setGain(GAIN_EIGHT);   // 8x gain   +/- 0.512V  1 bit = 0.25mV
// ads1115.setGain(GAIN_SIXTEEN); // 16x gain  +/- 0.256V  1 bit = 0.125mV

  ads1115VoltModule.setGain(GAIN_SIXTEEN); // 16x gain  +/- 0.256V  1 bit = 0.125mV
   
  Read_MainPowerVoltageIN1       = ads1115VoltModule.readADC_SingleEnded(0);
  Read_CapPowerVoltageIN2        = ads1115VoltModule.readADC_SingleEnded(1);
  Read_PowerBoardVoltageIN3      = ads1115VoltModule.readADC_SingleEnded(2);
  Read_ToroidOutput230VoltageIN4 = ads1115VoltModule.readADC_SingleEnded(3);


  
  Serial.print("AIN0: "); Serial.println(Read_MainPowerVoltageIN1 * multiplier);
  Serial.print("AIN1: "); Serial.println(Read_CapPowerVoltageIN2 * multiplier);
  Serial.print("AIN2: "); Serial.println(Read_PowerBoardVoltageIN3 * multiplier);
  Serial.print("AIN3: "); Serial.println(Read_ToroidOutput230VoltageIN4 * multiplier);
  Serial.println(" ");

  
  //Hvilken værdi skal ind i ADCvalue ?
  ADCvalue = (Read_MainPowerVoltageIN1 * multiplier);         // put the result in the float to get all the digits
 // BatteryVoltage_Float=ADCvalue;                              // Put it in a Global value
  last_raw_adc = ADCvalue;                                    // convert the flot to an integer


}
//------------------------------------------------------------------------------------------------------------
void ReadCurrentADC()
{
 ADCvalue=0;
 results =0 ;
 float AmpsPrMillivolt = 0.190; // skulle egentligt være 0,266 da shunten er 20A og har 75mv pr amp  20/75= 0.266mv
 float multiplier = 0.125F; /* ADS1115  @ +/- 6.144V gain (16-bit results) */

  for(int i=0;i<10;i++)  // taking 10 samples from sensors with a inerval of 2msec and then average the samples data collected
    {
      results = ads1115CurrentModule.readADC_Differential_2_3();  
      delay(10);
    }
  results=results/10;         //Hiv gennemsnittet af de 10 målinger


   
 //To boost small signals, the gain can be adjusted 
 // ads1115VoltModule.setGain(GAIN_SIXTEEN);    // 16x gain  +/- 0.256V  1 bit = 0.125mV

   
  if (results<1) results = 0;
  if (results>6000) results = 0;  

  Serial.print("Differential: "); Serial.print(results); 
 
  ADCvalue=results*multiplier;   
  Serial.print("  Meassured mv: "); Serial.print(ADCvalue); Serial.print("mv");

  ADCvalue=ADCvalue*AmpsPrMillivolt; 
  Serial.print("  Meassured Amps: "); Serial.print(ADCvalue); Serial.println("Amps");

  Read_BatteryPowerCurrent = ADCvalue;


  Serial.print("Result from A0-A1: "); Serial.println(Read_BatteryPowerCurrent);
  Serial.println(" ");

  ADCvalue=0;
  
  last_raw_adc = ADCvalue;                   // convert the flot to an integer
}
//------------------------------------------------------------------------------------------------------------
void AnalogReadVolt() 
{
  Read_5VSineWaveVoltageIN5  = 0;
  Read_12VSineWaveVoltageIN6 = 0;
   
  for(int i=0;i<10;i++)         // taking 10 samples from sensors with a inerval of 2msec and then average the samples data collected
    {
  
      Read_5VSineWaveVoltageIN5  += analogRead(SineWave5VoltPortA6);
      Read_12VSineWaveVoltageIN6 += analogRead(SineWave12VoltPortA3);
      delay(2);
    }
    
   Read_5VSineWaveVoltageIN5=Read_5VSineWaveVoltageIN5/10;          //average of the 10 measurements
   Read_12VSineWaveVoltageIN6=Read_12VSineWaveVoltageIN6/10;        //average of the 10 measurements
}


//------------------------------------------------------------------------------------------------------------
void Simulatedata() 
{
  Read_5VSineWaveVoltageIN5      = random(4, 7);
  Read_12VSineWaveVoltageIN6     = random(10, 20);
  Read_BatteryPowerCurrent       = random(1, 99);
  Read_MainPowerVoltageIN1       = random(23, 30);
  Read_CapPowerVoltageIN2        = random(23, 30);
  Read_PowerBoardVoltageIN3      = random(23, 30); 
  Read_ToroidOutput230VoltageIN4 = random(220, 230);
  Read_ToroidOutput230Current    = random(1, 10);
  FanVolt                        = random(0, 100);
  TemperatureForHeatsink         = random(10, 80); 
  TemperatureForToroid           = random(10, 80); 
  TemperatureForCase             = random(10, 80); 
  TotalInputPower                = Read_BatteryPowerCurrent*Read_MainPowerVoltageIN1; // Show Watt for DC inputt
  TotalOutputPower               = Read_ToroidOutput230Current*Read_ToroidOutput230VoltageIN4; // Show Watt for AC Output
float MainPowerVoltage = random(23, 30);


float inputwatt  = Read_MainPowerVoltageIN1 * Read_BatteryPowerCurrent;
float outputwatt = Read_ToroidOutput230VoltageIN4 * Read_ToroidOutput230Current;

float Powerloss  = inputwatt - outputwatt;
float Efficiency = (outputwatt / inputwatt)*100; // 1232.8 div 1345.9)*100;  =91.6%

// byte EfficiencyInByte; //flyttes til global

EfficiencyInByte = (int)Efficiency;         // smider værdien før komma over i EfficiencyInByte. Så kun hele tal

// Lad os se på resultatet ... om det passer
//Serial.print("Efficiency: "); Serial.print(Efficiency); Serial.println("%"); 
//Serial.print("Efficiency in 8 bit: "); Serial.print(EfficiencyInByte); Serial.println("%"); 


  //Hvilken værdi skal ind i ADCvalue ?
  ADCvalue = 29.52;
  last_raw_adc = (ADCvalue*100);                                    // convert the flot to an integer
 
  MainPowerVoltage=MainPowerVoltage+0.23;  //Test 
  Read_MainPowerVoltageIN1=MainPowerVoltage*10;
  //Serial.print("Read_MainPowerVoltageIN1: "); Serial.println(Read_MainPowerVoltageIN1); 
/*
 * DL-CT08CL5 20A/10mA 2000/1 0~120A Micro Current Transformer
 * Rated Input Current: 0 ~ 20A
Rated Output Current: 0 ~ 10mA
Error Ratio (f): ≤ ± 0.1%
Phase Shift (Φ): ≤10 '
Accuracy: 0.1
Compressive Strength: ≥4KV
Chang-current: 0 ~ 80A (long-term operating current of less than 80A)
Linear Protection Current: 0 ~ 120A
Usage: When the primary current from P1 through the transformer secondary winding current flowing into the external circuit from S1
Application Range: 60A following meter dedicated; 0 ~ 80A single-phase / three-phase current detection; 0 ~ 120A single-phase / three-phase motor protection
Zero sequence / drain / remaining capacity detection and monitoring
 * 
 */
  
}
//------------------------------------------------------------------------------------------------------------
void WriteDataToLCD() // heres the Communication to the LCD Nano, with all the variables. Will show the voltage of the module on the LCD
{

/*  HUSK kun 8 byte af gangen = Max værdi 255  Ellers skal den deles i 2. Lige som 5+6 */
// Numbers of cases to look in to. Also number of sensor data.
#define MaxCount 19  // Numbers of transmitted sensor data
 
  for(int CounterForLCDSend=0;CounterForLCDSend<=MaxCount;CounterForLCDSend++)
  {
    //Serial.print("This is CounterForLCDSend "); Serial.println(CounterForLCDSend);
    switch (CounterForLCDSend)
     {
       case 0:   
         {Serial.print("This is A ");Serial.println(TemperatureForHeatsink);
          //Transmitting Heatsink Temperature. Should be around 30-60 degrees  
          Wire.beginTransmission(8);
          Wire.write("A");        // sends five bytes
          Wire.write(TemperatureForHeatsink);       // sends one byte
          Wire.endTransmission();         // stop transmitting  
          break;
         }
       case 1:   
         {//Serial.print("This is B ");Serial.println(TemperatureForToroid);
          //Transmitting Toroid Temperature. Should be around 30-60 degrees
          Wire.beginTransmission(8);
          Wire.write("B");        // sends five bytes
          Wire.write(TemperatureForToroid);       // sends one byte
          Wire.endTransmission();         // stop transmitting  
          break;
         }
       case 2:   
         {//Serial.print("This is C ");Serial.println(TemperatureForCase);
          //Transmitting Case Temperature. Should be around 30-60 degrees  
          Wire.beginTransmission(8);
          Wire.write("C");        // sends five bytes
          Wire.write(TemperatureForCase);       // sends one byte
          Wire.endTransmission();         // stop transmitting  
          break;
         }
       case 3:   
         {//Serial.print("This is D ");Serial.println(Read_BatteryPowerCurrent);
          //Transmitting Current from Controller board. Should be around 10-20Amps 
          Wire.beginTransmission(8);
          Wire.write("D");        // sends five bytes
          Wire.write(Read_BatteryPowerCurrent);       // sends one byte
          Wire.endTransmission();         // stop transmitting
          break;
         }
       case 4:   
         {//Serial.print("This is E ");Serial.println(FanVolt);
          //Transmitting Fan speed. Should be around 0-100 
          Wire.beginTransmission(8);
          Wire.write("E");        // sends five bytes
          Wire.write(FanVolt);       // sends one byte
          Wire.endTransmission();         // stop transmitting
          break;
         }
       case 5:   //AC Current
         {//can't send a integer, only byte. So this function is split up in 2. Case5 + case6, as how to send as integer
          Serial.print("This is F 5 TotalInputPower ="); Serial.println(TotalInputPower);
          //Serial.print("This is F 5 "); Serial.println("TotalInputPower part 1");
          //Part1 = ((TotalInputPower >> 8) & 0xFF);
          //Transmitting the calculated power from Read_BatteryPowerCurrent*Read_MainPowerVoltageIN1; // Show Watt for DC inputt
          //can't send a integer, only byte. So this function is how to send as integer
          Wire.beginTransmission(8);
          Wire.write("F");              // sends five bytes
          Wire.write((byte)((TotalInputPower >> 8) & 0xFF)); //part1
          Wire.endTransmission();       // stop transmitting
          break;
         }
       case 6:   
         { 
          //Part2=(TotalPower & 0xFF);
          //Transmitting the calculated power from Read_BatteryPowerCurrent*Read_MainPowerVoltageIN1; // Show Watt for DC inputt
          //Serial.print("This is G 6 "); Serial.println("TotalInputPower part 2");
          Wire.beginTransmission(8);
          Wire.write("G");                       // sends five bytes
          Wire.write((byte)(TotalInputPower & 0xFF)); // part 2 
          Wire.endTransmission();                // stop transmitting
          break;
         }
       case 7:   
         {//Serial.print("This is H ");Serial.println(Read_PowerBoardVoltageIN3);
          //Transmitting the incomming voltage to the PowerBoard. Should be around 24-30volt 
          Wire.beginTransmission(8);
          Wire.write("H");        // sends five bytes
          Wire.write(Read_PowerBoardVoltageIN3);       // sends one byte
          Wire.endTransmission();         // stop transmitting
          break;
         }
       case 8:   
         {//Serial.print("This is I ");Serial.println(Read_12VSineWaveVoltageIN6);
          //Transmitting 12V supply from Controller board. Should be around 12-15volt 
          Wire.beginTransmission(8);
          Wire.write("I");        // sends five bytes
          Wire.write(Read_12VSineWaveVoltageIN6);       // sends one byte
          Wire.endTransmission();         // stop transmitting
          break;
         }
       case 9:   
         {//Serial.print("This is J ");Serial.println(Read_5VSineWaveVoltageIN5);
          //Transmitting 5V supply from Controller board. Should be around 5volt
          Wire.beginTransmission(8);
          Wire.write("J");        // sends five bytes
          Wire.write(Read_5VSineWaveVoltageIN5);       // sends one byte
          Wire.endTransmission();         // stop transmitting
          break;
         }
       case 10:   
         {//Serial.print("This is K ");Serial.println(Read_ToroidOutput230VoltageIN4);
          //Transmitting 230V~supply from Totoid. Should be around 10V DC for 230AC volt 
          Wire.beginTransmission(8);
          Wire.write("K");        // sends five bytes
          Wire.write(Read_ToroidOutput230VoltageIN4);       // sends one byte
          Wire.endTransmission();         // stop transmitting
          break;
         }
       case 11:   
         { //Serial.print("This is L 11 EfficiencyInByte "); Serial.println(EfficiencyInByte); 
          Wire.beginTransmission(8);
          Wire.write("L");                       // sends five bytes
          Wire.write(EfficiencyInByte);          // Efficiency In a Byte. Kun heltal
          Wire.endTransmission();                // stop transmitting
          break;
         }
       case 12:   
         {//Serial.print("This is M ");Serial.println(Read_ToroidOutput230Current);
          //Transmitting Read_ToroidOutput230Current. Should be around 0-99  
          Wire.beginTransmission(8);
          Wire.write("M");        // sends five bytes
          Wire.write(Read_ToroidOutput230Current);       // sends one byte
          Wire.endTransmission();         // stop transmitting
          break;
         }
       case 13:   //second part of case 12
         {//Serial.print("This is N ");Serial.println(Read_CapPowerVoltageIN2);
          //Transmitting the incomming voltage to the Big capasitor. Should be around 24-30volt 
          Wire.beginTransmission(8);
          Wire.write("N");        // sends five bytes
          Wire.write(Read_CapPowerVoltageIN2);       // sends one byte
          Wire.endTransmission();         // stop transmitting
          break;
         }
       case 14:   
         {//can't send a integer, only byte. So this function is split up in 2. Case14 + case14, as how to send as integer
          Serial.print("This is O 14 TotalOutputPower ="); Serial.println(TotalOutputPower);
          //Serial.println("This is O 14 "); Serial.println(TotalOutputPower part 1);
          //Part1 = ((TotalOutputPower >> 8) & 0xFF);
          //Transmitting the calculated power from the board.
          Wire.beginTransmission(8);
          Wire.write("O");              // sends five bytes
          Wire.write((byte)((TotalOutputPower >> 8) & 0xFF)); //part1
          Wire.endTransmission();       // stop transmitting
          break;
         }

       case 15:   //second part of case 14
         { 
          //Part2=(TotalOutputPower & 0xFF);
          //Transmitting the calculated power from the board.
          //Serial.print("This is P 15 "); Serial.print("TotalOutputPower part 2");Serial.println(TotalOutputPower);
          Wire.beginTransmission(8);
          Wire.write("P");                       // sends five bytes
          Wire.write((byte)(TotalOutputPower & 0xFF)); // part 2 
          Wire.endTransmission();                // stop transmitting
          break;
         }

       case 16:   
         {//can't send a integer, only byte. So this function is split up in 2. Case16 + case17, as how to send as integer
          Serial.print("This is Q 16 "); Serial.println(Read_MainPowerVoltageIN1);
          Wire.beginTransmission(8);
          Wire.write("Q");              // sends five bytes
          Wire.write((byte)((Read_MainPowerVoltageIN1 >> 8) & 0xFF)); //part1
          Wire.endTransmission();       // stop transmitting
          break;
         }
       
       case 17:   
         {//Serial.print("This is R Read_MainPowerVoltageIN1 part 2");Serial.println(Read_MainPowerVoltageIN1);
          //Transmitting the incomming voltage from the battery. Should be around 24-30volt 
          Wire.beginTransmission(8);
          Wire.write("R");                       // sends five bytes
          Wire.write((byte)(Read_MainPowerVoltageIN1 & 0xFF)); // part 2 
          Wire.endTransmission();                // stop transmitting
          break;
         }     
       case 18:   
         { Serial.print("This is S = ControllerStatus = ");Serial.println(ControllerStatus);
           Wire.beginTransmission(8);
           Wire.write("S");                       // sends five bytes
           Wire.write(ControllerStatus);                        // Graphics will show warning
           Wire.endTransmission();                // stop transmitting
           break;
          }
       case 19:    //sender med command og får data retur -RequestEvent-
         {Serial.print("This is T = 22 med et retursvar = "); 
           Wire.beginTransmission(8);
           Wire.write("T");  //Command configure device address
           Wire.write(22);   
           Wire.endTransmission();                // stop transmitting           
           Wire.requestFrom(8,1); // transmit to device, receive 1 byte return
           CommandFromDisplay = Wire.read();
           //return svar;
           Serial.print("Svar retur = ");Serial.println(CommandFromDisplay);
//           if (CommandFromDisplay==99)
//            Shutdown(); //lukker ned
//           if (CommandFromDisplay==22)
//            StartUp(); //starter op
            
           break;
         }     
     
     
     }  
  wdt_reset();           // Watchdog reset
  //Serial.print("Transmitting "); Serial.println(CounterForLCDSend);
  //delay(500);
 }  
}
//------------------------------------------------------------------------------------------------------------
void Effenciency234() {

/*input volt 53,9
input amp 24,97
input watt 1345,9

output AC  231,3
output amp 5,33
output watt 1232,8
Power loss 113,1
Efficiency % 91,6


float inputwatt  = Read_MainPowerVoltageIN1 * Read_BatteryPowerCurrent;
float outputwatt = Read_ToroidOutput230VoltageIN4 * Read_ToroidOutput230Current;

float Powerloss  = inputwatt - outputwatt;
float Efficiency = (outputwatt / inputwatt)*100; // 1232.8 div 1345.9)*100;  =91.6

*/
}

//------------------------------------------------------------------------------------------------------------
void StartUp()
{
 CommandFromDisplay=0;
  Serial.println("Normal start op procudure. ");
  wdt_reset();           // Watchdog reset
  digitalWrite(PowerRelayT1, HIGH);
  //Serial.println("PowerRelayT1  ON ");
  Serial.println("Normal start op procudure.");
  Serial.print("Slowly charging the BIG Capasitor.");
  ControllerStatus=1;                         //vis start skærm mens den store lyt lader op
  Wire.beginTransmission(8);
  Wire.write("S");                       // sends five bytes
  Wire.write(ControllerStatus);          // Graphics will load GraphicSetup
  Wire.endTransmission();                // stop transmitting
  //WriteDataToLCD();                         // Write 230volt alert on the display
  for (byte i=0; i<6; i++)
    {
      BeepOn();
      delay(1000);
      wdt_reset();                           // Watchdog reset
      Serial.print(".");
    } 
  Serial.println(" ");

  
  //BeepOff();

  ControllerStatus=2;                    //vis status grafikken   
  
  Wire.beginTransmission(8);
  Wire.write("S");                       // sends five bytes
  Wire.write(ControllerStatus);          // Graphics will load GraphicSetup
  Wire.endTransmission();                // stop transmitting

  delay(500);
 
  tone(Beeper, 2200); // pin, hz, duration  Melder klar
  delay(100);
  tone(Beeper, 3000); // pin, hz, duration  Melder klar
  delay(100);
  digitalWrite(CapRelayT2, HIGH);
  Serial.println("CapRelayT2  HIGH ");
  tone(Beeper, 4100,200); // pin, hz, duration  Melder klar
  wdt_reset();           // Watchdog reset
  digitalWrite(WaveOnSW1, HIGH);
  Serial.println();
  Serial.println();
  Serial.println("WaveOnSW1  ON !!!  theres now 230V on the system.");
  Serial.println();
  Serial.println();
  Serial.println();
  
  ControllerStatus=4;                    //status grafikken vises allerede  
  Wire.beginTransmission(8);
  Wire.write("S");                       // sends five bytes
  Wire.write(ControllerStatus);          // Graphics will load GraphicSetup
  Wire.endTransmission();                // stop transmitting
  CommandFromDisplay=0;
}
//------------------------------------------------------------------------------------------------------------
void Shutdown()
{
  Serial.println("Normal lukke procudure. ");
  digitalWrite(WaveOnSW1, LOW);
  Serial.println("WaveOnSW1  OFF ");
  tone(Beeper, 1000, 100); // pin, hz, duration  Melder klar
  delay(300);
  tone(Beeper, 1000, 100); // pin, hz, duration  Melder klar
  digitalWrite(PowerRelayT1, LOW);
  Serial.println("PowerRelayT1  OFF ");
  delay(500);
  digitalWrite(CapRelayT2, LOW);
  Serial.println("CapRelayT2  OFF ");
  delay(500);
  wdt_reset();           // Watchdog reset
  digitalWrite(DischargeRelaySW2, HIGH);
  Serial.println("DischargeResistor SW2-relay:   TURN-ON");
  Serial.print("Slowly discharging the BIG Capasitor.");
  ControllerStatus=3;           //Show shutdown
  Wire.beginTransmission(8);
  Wire.write("S");                       // sends five bytes
  Wire.write(ControllerStatus);          // Graphics will load GraphicSetup
  Wire.endTransmission();                // stop transmitting
  delay(500);
  for (byte i=0; i<10; i++)
    {
      tone(Beeper, 2100,200); // pin, hz, duration  Melder klar
      delay(1000);
      wdt_reset();           // Watchdog reset
      Serial.print(".");
    } 
  digitalWrite(DischargeRelaySW2, LOW);
  Serial.println("DischargeResistor SW2-relay:   TURN-OFF");
  Serial.println(" ");
  tone(Beeper, 4100,200); // pin, hz, duration  Melder klar
  delay(100);
  tone(Beeper, 3000); // pin, hz, duration  Melder klar
  delay(100);
  tone(Beeper, 2200); // pin, hz, duration  Melder klar
  delay(150);
  BeepOff();
  wdt_reset();           // Watchdog reset
  Serial.println();
  Serial.println("Batteri kan nu fjernes.");
  Serial.println();
  Serial.println();
  ControllerStatus  =0; //Clear ControllerStatus
  CommandFromDisplay=0; //Clear CommandFromDisplay
}
//------------------------------------------------------------------------------------------------------------
/*Watchdogsetup_info
 * 
 * For Atmel MCUs found in the Arduino boards, the following prescaller constants were defined: 
 * WDTO_15MS, WDTO_30MS, WDTO_60MS, WDTO_120MS, WDTO_250MS, WDTO_500MS, 
 * WDTO_1S, WDTO_2S, WDTO_4S, and WDTO_8S. 
 * Their names are suggestive, indicating possible time slots between 15 milliseconds and 8 seconds.
 */


ISR(WDT_vect) // Watchdog timer interrupt.
{ 
 WatchdogIsAlive=true;
 digitalWrite(PowerRelayT1, LOW);     //Turn off Power from battery
}
//------------------------------------------------------------------------------------------------------------
void DebugMode(){
/*
#define PowerRelayT1                  4   // Realy T1
#define CapRelayT2                    5   // Realy T2
#define WaveOnSW1                     6   // Realy SW1 
#define DischargeRelaySW2             7   // Realy SW2
#define Beeper                        2   // Error beeper


digitalWrite(PowerRelayT1, HIGH);  //Turn on Power from battery
digitalWrite(CapRelayT2, HIGH);  //Turn on Power from battery
digitalWrite(WaveOnSW1, HIGH);  //Turn on Power from battery
digitalWrite(DischargeRelaySW2, HIGH);  //Turn on Power from battery

  digitalWrite(PowerRelayT1,      LOW); //Turn off Power from battery
  digitalWrite(CapRelayT2,        LOW); //Turn off Power to Big Capasitor
  digitalWrite(DischargeRelaySW2, LOW); //Turn off Resistor to Big Capasitor  
  digitalWrite(WaveOnSW1,         LOW); //Turn off Waveform in Control Board
  digitalWrite(StatusLedPin,      LOW); //Turn off status led  
  digitalWrite(Beeper,            LOW); //Turn off beeper

#define Fan1                         10   // Fanblower for Heatsink
#define Fan2                         11   // Fanblower for Toroid

*/



//void ReadSerialInput() //Husk at terminal vindue skal være med "No Line ending"

   if (Serial.available() > 0)    // for incoming serial data
   {
     incomingByte = Serial.read();

     if (incomingByte == 49) //  1
     {
       Serial.println("PowerRelayT1. Power for Powerboard");
       PowerRelayT1B =!PowerRelayT1B;
       if (PowerRelayT1B ==LOW)
       { digitalWrite(PowerRelayT1, HIGH);
        Serial.println("PowerRelayT1  HIGH ");
       }
       else
        {
        digitalWrite(PowerRelayT1, LOW);
        Serial.println("PowerRelayT1  LOW ");
        }
     }
     // hvis der sendes et 2 tal:

     if (incomingByte == 50)  
     {
       Serial.println("CapRelayT2.  Power for the big Cap");
       CapRelayT2B =!CapRelayT2B;
       if (CapRelayT2B ==LOW)
       {
         digitalWrite(CapRelayT2, HIGH);
         Serial.println("CapRelayT2  HIGH ");
       }  
       else
       {
        digitalWrite(CapRelayT2, LOW);
        Serial.println("CapRelayT2  LOW ");
       }  
     }

    // hvis der sendes et 3 tal: 
    if (incomingByte == 51)  
     {
       
       Serial.println("WaveOnSW1  Power the Sinewave");
       WaveOnSW1B =!WaveOnSW1B;
       if (WaveOnSW1B ==LOW)
       {
        digitalWrite(WaveOnSW1, HIGH); 
        Serial.println("WaveOnSW1  HIGH ");
       }
       else
        {
          digitalWrite(WaveOnSW1, LOW);
          Serial.println("WaveOnSW1  LOW ");
          
        }
     }
    
    // hvis der sendes et 4 tal: 
    if (incomingByte == 52)  
     {
       
       Serial.println("DischargeRelaySW2.   Discharge the Caps.");
       DischargeRelaySW2B =!DischargeRelaySW2B;
       if (DischargeRelaySW2B ==LOW)
       {
        digitalWrite(DischargeRelaySW2, HIGH);
        Serial.println("DischargeRelaySW2  HIGH ");
       }
       else
        {
          digitalWrite(DischargeRelaySW2, LOW);
          Serial.println("DischargeRelaySW2   LOW");
        }
       
     }



    if (incomingByte == 53)  //5
     {
       FanDown1();
       Serial.print("FanDown 1v");

     }

    if (incomingByte == 54)  //6
     {
       FanUp1();
       Serial.print("FanUp 1v");
      
     }

    if (incomingByte == 55)  //7
     {
      Serial.print("FanVolt = ");Serial.println(FanVolt);      
//      int tones[] = {261, 277, 294, 311, 330, 349, 370, 392, 415, 440};
//                    mid C  C#   D    D#   E    F    F#   G    G#   A
      tone(Beeper, 2610); // pin, hz, duration  Melder klar
      delay(100);
      tone(Beeper, 3490); // pin, hz, duration  Melder klar
      delay(100);
      tone(Beeper, 4400,200); // pin, hz, duration  Melder klar  
      delay(100);
      tone(Beeper, 8000); // pin, hz, duration  Melder klar
      delay(300);
      noTone(Beeper);
      delay(50);
      tone(Beeper, 4400,200); // pin, hz, duration  Melder klar  
      delay(100);
      tone(Beeper, 8000, 700); // pin, hz, duration  Melder klar
      delay(500);
      noTone(Beeper);
    delay(1000);
    for (byte i=0; i<40; i++)
     {  
      wdt_reset();           // Watchdog reset
      byte spil = random (1, 7);
      switch(spil)
      {
        case 1:{ tone(Beeper, 1500); delay(100); break;} // pin, hz, duration  Melder klar
        case 2:{ tone(Beeper, 2500); delay(100); break;} // pin, hz, duration  Melder klar
        case 3:{ tone(Beeper, 3500); delay(100); break;} // pin, hz, duration  Melder klar
        case 4:{ tone(Beeper, 4000); delay(100); break;} // pin, hz, duration  Melder klar
        case 5:{ tone(Beeper, 5000); delay(100); break;} // pin, hz, duration  Melder klar
        case 6:{ tone(Beeper, 6000); delay(100); break;} // pin, hz, duration  Melder klar
        case 7:{ tone(Beeper, 7000); delay(100); break;} // pin, hz, duration  Melder klar
       }   
      noTone(Beeper);
      delay(100);
     }
    }
     
     if (incomingByte == 56) //8  
     {
       Serial.println("Normal start op procudure. ");
       wdt_reset();           // Watchdog reset
       digitalWrite(PowerRelayT1, HIGH);
       Serial.println("PowerRelayT1  ON ");
       Serial.println("Normal start op procudure.");
       Serial.print("Slowly charging the BIG Capasitor.");
       ControllerStatus=1;
       WriteDataToLCD();                         // Write 230volt alert on the display
       for (byte i=0; i<25; i++)
         {
          BeepOn();
          Wire.beginTransmission(8);
          Wire.write("S");                       // sends five bytes
          Wire.write(0);                         // Graphics do nothing. only show last screen
          Wire.endTransmission();                // stop transmitting
          delay(1000);
          wdt_reset();                           // Watchdog reset
          Serial.print(".");
         } 
       Serial.println(" ");
       BeepOff();

       ControllerStatus=2;
       Wire.beginTransmission(8);
       Wire.write("S");                       // sends five bytes
       Wire.write(ControllerStatus);          // Graphics will load GraphicSetup
       Wire.endTransmission();                // stop transmitting

       tone(Beeper, 2200); // pin, hz, duration  Melder klar
       delay(100);
       tone(Beeper, 3000); // pin, hz, duration  Melder klar
       delay(100);
       digitalWrite(CapRelayT2, HIGH);
       Serial.println("CapRelayT2  HIGH ");
       tone(Beeper, 4100,200); // pin, hz, duration  Melder klar
       wdt_reset();           // Watchdog reset
       digitalWrite(WaveOnSW1, HIGH);
       Serial.println();
       Serial.println();
       Serial.println("WaveOnSW1  ON !!!  theres now 230V on the system.");
       Serial.println();
       Serial.println();
       Serial.println();
     }


    // hvis der sendes et 9 tal:  Luk normalt ned.
     if (incomingByte == 57)  
     {
       Serial.println("Normal lukke procudure. ");
       digitalWrite(WaveOnSW1, LOW);
       Serial.println("WaveOnSW1  OFF ");
       tone(Beeper, 1000, 100); // pin, hz, duration  Melder klar
       delay(300);
       tone(Beeper, 1000, 100); // pin, hz, duration  Melder klar
       digitalWrite(PowerRelayT1, LOW);
       Serial.println("PowerRelayT1  OFF ");
       delay(500);
       digitalWrite(CapRelayT2, LOW);
       Serial.println("CapRelayT2  OFF ");
       delay(500);
       wdt_reset();           // Watchdog reset
       digitalWrite(DischargeRelaySW2, HIGH);
       Serial.println("DischargeResistor SW2-relay:   TURN-ON");
       Serial.print("Slowly discharging the BIG Capasitor.");
       ControllerStatus=3;           //Show shutdown
       WriteDataToLCD();        // Write 230volt alert on the display
       for (byte i=0; i<10; i++)
         {
          tone(Beeper, 2100,200); // pin, hz, duration  Melder klar
          delay(1000);
          wdt_reset();           // Watchdog reset
          Serial.print(".");
         } 
       digitalWrite(DischargeRelaySW2, LOW);
       Serial.println("DischargeResistor SW2-relay:   TURN-OFF");
       Serial.println(" ");
       tone(Beeper, 4100,200); // pin, hz, duration  Melder klar
       delay(100);
       tone(Beeper, 3000); // pin, hz, duration  Melder klar
       delay(100);
       tone(Beeper, 2200); // pin, hz, duration  Melder klar
       delay(150);
       BeepOff();
       wdt_reset();           // Watchdog reset
       Serial.println();
       Serial.println("Batteri kan nu fjernes.");
       Serial.println();
       Serial.println();
     }

    
   }

} 


//------------------------------------------------------------------------------------------------------------
void loop() {
  wdt_reset();           // Watchdog reset
  ReadTemp();            // Read Temp 
  ReadVoltADC();         // Read ADC Volt
  ReadCurrentADC();      // Read ADC Current 
  AnalogReadVolt();      // Read analog volt values from 5VSineWaveVoltage & 12VSineWaveVoltage
  ReadToroidVoltage();   // Check voltage from toroid
  ReadTemperature();     // Check the tree temp sensors and update the powersupply for the fan
  Simulatedata();        // Simulate input
  DebugMode();           // Serial input. Turn on/off the relays, powersupply, fan mf.
  WriteDataToLCD();      // Write all data to the display

  if (CommandFromDisplay==99)
    Shutdown(); //lukker ned
  if (CommandFromDisplay==22)
    StartUp(); //starter op
 
//end of loop
}

