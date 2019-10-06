/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */

#pragma GCC diagnostic ignored "-Wwrite-strings"
#include <elapsedMillis.h>
#include <FlashStorage.h>
#include <stdarg.h>
#include <stdio.h>
#include <Wire.h>
#include <SPI.h>

#include <Arduino.h>
#include "wiring_private.h"
//Beginning of Auto generated function prototypes by Atmel Studio
void SERCOM1_Handler();
String printFormater(const char* str, ... );
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max);
bool Checksum(unsigned char* val1, unsigned char* val2, int length);
void FadeChannels();
byte sendCommand(char* CMD, byte attempts, byte timeout);
byte sendCommand(char* CMD);
int configurationMode();
void loadDataFromEEPROM();
bool hasSettingsChanged();
void loadDataToEEPROM();
void loadDataToDevices();
void CheckPower();
void resetBattery();
void CheckDMX();
void CheckLink();
void CheckWS2811();
void CheckID();
void Shutdown();
void batOperatingLED();
void setWIFILED();
void initManagement();
void BatteryManagment();
bool spi_transfer(unsigned char cmd, unsigned char* src, unsigned char* dst, int length);
unsigned char spi_command(unsigned char cmd);
void spi_payload(unsigned char* src, unsigned char* dst, int length);
void timo_send_DMX();
void checkAndParseUDP();
uint8_t gencrc(uint8_t *data, size_t len);
void setTimoChannels(byte channel);
void isdmxTX(bool TX);
void DMXactivity();
void sendArtpollReply();
void cycleWifi(byte mode);
void writeWANN(byte WANN);
void writeFVER(byte FVER);
void writeFVEW(byte FVEW);
void writeEcho();
void writeArtNet(bool enabled);
void writeDefault(bool enabled);
void writeFSSSID();
void writeFSKEY();
void writeFSENC();
void writeConfig();
void writeSSID();
void writePassword();
void writeReset();
void writeSecurity();
void writeEthernet();
void writeTCP();
void writeTcpPort();
void writeUDPinfo();
void writeDHCP();
void writeRELD();
void connectToSSID();
void writeSTASecurity();
void writeLANN(byte eth);
void writeMode(byte STA);
void writeFAPSTA(byte APSTA);
void writeWIFI(byte on);
void writeLang();
void writeNodeName();
void writeUniverse();
void writeTimoPower();
void writeChannelWidth();
void writeSecondChannel();
void writeBitSettings();
//End of Auto generated function prototypes by Atmel Studio

void FlashClass_erase(const volatile void *flash_ptr, uint32_t size);
void bootloader(void);

#define CYCLES_IN_DLYTICKS_FUNC        8
#define US_TO_DLYTICKS(us)          (uint32_t)(48000000 / 1000000 * us / CYCLES_IN_DLYTICKS_FUNC) // ((float)(F_CPU)) / 1000.0
#define MS_TO_DLYTICKS(ms)          (uint32_t)(48000000 / 1000 * ms / CYCLES_IN_DLYTICKS_FUNC) // ((float)(F_CPU)) / 1000.0
#define DelayTicks(ticks)            {volatile uint32_t n=ticks; while(n--);}//takes 8 cycles
#define DelayMs(ms)                    DelayTicks(MS_TO_DLYTICKS(ms))//uses 20bytes
#define DelayUs(us)                    DelayTicks(US_TO_DLYTICKS(us))//uses 20bytes


/* Timo Registry Values

   This sets all Timo registry values
   When a new registry is added it goes here
*/
#define CONFIG_REG          0x00
#define STATUS_REG          0x01
#define IRQ_MASK_REG        0x02
#define IRQ_FLAGS_REG       0x03
#define DMX_WINDOW_REG      0x04
#define ASC_FRAME_REG       0x05
#define SIGNAL_QUALITY_REG  0x06

#define DMX_SPEC_REG        0x08
#define DMX_CONTROL_REG     0x09

#define VERSION_REG         0x10
#define RF_POWER_REG        0x11
#define BLOCKED_CHANNELS    0x12

#define RGB_LED_COLOR	    0x33

#define READ_REG(x)         (x)
#define WRITE_REG(x)        (0x40 | x)
#define NO_OPERATION        0xFF
#define READ_DMX            0x81
#define READ_ASC            0x82
#define WRITE_DMX           0x91

/* Bitmasking
   used to to fit two bytes in one short
   allowing for easy parsing of complex data
*/
#define short_get_high_byte(x) ((HIGH_BYTE & x) >> 8)
#define short_get_low_byte(x)  (LOW_BYTE & x)
#define bytes_to_short(h,l) (((h << 8) & 0xff00) | (l & 0x00FF))

#define AKS_VER "1.F"

bool isLaserAKS = false;

//Flash(SAMD, 128 * 1024);


/* Protocols

   Used to designate multiple
   protocols for parsing

 * ********** Work in progress **********
*/
#define ArtNet              0x01
#define sACN                0x02
#define KiNET               0x03

//SPI
SPISettings settingsA(2000000, MSBFIRST, SPI_MODE0);

/* Timers
 * ********** currently not used **********
*/
elapsedMillis TimoConfigTimeout, DmxTimeout, DmxRefreshRate, BGReset;

/* Artnet Parameters

   Stores all values in relation to Art net
*/
char ArtNetName[18] = {'R', 'a', 't', 'p', 'a', 'c', ' ', 'A', 'K', 'S', ' ', ' ', ' ', ' ', ' ', ' ', ' ', ' '};
byte dmxBuffer[4][128];
byte dmxBuffer2[4][128];
byte smallBuffer[11];// Artnet Header
int framesRecived, framesDropped;
bool trueForArtnet = true;
unsigned char ArtnetIP[4] = {10, 10, 100, 254};
bool ethLinked = false;
/* Timo Parameters

   Stores all values in relation to the Timo

   For old boards (IRQ = 7, CS = 6)
   For Beta and final boards (IRQ = 12, CS = 10)((IRQ = 9, CS = 8;))
*/
//byte IRQ = 7, CS = 6;
//byte IRQ = 12, CS = 10;
byte IRQ = 9, CS = 8;
byte timoConfigStep = 0;
byte timoPowerLevel = 3;
byte StatusREG;
byte timoConfigValue = 130;
byte channelWidth = 15;
unsigned char timoBlockedChannel[11] = {255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255};
bool bootToUpdate = false;
bool updateLED = true;
bool isTimoOn = true;
bool timoTogglePower = false;


byte hasToUpdateTimoSettings = 255;

typedef struct
{
	bool isValid;
	//Timo
	unsigned char timo_Config;
	unsigned char timo_IRQ_Mask;
	unsigned char timo_DMX_Window[4];
	unsigned char timo_DMX_Spec[8];
	unsigned char timo_DMX_Control;
	unsigned char timo_RF_Power;
	unsigned char timo_Blocked_Channel[11];

	//Battery 0b00110000
	unsigned char Battery[8];
	byte wifi_Mode;

} Settings;

//FlashStorage(settingsInEEPROM, Settings);
#define EEPROM_ADDRESS (80 * 1024)
FlashStorageClass<Settings> settingsInEEPROM((void *)EEPROM_ADDRESS);

Settings settingsBeforeLoad;
Settings settingsAfterLoad;

/* Power Button

   Stores on values in relation to the power button
*/
elapsedMillis powerTimePressed, powerBlinkTimer;
bool powerCurrent = LOW, powerPrevious = LOW;
bool powerLED = true, hasBooted = false;
#define PowerLEDPin 3
byte powerClickCount = 0;

/* Link Button

   Stores values in relation to the link button
*/
elapsedMillis linkTimePressed;
bool linkCurrent;
bool linkPrevious = HIGH;
bool linkLongPress = false;

//DMX Led
elapsedMillis dmxFrameTimeout;
elapsedMillis XLRdmxFrameTimeout;
bool gettingDMX = false;
bool gotFullFrame = false;

//Battery
const int numReadings = 100;    // number of samples
int readings[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int total = 0;                  // the running total
int average = 0;                // the average
int lowest, highest;            // lowest and highest battery values
int inputPin = A5;              // battery sense pin
int BatLimit = 750;             // value to be considered (low battery)
int BatHysteresis = 50;         // value hysteresis
bool BatLED = false;            // batter LED value
byte BatLEDPin = A4;            // LED pin
byte batteryBlinkMode = 9;
byte ledstatus = 0;
elapsedMillis batteryTimer;

//WiFi
elapsedMillis wifiTimeout, blinkTimeout, WIFILEDTimer;
boolean isWifiOn = true, isSTA = false, isAPSTA = false;
byte wifiCycle = 0;
byte wifiLEDPin = A3;
byte WIFIledstatus = 0;
byte wifiLedBlinkMode = 0;
byte wifimode = 0;
bool isWifiAlive = false;


elapsedMillis IDTimeout = 0;
bool IDEnable = false;
byte IDLEDS, IDTicks = 0;

uint8_t pixelData[3] = {0, 0, 0};


//Random
bool up = false;//Sets the direction of the fade method

////Second Uart
Uart Serial2 (&sercom1, 12, 10, SERCOM_RX_PAD_3, UART_TX_PAD_2);
byte DMXStage = 0;
int DMXch = 0;
byte DMXsc = 0;

void SERCOM2_Handler()
{
  if(SERCOM2->USART.INTFLAG.bit.ERROR)
  {
      SERCOM2->USART.INTFLAG.bit.ERROR = 1;
      if (SERCOM2->USART.STATUS.bit.FERR)    // Test for frame error
      {
		  if(1);
        return;
      }
  }
}


void SERCOM1_Handler()
{
  bool isRX = SERCOM1->USART.INTFLAG.bit.RXC;
  byte DATA = SERCOM1->USART.DATA.reg;
  //gettingDMX = true;

  if(SERCOM1->USART.INTFLAG.bit.ERROR)
  {
      SERCOM1->USART.INTFLAG.bit.ERROR = 1;
      if (SERCOM1->USART.STATUS.bit.FERR)    // Test for frame error
      {
        SERCOM1->USART.STATUS.reg = SERCOM_USART_STATUS_RESETVALUE;
        DMXStage = 1;
        XLRdmxFrameTimeout = 0;
        gettingDMX = true;
        if(DMXch)
        {
           memcpy(dmxBuffer, dmxBuffer2,sizeof(dmxBuffer));
           gotFullFrame = true;
        }
        DMXch = 0;
        return;
      }
  }
  //Serial2.IrqHandler();
  if(isRX)
  {
    switch (DMXStage)
    {
      case 0:               // Startup - Ignore Data
        break;
      case 1:               // First Byte After Break
        if(DATA == 0)DMXStage++;
        else DMXStage = 0;
        break;
      case 2:               // Save Data for Selected Channels
        dmxBuffer2[(DMXch) / 128][(DMXch) % 128] = DATA;
        DMXch++;
        if(DMXch == 512)DMXStage++;
        break;
      case 3:
        break;
    }
  }
}

#if 0
volatile int loop_flag = 1;

void loop_dummy(void)
{
	loop_flag = 0;
}
#endif

volatile unsigned char eeprom_flag;

void setup()
{
  if (isLaserAKS)
  {
    IRQ = 12;
    CS = 10;
  }
  pinMode(A0, OUTPUT);// Sets up the system latch
  digitalWrite(A0, HIGH);// Hold system on
  
#if 0  
  while (loop_flag)
  {
	  loop_dummy();
  }
#endif
  
  pinMode (PowerLEDPin, OUTPUT);// Sets up the Power LED
  digitalWrite(PowerLEDPin, HIGH);// Power LED
  pinMode(A1, INPUT_PULLUP); //PullUp for power button detector
  pinMode(13, INPUT_PULLUP); //PullUp for link button detector
  if (!digitalRead(13))bootToUpdate = true;
  pinMode(11, OUTPUT); // Sets up the DMX led
  digitalWrite(11, LOW);// Turns off DMX led
  
  pinMode(26, OUTPUT);
  
  SerialUSB.begin(250000);// Sets up USB interface
  Serial1.begin(230400);// Sets up UART interface
  Serial2.begin(250000, SERIAL_8N2);// Sets up XLR interface
  pinPeripheral(10, PIO_SERCOM);
  pinPeripheral(12, PIO_SERCOM);
  //SERCOM0->USART.INTENSET.reg |= SERCOM_USART_INTENSET_DRE;
  Wire.begin(); // join i2c bus (address optional for master)
  Wire.setClock(400000L);


  loadDataFromEEPROM();
  
  eeprom_flag = (unsigned char)(settingsBeforeLoad.isValid);
  if ((eeprom_flag != 0) && (eeprom_flag != 1))
  {
	  settingsBeforeLoad.isValid = false;		// first time, the data is 0xff
  }

  if (!settingsBeforeLoad.isValid)
  {
    settingsBeforeLoad.isValid = true;
    settingsBeforeLoad.timo_Config = 0b10000010;
    settingsBeforeLoad.timo_IRQ_Mask = 0b00000000;
    unsigned char timo_DMX_Window[4] = {0b00000000, 0b00000000, 0b00000010, 0b00000000};
    memcpy( settingsBeforeLoad.timo_DMX_Window, timo_DMX_Window, 4);

    unsigned char timo_DMX_Spec[8] = {0b00000000, 0b00000010, 0b00000000, 0b00000000, 0b00000000, 0b00000000, 0b01100001, 0b10101000};
    memcpy( settingsBeforeLoad.timo_DMX_Spec, timo_DMX_Spec, 8);

    settingsBeforeLoad.timo_DMX_Control = 0b00000001;
    settingsBeforeLoad.timo_RF_Power = 0b00000101;
    unsigned char timo_Blocked_Channel[11] = {255, 255, 255, 0,0,0,0,0, 255, 255, 255};
    memcpy( settingsBeforeLoad.timo_Blocked_Channel, timo_Blocked_Channel, 11);

    unsigned char Battery[8] = {0b00110100, 0b00011011, 0b01100000, 0b00010001, 0b10110010, 0b10011010, 0b00000011, 0b01001011};
    memcpy( settingsBeforeLoad.Battery, Battery, 8);
	settingsBeforeLoad.wifi_Mode = 1;
	
	
    settingsInEEPROM.write(settingsBeforeLoad);
    loadDataFromEEPROM();
  }
//settingsAfterLoad = settingsBeforeLoad;
  pinMode (5, OUTPUT);// Sets up the DMX RX/TX
  pinMode (6, OUTPUT);
  isdmxTX(false);
  //Wire.begin();
  SPI.begin();// Sets us the SPI
  SPI.beginTransaction(settingsA);// Starts an SPI Transaction
  pinMode (BatLEDPin, OUTPUT);
  pinMode (A3, OUTPUT);
  pinMode (CS, OUTPUT);// Sets up CS pin
  digitalWrite(CS, HIGH);// Sets CS HIGH
  pinMode(IRQ, INPUT_PULLUP);// Sets up IRQ pin

  digitalWrite(BatLEDPin, HIGH);
  delay(100);
  digitalWrite(BatLEDPin, LOW);
  initManagement();


//
//spi_transfer(WRITE_REG(DMX_SPEC_REG), timo_DMX_Spec, NULL, 8);
//    bitClear(hasToUpdateTimoSettings, 3);
//    delayMicroseconds(300);
//
byte buffer[1];
buffer[0] = 1;//settingsAfterLoad.timo_DMX_Control;
    spi_transfer(WRITE_REG(DMX_CONTROL_REG), buffer, NULL, 1);


    





  if (average <= 975)
  {
    delay(500);
    digitalWrite(BatLEDPin, HIGH);
    delay(100);
    digitalWrite(BatLEDPin, LOW);
    delay(100);
    digitalWrite(BatLEDPin, HIGH);
    delay(100);
    digitalWrite(BatLEDPin, LOW);
  }
  if (average < 680)
  {
    digitalWrite(BatLEDPin, HIGH);
    delay(1000);
    Shutdown();
  }
  batOperatingLED();
  loadDataToDevices();
  Serial1.setTimeout(100);
  //wifimode = settingsAfterLoad.wifi_Mode;
  if (bootToUpdate)
  { // boot into update mode
    resetBattery();
    settingsBeforeLoad.isValid = false;
    settingsInEEPROM.write(settingsBeforeLoad); 
    digitalWrite(11, HIGH);
    delay(10000);
        configurationMode();
        writeConfig();
        //loadDataToDevices();
        //digitalWrite(A3, HIGH);
    while (true)
    {
          if (blinkTimeout > 250 && wifiTimeout > 10000)
           { // flash LED
             blinkTimeout = 0;
             updateLED = !updateLED;
             digitalWrite(11, updateLED);// Turns off DMX led
           }
          CheckPower();// check power
          CheckLink();
          BatteryManagment();
          //DMXactivity();
          //CheckDMX();
          //Serial2.write('0');
    }
  }
  
  
  // 3: Ethernet Mode, 2: Sta mode, 1: AP mode
  if (1 == settingsAfterLoad.wifi_Mode)
  {
	  return;
  }
  elapsedMillis timeElapsed = 0;
  while(timeElapsed < (8000))
  {
	  if(Serial1.find("_reboot"))
	  {
		  digitalWrite(wifiLEDPin, HIGH);
		  delay(500);
		  digitalWrite(wifiLEDPin, LOW);
		  delay(500);
		  break;
	  }
	  digitalWrite(wifiLEDPin, HIGH);
	  delay(50);
	  digitalWrite(wifiLEDPin, LOW);
	  delay(50);
  }
  digitalWrite(wifiLEDPin, HIGH);
  delay(500);
  digitalWrite(wifiLEDPin, LOW);
  delay(500);
  configurationMode();//Enter config mode
  writeEcho();	//Turn off Echo
  if (3 == settingsAfterLoad.wifi_Mode)
  {
	  writeWIFI(false);  // Turn off wifi for ethernet mode
  }
  writeArtNet(true);
  char CMD[] = "ENTM";
  sendCommand(CMD);
  
}

elapsedMillis timer;
int ie;
bool wifiReset = false;
void loop()
{
	CheckWS2811();
  CheckPower();
  CheckID();
  CheckLink();
  //CheckWS2811();
  DMXactivity();
  CheckDMX();
  //Serial2.write('0');
  setWIFILED();
  BatteryManagment();
  batOperatingLED();
  loadDataToDevices();
  delay(2);
  checkAndParseUDP();
  delay(2);
}


/* printFormater
 *  
 * String formater 
 * Same as Java printf  
 * ********** currently not used **********
 */
String printFormater(const char str[], ...)
{
  String string;
  char temp[16+1];
  int i, j;
  va_list argv;
  va_start(argv, str);
  for(i=0,j=0; str[i]!='\0';i++)
  {
    if(str[i]=='%')
    {
      temp[j] = '\0';
      string += temp;
      j=0;
      temp[0] = '\0';
      switch(str[++i])
      {
        case 'd':  string += va_arg(argv, int);
                  break;
        case 'l':  string += va_arg(argv, long);
                  break;
        case 'f':  string += va_arg(argv, double);
                  break;
        case 'c':  string += (char)va_arg(argv, int);
                  break;
        case 's':  string += va_arg(argv, char *);
                  break;
        default:  ;
      };
    }
    else 
    {
      temp[j] = str[i];
      j = (j+1)%16;
      if(j==0) 
      {
        temp[16] = '\0';
        string += temp;
        temp[0] = '\0';
      }
    }
  };
  return string;
}

/* mapfloat
 *  
 * Same as arduino map funtion but for floats  
 * ********** currently not used **********
 */
float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

/* Checksum
 *  
 * Checks if all values in val1 and val2 are the same 
 */
bool Checksum(unsigned char* val1, unsigned char* val2, int length)
{
  for (int i = 1; i <= length; i++)// Loop through all entities
  {
    if(*val1++ != *val2++) return false;// if they arent the same return false
  }
  return true;// return true
}

/* FadeChannels
 *  
 * Fades the channels for debug mode 
 * ********** currently not used **********
 */
void FadeChannels()
{
    if(dmxBuffer[0][0] == 255 || dmxBuffer[0][0] == 0)// if it hits a limit
    {
      up = !up;// reverse direction
    }
    for (int x=0;x<4;x++)// loop through segments
    {
      for (int i=0;i<128;i++)// loop though channels in segment
      {
        if(up)// if incrementing
        {
          dmxBuffer[x][i]+=1;// increment
        }
        else// else
        {
          dmxBuffer[x][i]-=1;// decrement
        }
      }
    }
}

void fatal_error(void)
{
	while (1)
	{
		CheckPower();
		digitalWrite(wifiLEDPin, HIGH);
		delay(500);
		digitalWrite(wifiLEDPin, LOW);
		delay(500);
	}
}

byte sendCommand(char CMD[])
{
	sendCommand(CMD, 2, 10);
}

byte sendCommand(char CMD[], byte attempts, byte timeout)
{
	byte gotReply = 0;//0 == Nothing
	while (Serial1.available() > 0) Serial1.read();
	for (u_int8_t i = 0; i < attempts; i++)
	{
		if(gotReply == 2)break;
		Serial1.print("AT+");//send command
		Serial1.print(CMD);
		Serial1.print("\r");
		//if(CMD[0] != "Z")Serial1.print("\n\r\n");
		Serial1.flush();
		elapsedMillis timeElapsed = 0;
		while(timeElapsed < (timeout * 1000))
		{
			digitalWrite(PowerLEDPin, LOW);
			delay(25);
			digitalWrite(PowerLEDPin, HIGH);
			delay(25);
			if(Serial1.find("+ok"))
			{
				gotReply = 2;//2 == OK
				break;
			}
			#if 0
			if(Serial1.find("+ERR"))
			{
				gotReply = 1;//2 == ERROR
				break;
			}
			#endif
		}
	}
	digitalWrite(PowerLEDPin, LOW);
	delay(100);
	digitalWrite(PowerLEDPin, HIGH);
	delay(100);
	if (2 != gotReply)
	{
		fatal_error();
	}
	return gotReply;
}
int configurationMode()
{//Put HF-A11 into configuration mode
	while (Serial1.available() > 0) Serial1.read();
	elapsedMillis timeElapsed = 0;
	bool gotReply = false;
	Serial1.print("+++");
	Serial1.flush();
	while(timeElapsed < 2500 && gotReply == false)
	{
		if(Serial1.find("a"))gotReply = true;
	}
	if(!gotReply)
	{
		fatal_error();
		return 0;
	}
	gotReply = false;
	delay(10);
	Serial1.print("a");
	Serial1.flush();
	while(timeElapsed < 2500 && gotReply == false)
	{
		if(Serial1.find("+ok"))gotReply = true;
	}
	delay(25);
	if(!gotReply)
	{
		fatal_error();
		return 1;
	}
	return 2;
}

void loadDataFromEEPROM()
{
  settingsBeforeLoad = settingsInEEPROM.read();
  settingsAfterLoad = settingsBeforeLoad;
  settingsAfterLoad.Battery[0] = 0b00110111;
  settingsAfterLoad.Battery[2] = 0b01111100;
}

bool hasSettingsChanged()
{
  bool hasChanged = false;
  if(settingsBeforeLoad.isValid != settingsAfterLoad.isValid)hasChanged = true;
  if(settingsBeforeLoad.timo_Config != settingsAfterLoad.timo_Config)hasChanged = true;
  if(settingsBeforeLoad.timo_IRQ_Mask != settingsAfterLoad.timo_IRQ_Mask) hasChanged = true;
  if(settingsBeforeLoad.timo_DMX_Window != settingsAfterLoad.timo_DMX_Window) hasChanged = true;
  if(settingsBeforeLoad.timo_DMX_Spec != settingsAfterLoad.timo_DMX_Spec) hasChanged = true;
  if(settingsBeforeLoad.timo_DMX_Control != settingsAfterLoad.timo_DMX_Control) hasChanged = true;
  if(settingsBeforeLoad.timo_RF_Power != settingsAfterLoad.timo_RF_Power) hasChanged = true;
  if(settingsBeforeLoad.timo_Blocked_Channel != settingsAfterLoad.timo_Blocked_Channel) hasChanged = true;

  //Battery
  if(settingsBeforeLoad.Battery != settingsAfterLoad.Battery) hasChanged = true;

  return hasChanged;
}

void loadDataToEEPROM()
{
  if(hasSettingsChanged)
  {
    Serial1.print("Writing EEPROM");
    settingsInEEPROM.write(settingsAfterLoad);
  }
}

void loadDataToDevices()
{
  uint8_t buffer[1];
  if(bitRead(hasToUpdateTimoSettings, 4))
  {
    buffer[0] = 1;//settingsAfterLoad.timo_DMX_Control;
    spi_transfer(WRITE_REG(DMX_CONTROL_REG), buffer, NULL, 1);
    bitClear(hasToUpdateTimoSettings, 4);
    delayMicroseconds(1000);
  }
  if(bitRead(hasToUpdateTimoSettings, 0))
  {
    buffer[0] = settingsAfterLoad.timo_Config;
    spi_transfer(WRITE_REG(CONFIG_REG), buffer, NULL, 1);
    bitClear(hasToUpdateTimoSettings, 0);
    delayMicroseconds(1000);
  }
  /*
  if(bitRead(hasToUpdateTimoSettings, 1))
  {
    buffer[0] = settingsAfterLoad.timo_IRQ_Mask;
    spi_transfer(WRITE_REG(IRQ_MASK_REG), buffer, NULL, 1);
    bitClear(hasToUpdateTimoSettings, 1);
    delayMicroseconds(300);
  }
  
  if(bitRead(hasToUpdateTimoSettings, 2))
  {
    spi_transfer(WRITE_REG(DMX_WINDOW_REG), settingsAfterLoad.timo_DMX_Window, NULL, 4);
    bitClear(hasToUpdateTimoSettings, 2);
    delayMicroseconds(300);
  }
 
  if(bitRead(hasToUpdateTimoSettings, 3))
  { 
    spi_transfer(WRITE_REG(DMX_SPEC_REG), settingsAfterLoad.timo_DMX_Spec, NULL, 8);
    bitClear(hasToUpdateTimoSettings, 3);
    delayMicroseconds(300);
  }
  */
  if(bitRead(hasToUpdateTimoSettings, 6))
  {
    spi_transfer(WRITE_REG(BLOCKED_CHANNELS), settingsAfterLoad.timo_Blocked_Channel, NULL, 11);
    bitClear(hasToUpdateTimoSettings, 6);
    delayMicroseconds(1000);
  }
  if(bitRead(hasToUpdateTimoSettings, 5))
  {
    buffer[0] = settingsAfterLoad.timo_RF_Power;
//	if(buffer[0] < 3) buffer[0] = 3; // Limit AKS Lite to 65mW
    spi_transfer(WRITE_REG(RF_POWER_REG), buffer, NULL, 1);
    bitClear(hasToUpdateTimoSettings, 5);
    delayMicroseconds(1000);
  }
  if(bitRead(hasToUpdateTimoSettings, 7))
  {
    for (int i=0; i <= 7; i++)
    {
      Wire.beginTransmission(0x6b);
      Wire.write(i);
      Wire.write(settingsAfterLoad.Battery[i]); 
      Wire.endTransmission(); 
      delayMicroseconds(300);
    } 
  }
}

void CheckPower()
{
  powerCurrent = digitalRead(A1);// get value of button
  if(!hasBooted) // if the AKS has booted (if the button has been released since first boot)
  {// Hold values low
    powerTimePressed = 0;
    powerBlinkTimer = 0;
    powerClickCount = 0;
  }
  if(powerCurrent == HIGH)hasBooted = true;// Sets booted
  if (powerCurrent == LOW && powerPrevious == HIGH)
  { // reset values
    powerPrevious = powerCurrent;
    powerTimePressed = 0;
    powerBlinkTimer = 0;
  }
  if (powerCurrent == HIGH && powerPrevious == LOW && hasBooted)
  {// On release
    powerPrevious = powerCurrent;
    if (powerTimePressed >= 2000)
    {
      Shutdown();
    }
    if(powerTimePressed >= 100 && powerTimePressed <= 500)
    {
      powerClickCount++;
    }
    powerTimePressed = 0;
  }
  if (powerCurrent == LOW && powerTimePressed >= 2000 && hasBooted)
  {// On Press
    if (powerBlinkTimer >= 100)
    {// blink LED
      powerLED = !powerLED;
      digitalWrite(PowerLEDPin, powerLED);
      powerBlinkTimer = 0;
    }
  }
  if(powerTimePressed >= 500)
  {
    if(powerClickCount == 2)
    {
		cycleWifi(1);
    }
	if(powerClickCount == 3)
	{
		cycleWifi(0);
	}
	if(powerClickCount == 5)
	{
		cycleWifi(2);
	}
    powerClickCount = 0;
  }
}
void resetBattery()
{
    char i = settingsAfterLoad.Battery[1];
    bitSet(i, 7);
    Wire.beginTransmission(0x6b);
    Wire.write(1);
    Wire.write(i); 
    Wire.endTransmission(); 
    delayMicroseconds(300);
}
void CheckDMX()
{
  if (dmxFrameTimeout > 500 && !gettingDMX)
  {
	dmxFrameTimeout = 0;
    digitalWrite(11, LOW);// Turns off DMX led
  }
}

void CheckLink()
{
  uint8_t buffer[1];
  linkCurrent = digitalRead(13);// Get value of link switch
  if (linkCurrent == LOW && linkPrevious == HIGH)
  {// on pressed
    linkLongPress = false;
    linkPrevious = linkCurrent;
    linkTimePressed = 0;
  }
  if (linkCurrent == HIGH && linkPrevious == LOW)
  {// on release
    linkPrevious = linkCurrent;

    if (linkTimePressed >= 100 && linkTimePressed <= 1000 && !linkLongPress)
    {//if held for linking
      buffer[0] = 2;
      spi_transfer(WRITE_REG(STATUS_REG), buffer, NULL, 1);
      if(bootToUpdate)SerialUSB.println("Linking");
    }
    linkLongPress = false;
    linkTimePressed = 0;
  }
  if (linkCurrent == LOW && linkTimePressed >= 3000 && !linkLongPress)
  {//if held for unlinking
    linkLongPress = true;
    buffer[0] = 1;
    spi_transfer(WRITE_REG(STATUS_REG), buffer, NULL, 1);
    if(bootToUpdate)SerialUSB.println("Unlinking");
  }
}

void CheckWS2811()
{
	uint8_t bitMask;
	noInterrupts();// cli();
	for (uint16_t p=0; p<3; p++)
	{
		bitMask = 0x80;
		while (bitMask)
		{
			uint8_t value = pixelData[p] & bitMask;
			PORT->Group[0].OUTSET.reg = PORT_PA27;
			if (value)
			{	
				DelayTicks(2)
				PORT->Group[0].OUTCLR.reg = PORT_PA27;
				DelayTicks(0)			
			}else
			{
				asm("NOP");
				asm("NOP");
				asm("NOP");
				asm("NOP");
				asm("NOP");
				asm("NOP");
				asm("NOP");
				PORT->Group[0].OUTCLR.reg = PORT_PA27;
				DelayTicks(2)
				asm("NOP");
				asm("NOP");
				asm("NOP");
				asm("NOP");
				asm("NOP");
			}
			bitMask >>= 1;
		}
		
	}
	interrupts();
}

void CheckID()
{
	if(!IDEnable)return;

	if(!IDTicks)
	{
		digitalWrite(PowerLEDPin, HIGH);// Power LED
		IDEnable = false;
		return;
	}
	if(IDTimeout > 100)
	{
		IDTicks--;
		IDTimeout = 0;
		digitalWrite(BatLEDPin, !digitalRead(BatLEDPin));
		digitalWrite(wifiLEDPin, !digitalRead(wifiLEDPin));
		digitalWrite(PowerLEDPin, !digitalRead(PowerLEDPin));
		digitalWrite(11, !digitalRead(11));
		
		//wifiLEDPin PowerLEDPin
		
	}
}

void Shutdown()
{
    loadDataToEEPROM();
	
	pixelData[0] = pixelData[1] = pixelData[2] = 0;
	
	CheckWS2811();
	
    delay(100);
	digitalWrite(A0, LOW);
	//NVIC_SystemReset();
}
void batOperatingLED()
{
  int batteryBlinkOnTimes[4] = {2000,1000,500,100};
  if(ledstatus == 0)
  {
    batteryBlinkMode = 5;
    if(average >= 730) batteryBlinkMode = 4;
    if(average >= 780) batteryBlinkMode = 3;
    if(average >= 800) batteryBlinkMode = 2;
    if(average >= 830) batteryBlinkMode = 1;
    if(average >= 915) batteryBlinkMode = 0;
    ledstatus = 1;
    batteryTimer = 0;
  }
  if(ledstatus != 0)
  {
    if(batteryBlinkMode == 0)
    {
      if(ledstatus == 1 && batteryTimer >= 5000)
      {
        digitalWrite(BatLEDPin, LOW);
        ledstatus = 0;
        batteryTimer = 0;
      }
    }

    if(batteryBlinkMode == 1)
    {
      if(ledstatus == 1 && batteryTimer >= 100)
      {
        digitalWrite(BatLEDPin, LOW);
        ledstatus = 2;
        batteryTimer = 0;
      }
      if(ledstatus == 2 && batteryTimer >= 2000)
      {
        digitalWrite(BatLEDPin, HIGH);
        ledstatus = 0;
        batteryTimer = 0;
      }
    }
    if(batteryBlinkMode == 2)
    {
      if(ledstatus == 1 && batteryTimer >= 100)
      {
        digitalWrite(BatLEDPin, LOW);
        ledstatus = 2;
        batteryTimer = 0;
      }
      if(ledstatus == 2 && batteryTimer >= 1000)
      {
        digitalWrite(BatLEDPin, HIGH);
        ledstatus = 0;
        batteryTimer = 0;
      }
    }
    if(batteryBlinkMode == 3)
    {
      if(ledstatus == 1 && batteryTimer >= 100)
      {
        digitalWrite(BatLEDPin, LOW);
        ledstatus = 2;
        batteryTimer = 0;
      }
      if(ledstatus == 2 && batteryTimer >= 500)
      {
        digitalWrite(BatLEDPin, HIGH);
        ledstatus = 0;
        batteryTimer = 0;
      }
    }
    if(batteryBlinkMode == 4)
    {
      if(ledstatus == 1 && batteryTimer >= 100)
      {
        digitalWrite(BatLEDPin, LOW);
        ledstatus = 2;
        batteryTimer = 0;
      }
      if(ledstatus == 2 && batteryTimer >= 100)
      {
        digitalWrite(BatLEDPin, HIGH);
        ledstatus = 0;
        batteryTimer = 0;
      }
    }
    if(batteryBlinkMode == 5 && batteryTimer >= 1000)
    {
      digitalWrite(BatLEDPin, HIGH);
      ledstatus = 0;
      batteryTimer = 0;
    }
  }
}
void setWIFILED()
{
	uint8_t timeSlice = (millis()/100) % 10;
	switch (wifimode)
	{
		case 1:
			digitalWrite(wifiLEDPin, HIGH);
		break;
		case 2:	
			digitalWrite(wifiLEDPin, (ethLinked) ? (!timeSlice || timeSlice == 2) : (!timeSlice));
		break;
		case 3:
			digitalWrite(wifiLEDPin, (ethLinked) ? !(!timeSlice || timeSlice == 2) : (timeSlice));
		break;
		case 4:
			digitalWrite(wifiLEDPin, timeSlice % 2);
		break;
		default:
			digitalWrite(wifiLEDPin, LOW);
		break;
	}
}
void initManagement()
{
  lowest = analogRead(inputPin);
  for (int thisReading = 0; thisReading < numReadings; thisReading++)
  {
    total = total - readings[readIndex];//Removes previous value
    readings[readIndex] = analogRead(inputPin);// read from the sensor:
    if(isLaserAKS)readings[readIndex] * 0.636364;
    total = total + readings[readIndex];// add the reading to the total:
    readIndex++;// advance to the next position in the array:
    if (readIndex >= numReadings)// if we're at the end of the array
    {
      readIndex = 0;// wrap around to the beginning:
    }
    average = total / numReadings;// calculate the average:
  }
}
void BatteryManagment()
{
  if(wifiTimeout > 10000 && !isWifiAlive) wifimode = 4;
  /*
  if(!wifiReset && isWifiAlive)
  {
    configurationMode();
    writeMode();
    writeWIFI();
    writeReset();
    wifiReset = true;
  }
  */
  total = total - readings[readIndex];// subtract the last reading:
  readings[readIndex] = analogRead(inputPin);// read from the sensor:
  if(isLaserAKS)readings[readIndex] * 0.636364;
  total = total + readings[readIndex];// add the reading to the total:
  readIndex = readIndex + 1;// advance to the next position in the array:
  if (readIndex >= numReadings)// if we're at the end of the array
  {
    readIndex = 0;//wrap around to the beginning:
  }

  // calculate the average:
  average = total / numReadings;
  // send it to the computer as ASCII digits

//  if(average < BatLimit)
//  {// if low battery
//    BatLED = true;
//    digitalWrite(BatLEDPin, HIGH);
//  }
//  else if(average > BatLimit + BatHysteresis)
//  {
//    BatLED = false;
//    digitalWrite(BatLEDPin, LOW);
//  }

  
  if(average > highest)highest = average;
  if(average < lowest)lowest = average;
  if(bootToUpdate)
  {
    SerialUSB.print(" ");
    SerialUSB.print(0);
    SerialUSB.print(" ");
    SerialUSB.print(1023);
    SerialUSB.print(" ");
    SerialUSB.print(highest);
    SerialUSB.print(" ");
    SerialUSB.print(lowest);
    SerialUSB.print(" ");
    
    SerialUSB.print(average);
    SerialUSB.print(" ");
    SerialUSB.print(readings[readIndex]); 
    SerialUSB.print(" "); 
  }
  if(average < BatLimit - BatHysteresis) Shutdown();
}

//void ChargerManagment()
//{
//  Wire.beginTransmission(107); // transmit to device #44 (0x2c)
//  // device address is specified in datasheet
//  Wire.write(byte(0x00));            // sends instruction byte
//  //Wire.write(val);             // sends potentiometer value byte
//  Wire.endTransmission();     // stop transmitting
//}


bool spi_transfer(unsigned char cmd, unsigned char* src, unsigned char* dst, int length)
{
  spi_command(cmd);						// send the command
  for (int i=0; i<10000; i++)		//
  {
    if(!digitalRead(IRQ))				// when the IRQ flips
    {
		delayMicroseconds(10);			// wait   
        spi_payload(src, dst, length);	// send the payload
        return true;
    }
  }
  return false;
}
unsigned char spi_command(unsigned char cmd)
{
  uint8_t Response = 0x80;
  while (Response & 0x80)
  {
    digitalWrite(CS, LOW);// drop the CS pin
    delayMicroseconds(10);// wait       
    Response = SPI.transfer(cmd);// send the command
    digitalWrite (CS, HIGH);// rise the CS pin
    for (int i=0; i<10000; i++)
    {//wait for IRQ rise
      if (digitalRead(IRQ))
      {
        break;
      }
    }
    if(!(Response & 0x80))return Response;
    delay(100);
  }
  return Response; // returning the byte we read from SPI
}

void spi_payload(unsigned char* src, unsigned char* dst, int length) {
  uint8_t Response = 0x80;
  while (Response & 0x80)
  {
    digitalWrite (CS, LOW);// drop the CS pin
    delayMicroseconds(10);// wait   
    Response = SPI.transfer(0x00); // send the command
    if (Response & 0x80)
    {// if busy
      digitalWrite (CS, HIGH);
      for (int i=0; i<10000; i++)
      {
          __asm__("nop\n\t");// wait 
      }
    }
  }
  while (length-- > 0)//go through all bytes
  {
    if (src == NULL)
    {
      Response = SPI.transfer(0x00);
    } else
    {
      Response = SPI.transfer(*src++);
    }
    if (dst != NULL) {
      *dst++ = Response;
    }
  }
  digitalWrite (CS, HIGH);
  for (int i=0; i<10000; i++)
  {//wait for IRQ rise
    if (digitalRead(IRQ))
    {
      break;
    }
  }
}
void timo_send_DMX()
{
  spi_transfer(WRITE_DMX, dmxBuffer[0], NULL, 128);
  delayMicroseconds(500);        
  spi_transfer(WRITE_DMX, dmxBuffer[1], NULL, 128);
  delayMicroseconds(500);       
  spi_transfer(WRITE_DMX, dmxBuffer[2], NULL, 128);
  delayMicroseconds(500);      
  spi_transfer(WRITE_DMX, dmxBuffer[3], NULL, 128);
  delayMicroseconds(500); 
}

void checkAndParseUDP()
{
     if(trueForArtnet)
     {
          if(Serial1.available() >= 5 && Serial1.find("Art-Net"))
          {// if contains header value
               byte ArtnetBuffer[5];
               Serial1.readBytes(ArtnetBuffer, 5);
               switch (bytes_to_short(ArtnetBuffer[2], ArtnetBuffer[1]))
               {// switch operations
                    case 0x5000: //ArtNetDMX
                         byte ArtDmxBuffer[6];
                         Serial1.readBytes(ArtDmxBuffer, 6);
						 if(gettingDMX)break;
                         //digitalWrite(11, HIGH);// Turns on DMX led
                         for (int i=0; i<10000; i++)
                         {//Wait for full frame
                              delayMicroseconds(10);// wait  
                              if(!(Serial1.available() < 512))
                              {
                                   break;
                              }
                         }
                         if(Serial1.available() < 512){return;}//skip frame if missing full frame
                         for (int x=0;x<4;x++)
                         {//loop through all channels
                              for (int i=0;i<128;i++)
                              {        
                                   dmxBuffer[x][i] = Serial1.read();
                              }
                         }
                         //SerialUSB.print(dmxBuffer[0][0]);
                         digitalWrite(11, HIGH);// Turns on DMX led
                         timo_send_DMX();// send frame
                         break;
                    case 0x2000:             
                         for (int i=0; i<5000; i++)
                         {
                              if(Serial1.available() >= 2)break;
                              delayMicroseconds(10);// wait  
                         }
                         byte ArtPollBuffer[2];
                         Serial1.readBytes(ArtPollBuffer, 2);
                         sendArtpollReply();
                         //char Timo[] = {'A','r','t','-','N','e','t',0,0,50,0,0, 0, 0b10000010, 0b00000000;
                         
                         break;
                    case 0x3200: 
                         for (int i=0; i<5000; i++)
                         {
                              if(Serial1.available() >= 2)break;
                              delayMicroseconds(10);// wait  
                         }
						 byte wifiBuffer[35];
						 Serial1.readBytes(wifiBuffer, 1);
                         switch (wifiBuffer[0])
                         {
							case 0: {
								for (int i=0; i<5000; i++)
								{
									if(Serial1.available() >= 35)break;
									delayMicroseconds(10);// wait
								}
							 /*
								1-Config
								1-IRQ Mask
								4-DMX Window
								8-DMX Spec
								1-DMX Control
								1-RF Power
								11-Blocked_Channel
								8-Battery
								*/
								Serial1.readBytes(wifiBuffer, 35);
								if(settingsAfterLoad.timo_RF_Power != wifiBuffer[15])
								{
									bitSet(hasToUpdateTimoSettings, 5);
									settingsAfterLoad.timo_RF_Power = wifiBuffer[15];
								}
								if(memcmp(wifiBuffer + 16, settingsAfterLoad.timo_Blocked_Channel, sizeof(settingsAfterLoad.timo_Blocked_Channel)) != 0)
								{
									bitSet(hasToUpdateTimoSettings, 6);
									memcpy(settingsAfterLoad.timo_Blocked_Channel, wifiBuffer + 16, 11);
								}
								//Serial1.println("OK");
                           }break;
                           case 1: { //artpoll Data in
								for (int i=0; i<5000; i++)
								{
									if(Serial1.available() >= 1)break;
									delayMicroseconds(10);// wait
								}
								ethLinked = Serial1.read();
								//Serial1.println("OK");
                           }break;
						   // 5 = battery
						   // 6 gaffers
						   
						
						   
                           case 3: {
								for (int i=0; i<5001; i++)
								{
									if(Serial1.available() >= 2)break;
									delayMicroseconds(10);// wait
								}
								/*wifimode =*/ //Serial1.read();
								byte link = Serial1.read();
								unsigned char buffer[1]; 
								switch (link)
								{
									case 1:
										delay(5);
										buffer[0] = 1;
										spi_transfer(WRITE_REG(STATUS_REG), buffer, NULL, 1);
									break;
									case 2:
										delay(5);
										/* Your code here */
										buffer[0] = 2;
										spi_transfer(WRITE_REG(STATUS_REG), buffer, NULL, 1);
									break;
									default:/* Invalid */
										IDTimeout = 0;
									break;
								}
								Serial1.println("ok");
							}break;
							case 4: {
								if (!IDEnable)
								{
									IDEnable = true;
									IDTimeout = 0;
									IDTicks = 100;
									//TODO: Remember LEDS
									IDLEDS = 0;
									IDLEDS |= (digitalRead(BatLEDPin))<<3 |(digitalRead(wifiLEDPin))<<2 |(digitalRead(PowerLEDPin))<<1 |(digitalRead(11));										
								} 
								else
								{
									IDEnable = false;
									//TODO: Reset LEDS						
									digitalWrite(BatLEDPin, bitRead(IDLEDS, 3));
									digitalWrite(wifiLEDPin, bitRead(IDLEDS, 2));
									digitalWrite(PowerLEDPin, bitRead(IDLEDS, 1));
									digitalWrite(11, bitRead(IDLEDS, 0));
									
								}
								Serial1.println("ok");
							}break;
							case 5:{      // return Battery and version	
								 String stringThree = String(constrain(map(average, 700, 995, 0, 100), 0, 99));
								 Serial1.println(AKS_VER + stringThree);
							} break;
							case 6:{
								for (int i=0; i<5000; i++)
								{
									if(Serial1.available() >= 8)break;
								 	delayMicroseconds(10);// wait
								}
								
							    byte GafferBuffer[5];
							    Serial1.readBytes(GafferBuffer, 5);
								IDTimeout = 0;
								
								unsigned char colorbuffer[3]; 
								
								colorbuffer[0] = pixelData[1] = Serial1.read();
								colorbuffer[1] = pixelData[0] = Serial1.read();
								colorbuffer[2] = pixelData[2] = Serial1.read();
								
								spi_transfer(WRITE_REG(RGB_LED_COLOR), colorbuffer, NULL, 3);
								
								CheckWS2811();
								
								
							}break;
							
							case 7:
							{
								// do reset to jump to bootloader
								// Disable all interrupts
								bootloader();
								
								#if 0
								__disable_irq();
								// Reset the device
								// NVIC_SystemReset() ;

								while (true);
								#endif
								
								#if 0
								uint32_t * ptr_reset_vector;
								uint32_t * ptr_msp;
								uint32_t app_start_add;

								__disable_irq();
								//Get reset vector from intvect table of application
								ptr_reset_vector = (uint32_t *) (0+4);
								app_start_add = (*ptr_reset_vector);
								ptr_msp = (uint32_t *) (0);
								__set_MSP(*(ptr_msp));
								
								FlashClass_erase((const volatile void *)0x4000, 128);
				
								/* Rebase the vector table base address */
								SCB->VTOR = ((uint32_t) 0 & SCB_VTOR_TBLOFF_Msk);
				
								/* Load the Reset Handler address of the application */

								//application_code_entry = (void (*)(void))(unsigned *)(*(unsigned *)(FIRMWARE_START_ADDR +4));
				
								/* Jump to user Reset Handler in the application */

								//application_code_entry();
								asm("bx %0"::"r"(app_start_add));								
								#endif
							} break;
							default: {/* Invalid */
								Serial1.println("INV");
							}break;
						}
						if(!isWifiAlive)wifimode = settingsAfterLoad.wifi_Mode;
                        isWifiAlive = true;
					break;
                    default: 
                         SerialUSB.print("Unknown ArtNet OpCode: ");
                         SerialUSB.print(bytes_to_short(ArtnetBuffer[2], ArtnetBuffer[1]));
                    break;
              }
          }
     }
}

uint8_t gencrc(uint8_t *data, size_t len)
{
	uint8_t crc = 0x00;
	for (size_t i = 0; i < len; i++)
    {
            crc ^= data[i];
    }
	return crc;
}

void setTimoChannels(byte channel)
{
  if(channel == 0)
  {



  }else
  {
       byte actualChannel[11] = {12,17,22,27,32,37,42,47,52,57,62};
       byte channelTotal = 0;
       for (int x=0; x<11; x++)
       {
            for (int y=0; y<8; y++)
            {
                 (channelTotal < (actualChannel[channel-1] - channelWidth) || channelTotal > (actualChannel[channel-1] + channelWidth))? bitClear(timoBlockedChannel[x], y) : bitSet(timoBlockedChannel[x], y);// CLEAR THEN SET
                 channelTotal++;
            }
       }
  }
}

void isdmxTX(bool TX)
{
  digitalWrite(5, TX);// Power LED
  digitalWrite(6, TX);// Power LED
}

void DMXactivity()
{
  if(gettingDMX)
  {
    if(gotFullFrame)
    {
      gotFullFrame = false;
      timo_send_DMX();
    }
    if(XLRdmxFrameTimeout > 1000)
    {
      digitalWrite(11, HIGH);// Turns off DMX led
      gettingDMX = false;
      DMXch = 0;
    }else
    {
      digitalWrite(11, HIGH);
    }
  }
}

void sendArtpollReply()
{
     uint8_t artnetID[] = {'A', 'r', 't', '-', 'N', 'e', 't', 0x00};
     // byte ArtPollBuffer[18];
     Serial1.write(artnetID, sizeof(artnetID)); // ID[8]
     Serial1.write((uint8_t)0x00);
     Serial1.write((uint8_t)0x21);
     
     
     Serial1.write((uint8_t)ArtnetIP[0]);//ip
     Serial1.write((uint8_t)ArtnetIP[1]);//ip
     Serial1.write((uint8_t)ArtnetIP[2]);//ip
     Serial1.write((uint8_t)ArtnetIP[3]);//ip
     
     Serial1.write((uint8_t)0x36);//port
     Serial1.write((uint8_t)0x19);
	 
	 Serial1.write((uint8_t)12);//vers
	 Serial1.write((uint8_t)18);
     
	 Serial1.write((uint8_t)0); // NetSwitch
	 Serial1.write((uint8_t)0); // SubSwitch
	 
	 
     Serial1.write((uint8_t)0x00); // OemHi
     Serial1.write((uint8_t)0xFF); // OemLo
	 
     Serial1.write((uint8_t)0); // Ubea Version
     Serial1.write((uint8_t)0); // Status1
	 
     Serial1.write(0xD7); // EstaManLo
     Serial1.write(0x51); // EstaManHi
     
     Serial1.write(ArtNetName, 18);//short name       
     //Serial1.write("AKS                                                             ");
     Serial1.write("Ratpac AKS - Designed by Ratpac Dimmers");
	 
	 
	 
	 
	 
	 
	 for (int i = 0; i < 25 ; i++)Serial1.write((uint8_t)0);//null
	 for (int i = 0; i < 64 ; i++)Serial1.write((uint8_t)0);//NodeReport
	 
  Serial1.write((uint8_t)0);//Numports
  Serial1.write((uint8_t)1);

  Serial1.write((uint8_t)0b10000000);//PortTypes
  Serial1.write((uint8_t)0);
  Serial1.write((uint8_t)0);
  Serial1.write((uint8_t)0);

  Serial1.write((uint8_t)0b10000000);//GoodInput
  Serial1.write((uint8_t)0b00001000);
  Serial1.write((uint8_t)0b00001000);
  Serial1.write((uint8_t)0b00001000);
  
  Serial1.write((uint8_t)0b10000000);//GoodOutput
  Serial1.write((uint8_t)0);
  Serial1.write((uint8_t)0);
  Serial1.write((uint8_t)0);
  
  Serial1.write((uint8_t)0);//SwIn
  Serial1.write((uint8_t)0);
  Serial1.write((uint8_t)0);
  Serial1.write((uint8_t)0);
  
  Serial1.write((uint8_t)0);//SwOut
  Serial1.write((uint8_t)0);
  Serial1.write((uint8_t)0);
  Serial1.write((uint8_t)0);
  
  Serial1.write((uint8_t)0);//SwVideo
  Serial1.write((uint8_t)0);//SwMacro
  
  Serial1.write((uint8_t)0);//SwRemote
  
  Serial1.write((uint8_t)0);//Zeros
  Serial1.write((uint8_t)0);//Zeros
  Serial1.write((uint8_t)0);//Zeros
  
  Serial1.write((uint8_t)0);//Style


  Serial1.write((uint8_t)0);//mac
  Serial1.write((uint8_t)0);
  Serial1.write((uint8_t)0);
  Serial1.write((uint8_t)0);
  Serial1.write((uint8_t)0);
  Serial1.write((uint8_t)0);
  
   Serial1.write((uint8_t)0);
   Serial1.write((uint8_t)0);
   Serial1.write((uint8_t)0);
   Serial1.write((uint8_t)0);
   
   Serial1.write((uint8_t)0);
   
   Serial1.write((uint8_t)15);
   
   for (int i = 0; i < 26 ; i++)Serial1.write((uint8_t)0);//null
  
}

void cycleWifi(byte mode)
{
	//Blink LEDS
	digitalWrite(PowerLEDPin, LOW);
	delay(250);
	digitalWrite(PowerLEDPin, HIGH);
	delay(250);
	digitalWrite(PowerLEDPin, LOW);
	delay(250);
	digitalWrite(PowerLEDPin, HIGH);
	
	configurationMode();//Enter config mode
	writeEcho();	//Turn off Echo
	// writeArtNet(false);
	
	byte WANN = 1;
	wifimode = mode + 1; //Fix wifimode indexing
	settingsAfterLoad.wifi_Mode = wifimode;//apply settings to new wifimode
	
	switch (mode)
	{
		case 1:	//STA MODE
		isWifiOn = true;
		isSTA = true;
		break;
		case 2://ETH MODE
		WANN = 0;
		isWifiOn = false;
		isSTA = false;
		break;
		default://AP MODE
		isWifiOn = true;
		isSTA = false;
		break;
	}
	
	
	if(mode == 2)writeFVEW(true);
	else writeFVEW(false);
	
	writeRELD();
	
	for (int i = 0; i < 400 ; i++)
	{
		digitalWrite(PowerLEDPin, LOW);
		delay(25);
		digitalWrite(PowerLEDPin, HIGH);
		delay(25);
		if(Serial1.find("_reb")) break;
	}
	digitalWrite(PowerLEDPin, LOW);
	delay(500);
	digitalWrite(PowerLEDPin, HIGH);
	delay(500);
	configurationMode();
	writeEcho();
	// writeArtNet(false);
	
	writeMode(isSTA);
	if (mode == 1 || mode == 2)     // Ethernet or Sta mode
	{
		writeLANN(1);
	}
	else
	{
		writeLANN(0);
	}
	//writeLANN(!isWifiOn);
	
	//Now Things are different
	char CMD[] = "ENTM";
	switch (mode)
	{
		case 1:	//STA MODE
		writeSTASecurity();
		connectToSSID();
		writeReset();
		
		//pre_loop();
		#if 0
		for (int i = 0; i < 400 ; i++)
		{
			digitalWrite(PowerLEDPin, LOW);
			delay(25);
			digitalWrite(PowerLEDPin, HIGH);
			delay(25);
			if(Serial1.find("_reb")) break;
		}
		
		digitalWrite(PowerLEDPin, LOW);
		delay(500);
		digitalWrite(PowerLEDPin, HIGH);
		delay(500);
		
		configurationMode();
		writeEcho();
		checkWANIP(0);
		writeArtNet(true);
		//writeWANN(1);
		sendCommand(CMD);
		#endif
		break;
		case 2://ETH MODE
		writeReset();
		//pre_loop();
		
		for (int i = 0; i < 400 ; i++)
		{
			digitalWrite(PowerLEDPin, LOW);
			delay(25);
			digitalWrite(PowerLEDPin, HIGH);
			delay(25);
			if(Serial1.find("_reb")) break;
		}
		digitalWrite(PowerLEDPin, LOW);
		delay(500);
		digitalWrite(PowerLEDPin, HIGH);
		delay(500);
		configurationMode();
		writeEcho();
		writeWIFI(isWifiOn);
		//checkWANIP(0);
		//writeArtNet(true);
		
		//writeWANN(1);
		sendCommand(CMD);
		
		break;
		default://AP MODE
		writeSSID();	
		writeReset();
		break;
	}
	
	digitalWrite(PowerLEDPin, LOW);
	delay(250);
	digitalWrite(PowerLEDPin, HIGH);
	delay(250);
	digitalWrite(PowerLEDPin, LOW);
	delay(250);
	digitalWrite(PowerLEDPin, HIGH);
}

void writeConfig()
{
	digitalWrite(PowerLEDPin, LOW);
	delay(250);
	digitalWrite(PowerLEDPin, HIGH);
	delay(250);
	digitalWrite(PowerLEDPin, LOW);
	delay(250);
	digitalWrite(PowerLEDPin, HIGH);
	
	//return;
	
	writeEcho();
	writeArtNet(false);
	writeDefault(true);
	writeUDPinfo();//NETP
	writeMode(true);//WMODE
	writeWANN(1);
	writeLANN(1);
	connectToSSID();
	writeSTASecurity();
	
	writeMode(false);//WMODE
	writeLANN(0);
	writeSSID();
	writeSecurity();
	
	writeFSSSID();
	writeFSKEY();
	writeFSENC();
	
	char CMD[] = "ENTM";
	sendCommand(CMD);
	
	cycleWifi(0);
	
	return;
	writePassword();
	writeNodeName();
	//writeWNodeName();
	writeUniverse();
	writeTimoPower();
	writeChannelWidth();
	writeSecondChannel();
	writeLANN(isSTA);
	writeWANN(1);
	writeFVER(1);
	//writeBitSettings();
	//writeReset();
	//return;
	writeMode(1);
	writeLang();
	writeFAPSTA(0);
	writeSSID();
	connectToSSID();
	writeSecurity();
	writeSTASecurity();
	writeEthernet();
	writeUDPinfo();
	writeDHCP();
	writeWIFI(1);
	writeReset();
}
void writeEcho()
{
	char CMD[] = "E";
	sendCommand(CMD);
}
void writeArtNet(bool enabled)
{
	if (enabled)
	{
		char CMD[] = "ARTNET=1";
		sendCommand(CMD);
	} else
	{
		char CMD[] = "ARTNET=0";
		sendCommand(CMD);
	}
}
void writeDefault(bool enabled)
{
	if (enabled)
	{
		char CMD[] = "DEFAULT=1";
		sendCommand(CMD);
	} else
	{
		char CMD[] = "DEFAULT=0";
		sendCommand(CMD);
	}
}
void writeFSSSID()
{
	char CMD[] = "FSSSID=Ratpac AKS";
	sendCommand(CMD);
}
void writeFSKEY()
{
	char CMD[] = "FSKEY=quietonset";
	sendCommand(CMD);
}
void writeFSENC()
{
	char CMD[] = "FSENC=WPA2PSK,AES";
	sendCommand(CMD);
}
void writeSSID()
{
	char CMD[] = "WAP=11BGN,Ratpac AKS,CH1";
	sendCommand(CMD);
}
void writePassword()
{
	char CMD[] = "WEBU=admin,admin";
	sendCommand(CMD);
}
void writeReset()
{
	digitalWrite(PowerLEDPin, LOW);
	delay(500);
	digitalWrite(PowerLEDPin, HIGH);
	delay(500);
	
	char CMD[] = "Z";
	sendCommand(CMD);
}
void writeSecurity()
{
	char CMD[] = "WAKEY=WPA2PSK,AES,quietonset";
	sendCommand(CMD);
}
void writeEthernet()
{
	char CMD[] = "FEPHY=on";
	sendCommand(CMD);
}
void writeTCP()
{
	writeTcpPort();
}
void writeTcpPort()
{
	char CMD[] = "TCPPTB=18899";
	sendCommand(CMD);
}
void writeUDPinfo()
{
	char CMD[] = "NETP=UDP,CLIENT,6454,10.10.100.255";
	sendCommand(CMD);
}
void writeDHCP()
{
	char CMD[] = "DHCPDEN=on";
	sendCommand(CMD);
}
void writeRELD()
{
	char CMD[] = "RELD";
	sendCommand(CMD);
}
void connectToSSID()
{
	char CMD[] = "WSSSID=Ratpac AKS";
	sendCommand(CMD);
}
void writeSTASecurity()
{
	char CMD[] = "WSKEY=WPA2PSK,AES,quietonset";
	sendCommand(CMD);
}
void writeLANN(byte eth)
{
	if(eth)
	{
		char CMD[] = "LANN=10.10.99.254,255.255.255.0";
		sendCommand(CMD);
	}else
	{
		char CMD[] = "LANN=10.10.100.254,255.255.255.0";
		sendCommand(CMD);
	}
}
void writeWANN(byte WANN)
{
	char CMD[] = "WANN=DHCP,10.10.100.100,255.255.255.0,10.10.100.254";
	sendCommand(CMD);
}
void writeFVER(byte FVER)
{
	if(FVER)
	{
		char CMD[] = "FVER=n";
		sendCommand(CMD);
	}else
	{
		char CMD[] = "FVER=z";
		sendCommand(CMD);
	}
}

void writeFVEW(byte FVEW)
{
	if(FVEW)
	{
		char CMD[] = "FVEW=enable";
		sendCommand(CMD);
	}else
	{
		char CMD[] = "FVEW=disable";
		sendCommand(CMD);
	}
}


void writeMode(byte STA)
{
	if(STA)
	{
		char CMD[] = "WMODE=STA";
		sendCommand(CMD);
	}else
	{
		char CMD[] = "WMODE=AP";
		sendCommand(CMD);
	}
}

void writeFAPSTA(byte APSTA)
{
	if(APSTA)
	{
		char CMD[] = "FAPSTA=ON";
		sendCommand(CMD);
	}else
	{
		char CMD[] = "FAPSTA=OFF";
		sendCommand(CMD);
	}
}

void writeWIFI(byte on)
{
	if(on)
	{
		char CMD[] = "MSLP=ON";
		sendCommand(CMD);
	}else
	{
		char CMD[] = "MSLP=OFF";
		sendCommand(CMD);
	}
}
void writeLang()
{
	char CMD[] = "FLANG=EN";
	sendCommand(CMD);
}
void writeNodeName()
{
	char CMD[] = "NODENAME=Ratpac AKS";
	sendCommand(CMD);
}
void writeUniverse()
{
	char CMD[] = "UNIVERSE=0";
	sendCommand(CMD);
}
void writeTimoPower()
{
	char CMD[] = "TIMOPOWER=3";
	sendCommand(CMD);
}
void writeChannelWidth()
{
	char CMD[] = "CHANNELWIDTH=16";
	sendCommand(CMD);
}
void writeSecondChannel()
{
	char CMD[] = "SECONDCHANNEL=0";
	sendCommand(CMD);
}
void writeBitSettings()
{
	char CMD[] = "BITSETTING=0";
	sendCommand(CMD);
}

//void SERCOM2_Handler()
//{
//  if (SERCOM2->USART.INTFLAG.bit.RXC)
//  {
//
//  }
//}

void FlashClass_erase(const volatile void *flash_ptr)
{
	NVMCTRL->ADDR.reg = ((uint32_t)flash_ptr) / 2;
	NVMCTRL->CTRLA.reg = NVMCTRL_CTRLA_CMDEX_KEY | NVMCTRL_CTRLA_CMD_ER;
	while (!NVMCTRL->INTFLAG.bit.READY) { }
}

#define FLASH_ROW_SIZE (FLASH_PAGE_SIZE * 4)
void FlashClass_erase(const volatile void *flash_ptr, uint32_t size)
{
	const uint8_t *ptr = (const uint8_t *)flash_ptr;
	while (size > FLASH_ROW_SIZE) {
		FlashClass_erase(ptr);
		ptr += FLASH_ROW_SIZE;
		size -= FLASH_ROW_SIZE;
	}
	FlashClass_erase(ptr);
}

#pragma GCC push_options
#pragma GCC optimize("O0")
void bootloader(void)
{
								uint32_t * ptr_reset_vector;
								uint32_t * ptr_msp;
								uint32_t app_start_add;

								__disable_irq();
								//Get reset vector from intvect table of application
								ptr_reset_vector = (uint32_t *) (0+4);
								app_start_add = (*ptr_reset_vector);
								ptr_msp = (uint32_t *) (0);
								__set_MSP(*(ptr_msp));
								
								FlashClass_erase((const volatile void *)0x4000, 128);
								
								/* Rebase the vector table base address */
								SCB->VTOR = ((uint32_t) 0 & SCB_VTOR_TBLOFF_Msk);
								
								/* Load the Reset Handler address of the application */

								//application_code_entry = (void (*)(void))(unsigned *)(*(unsigned *)(FIRMWARE_START_ADDR +4));
								
								/* Jump to user Reset Handler in the application */

								//application_code_entry();
								asm("bx %0"::"r"(app_start_add));	
}
#pragma GCC pop_options





