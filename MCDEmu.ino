/*
  MCDEmu - Mitsubishi Car CD Drive Emulator and Head Unit Emulator

  Basic Diagram:
  Master HU Volvo (1) <-----34W515 5-wire protocol-----> MCDEmu 34W515 Slave protocol Emulator (2) <-----MCDEmu protocol translator (3)-----> MCDEmu 34W539 Master protocol Emulator (4) <-----34W539 5-wire protocol-----> Slave Chrysler CD drive (5)
  
  Basic usage:
  The goal of this project is to be able to connect a Volvo P1 Chassis head unit (1) (34W515 made by Mitsubishi) with no cdmp3 support to a Chrysler CD Drive (5) (34W539 made by Mitsubishi) with mp3 support, and have a fully flawless working CD MP3 receiver.
  The software below is therefore composed of three parts:
  (2) 34W515 Slave protocol emulator : behaves as a standard 34W515 CD drive for he HU
  (3) Protocol handling : generic layer to connect the pipes
  (4) 34W539 Master protocol emulator : behaves as a standard Chrysler HU for the 34W539 CD drive
  
  2016 - johnbutol
*/

/* HW SPI
#define HARD_MOSI_PIN 11
#define HARD_MISO_PIN 12
#define HARD_SCK_PIN 13
*/

/*
 * SOFT SPI 
 * 0 - Off
 * 1 - v1 - SoftSPI done in header file
 * 2 - v2 - SoftSPI Library
*/
#define SOFTSPI_VERSION 2

#define SOFT_MOSI_PIN 16//7
#define SOFT_MISO_PIN 17//23
#define SOFT_SCK_PIN 18//14

//For debug purposes
#include <Debug.h>
#include <SoftwareSerial.h>

// include the SPI library:
#include <SPI.h>
// include the soft SPI library:
#if SOFTSPI_VERSION==1
#include "softspiv1.h"
#elif SOFTSPI_VERSION==2
#include <SoftSPI.h>
// Create a new SPI port with:
SoftSPI mySPI(SOFT_MOSI_PIN, SOFT_MISO_PIN, SOFT_SCK_PIN);
#endif

// set pin 10 as the slave select HWSPI:
const int _STM = 10;
// set pin 14 as the master select HWSPI:
const int STM = 14;
// set pin 15 as the slave select SWSPI:
const int _MTS = 15;
// set pin 19 as the master select SWSPI:
const int MTS = 19;
// set pin 20 as the mute:
const int _MUTE = 20;
// set pin 21 as the cd detect:
const int DDCNT = 21;

// set up the speed, mode and endianness of each device
SPISettings settingsHWSPI(250000, LSBFIRST, SPI_MODE3); 
#define HardSlave_CS _STM
#define HardMaster_CS STM
#if SOFTSPI_VERSION==2
// set up the speed, mode and endianness of each device
SPISettings settingsSWSPI(250000, LSBFIRST, SPI_MODE3); 
#define SoftMaster_CS _MTS
#define SoftSlave_CS MTS
#endif

/**********************************************
Common Definitions
**********************************************/
#define SLAVE_ACK 0x5A
/**********************************************
CD-P1L (34W515) Transmit Definitions
**********************************************/
#define T_34W515_TRACK_INFO_STOPPED 0x72
#define T_34W515_TRACK_INFO_LOADING 0x74
#define T_34W515_TRACK_INFO_NO_CD 0x61
#define T_34W515_TRACK_INFO_PLAYING 0x64
#define T_34W515_TRACK_INFO_PAUSED_1 0x7C
#define T_34W515_TRACK_INFO_PAUSED_2 0x6C
#define T_34W515_STATE_CHANGE_RESPONSE 0x7A
#define T_34W515_FAST_FORWARD_RESPONSE_1 0x76
#define T_34W515_FAST_FORWARD_RESPONSE_2 0x66
#define T_34W515_REWIND_RESPONSE_1 0x77
#define T_34W515_REWIND_RESPONSE_2 0x67
#define T_34W515_EJECT_RESPONSE 0x71
#define T_34W515_CD_INFO_RESPONSE(nb_tracks,total_time) {0x6E,0x01,nb_tracks,total_time,0x01,0x01}

int statusT_34W515[6]={0,0,0,0,0,0};

/**********************************************
CD-P1L (34W515) Receive Definitions
**********************************************/
#define R_34W515_UNKNOWN_1ST_MSG {0x5F,0x50,0xFE,0x3B}
#define R_34W515_EJECT 0xE1
#define R_34W515_STOP 0xE2
#define R_34W515_PLAY 0xE4
#define R_34W515_SCAN_DISABLE 0xE4
#define R_34W515_SCAN_ENABLE 0xE5
#define R_34W515_FAST_FORWARD 0xE6
#define R_34W515_REWIND 0xE7
#define R_34W515_RANDOM_ENABLE 0xEA
#define R_34W515_PAUSE 0xEC
#define R_34W515_GOTO_TRACK(track_number) {0xF4,track_number}
#define R_34W515_ERROR_INFO 0xF7
#define R_34W515_TRACK_INFO 0xF8
#define R_34W515_RANDOM_DISABLE 0xFA
#define R_34W515_CD_INFO 0xFC

int statusR_34W515[6]={0,0,0,0,0,0};

const int StartMsgT_34W515[]=R_34W515_UNKNOWN_1ST_MSG;
const int CdInfoMsgT_34W515=R_34W515_CD_INFO;

/**********************************************
Chrysler CD drive (34W539) Transmit Definitions
**********************************************/
#define T_34W539 0xF8
#define T_34W539_CD_INFO_RESPONSE {0xF2,0x10,0x00,0x05,0x01}
#define T_34W539_TRACK_INFO_RESPONSE {T_34W539,0x25,0x09}
#define T_34W539_1ST_MESSAGE {T_34W539,0x82}
#define T_34W539_2ND_MESSAGE {T_34W539,0x04}
#define T_34W539_3RD_MESSAGE {T_34W539,0x85}
#define T_34W539_4TH_MESSAGE_STD_CD {T_34W539,0xA5,0x09}
#define T_34W539_4TH_MESSAGE_MP3_CD {T_34W539,0x26,0x11}
#define T_34W539_DIRECTORY_SET_CMD_RESPONSE {T_34W539,0xAB}
#define T_34W539_EJECT_CMD_RESPONSE_1 {T_34W539,0xA1}
#define T_34W539_EJECT_CMD_RESPONSE_2 {T_34W539,0x21}
#define T_34W539_EJECT_CMD_RESPONSE_3 {T_34W539,0x01}
#define T_34W539_STOP_CMD_RESPONSE_1 {T_34W539,0xA4}
#define T_34W539_STOP_CMD_RESPONSE_2 {T_34W539,0x24}
#define T_34W539_ERROR_RESPONSE {0xF9,0x84}
#define T_34W539_METADATA_RESPONSE 0xFA
#define T_34W539_OTHER_INFO_RESPONSE 0xFB

int statusT_34W539[10]={0,0,0,0,0,0,0,0,0,0};

/**********************************************
Chrysler CD drive (34W539) Receive Definitions
**********************************************/
#define R_34W539_EJECT 0xE1
#define R_34W539_STOP 0xE2
#define R_34W539_PLAY 0xE4
#define R_34W539_FAST_FORWARD {0xE6,0x02}
#define R_34W539_REWIND {0xE7,0x02}
#define R_34W539_NEXT_TRACK 0xEA
#define R_34W539_CURRENT_BACK_TRACK {0xEB,0x01}
#define R_34W539_PREVIOUS_TRACK {0xEB,0x02}
#define R_34W539_ERROR_STOP 0xF7
#define R_34W539_RANDOM_DISABLE 0xF0
#define R_34W539_RANDOM_ENABLE 0xFA
#define R_34W539_CD_INFO {0xFC,0x10,0x00}
#define R_34W539_DIRECTORY_SET_PREVIOUS 0xC5
#define R_34W539_DIRECTORY_SET_NEXT 0xC6
#define R_34W539_METADATA_DIRECTORY_NAME(x) {0xC7,0x01,0x02,0x02,0x00,0x00,x,0x01}
#define R_34W539_METADATA_TRACK_NAME(x,y) {0xC7,0x01,0x02,0x05,0x00,0x00,x,y}
#define R_34W539_METADATA_ARTIST_NAME(x,y) {0xC7,0x01,0x02,0x04,0x00,0x00,x,y}
#define R_34W539_METADATA_FILE_NAME(x,y) {0xC7,0x01,0x02,0x01,0x00,0x00,x,y}
#define R_34W539_OTHER_INFO {0xC8,0x10,0x00}
#define R_34W539_UNKNOWN_1 {0xC9,0x01}

int statusR_34W539[10]={0,0,0,0,0,0,0,0,0,0};

const int StartMsgT_34W539[]=R_34W515_UNKNOWN_1ST_MSG;
const int CdInfoMsgT_34W539=R_34W515_CD_INFO;

/**********************************************
SETUP
**********************************************/
void setup() {
  // set the HardSlave_CS as an output:
  pinMode(HardSlave_CS, OUTPUT);
  // set the SoftMaster_CS as an output:
  pinMode(SoftMaster_CS, OUTPUT);
  // set the HardMaster_CS as an input:
  pinMode(HardMaster_CS, INPUT);
  // set the SoftSlave_CS as an input:
  pinMode(SoftSlave_CS, INPUT);
  // set the DDCNT as an output:
  pinMode(DDCNT, OUTPUT);
   // set the _MUTE as an output:
  pinMode(_MUTE, OUTPUT);
  // initialize SPI:
  SPI.begin();

#if SOFTSPI_VERSION==2
  // initialize Software SPI v2:
  mySPI.begin();
#endif

  //for debug purposes
  Serial.begin(9600);

  digitalWrite(SoftMaster_CS, HIGH); 
  digitalWrite(SOFT_MOSI_PIN, HIGH);
}

/**********************************************
LOOP
**********************************************/
void loop()
{
//  MCDEmu_Slave_34W515();
  MCDEmu_Generic_Commands();
//  MCDEmu_Master_34W539();
}

/**********************************************
 * For debug purposes
**********************************************/
typedef struct
{
  bool initReq;
  bool playReq;
  bool pauseReq;
  bool nextReq;
  bool prevReq;
  bool ejectReq;
  bool cdinfoReq;
  bool trackinfoReq;
}commandsRequest_s;

commandsRequest_s commandsRequest={false,false,false,false,false,false,false};

void serialEvent()
{
    do
    {
      switch(Serial.read())
      {
        case 'i':
        {
          commandsRequest.initReq = true;
          Serial.println("INIT");
          break;
        }
        case 'p':
        {
          commandsRequest.pauseReq = true;
          Serial.println("PAUSE");
          break;
        }
        case 'P':
        {
          commandsRequest.playReq = true;
          Serial.println("PLAY");
          break;
        }
        case 'n':
        {
          commandsRequest.nextReq = true;
          Serial.println("NEXT");
          break;
        }
        case 'c':
        {
          commandsRequest.cdinfoReq = true;
          Serial.println("CD STATUS");
          break;
        }
        case 't':
        {
          commandsRequest.trackinfoReq = true;
          Serial.println("TRACK STATUS");
          break;
        }
        case 'e':
        {
          commandsRequest.ejectReq = true;
          Serial.println("EJECT");
          break;
        }
        default:
        {
          Serial.println("NOT A COMMAND");
          break;
        }
      }
    }
    while(Serial.read() != -1); 
}


/**********************************************
(2) 34W515 Slave protocol emulator
**********************************************/

bool onReceiveStartMsg_34W515(void)
{
  bool result=true;
  bool OnGoingTransmission = false;
  int receivedValue[4] = {0,0,0,0};
  
  if(!digitalRead(HardSlave_CS))
  {
    OnGoingTransmission = true;
  }

  if(0)
  {
    for (int channel = 0; channel < sizeof(receivedValue)/sizeof(int); channel++)
    {
      receivedValue[channel] = digitalHWSPIWrite(SLAVE_ACK);
      while(digitalRead(HardSlave_CS));
    }
  }
  return(result);
}

void MCDEmu_Slave_34W515(void)
{
  static bool initialized = false;
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  long interval = 500;

  if(initialized == false)
  {
    if(onReceiveStartMsg_34W515())
    {
      initialized = true;
    }
  }
  else if(currentMillis - previousMillis > interval)
  {
    previousMillis = currentMillis;
    
    for (int channel = 0; channel < sizeof(StartMsgT_34W515)/sizeof(int); channel++)
    {
        digitalHWSPIWrite(StartMsgT_34W515[channel]);
        delayMicroseconds(850);
    }
  }
    // wait at the top:
    //delay(500);
}

/**********************************************
(3) Generic Layer: connecting the dots
**********************************************/
void MCDEmu_Generic_Commands(void)
{
  int returnValue=0;
  
  if(commandsRequest.initReq)
  {
    commandsRequest.initReq=false;
    returnValue = send_34W539(0x55);
  }
  if(commandsRequest.playReq)
  {
    commandsRequest.playReq=false;
    returnValue = send_34W539(R_34W539_PLAY);
  }
  if(commandsRequest.pauseReq)
  {
    commandsRequest.pauseReq=false;
    returnValue = send_34W539(R_34W539_STOP);
  }
  if(commandsRequest.nextReq)
  {
    commandsRequest.nextReq=false;
    returnValue = send_34W539(R_34W539_NEXT_TRACK);
  }
  if(commandsRequest.ejectReq)
  {
    commandsRequest.ejectReq=false;
    returnValue = send_34W539(R_34W539_EJECT);
  }
  if(commandsRequest.cdinfoReq)
  {
    commandsRequest.cdinfoReq=false;
    returnValue = send_34W539(0xAA);
  }
  if(commandsRequest.trackinfoReq)
  {
    commandsRequest.trackinfoReq=false;
    returnValue = send_34W539(R_34W539_EJECT);
  }
}

/**********************************************
(4) 34W539 Master protocol emulator
**********************************************/
void MCDEmu_Master_34W539(void)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  long interval = 500;
  
  if(currentMillis - previousMillis > interval)
  {
    previousMillis = currentMillis;
    
    for (int channel = 0; channel < sizeof(StartMsgT_34W539)/sizeof(int); channel++)
    {
  #if SOFTSPI_VERSION!=0
        digitalSWSPIWrite(StartMsgT_34W539[channel]);
  #endif
        delayMicroseconds(850);
    }
  }
}

/**********************************************
Software SPI Transfer
**********************************************/
#if SOFTSPI_VERSION==1
int digitalSWSPIWrite(int value)
{
  int returnValue;
  
  // take the SS pin low to select the chip:
  digitalWrite(SoftMaster_CS, LOW);
  spi_send(value);
  // take the SS pin high to de-select the chip:
  digitalWrite(SoftMaster_CS, HIGH);
  
  return returnValue;
}
#elif SOFTSPI_VERSION==2
int digitalSWSPIWrite(int value)
{
  int returnValue=0;
  
  mySPI.beginTransaction(settingsSWSPI);
  // take the SS pin low to select the chip:
  digitalWrite(SoftMaster_CS, LOW);
  mySPI.transfer(value);

  // force soft SPI to idle high
  digitalWrite(SOFT_SCK_PIN, HIGH);
  digitalWrite(SOFT_MOSI_PIN, HIGH);
  
  // take the SS pin high to de-select the chip:
  digitalWrite(SoftMaster_CS, HIGH);
  mySPI.endTransaction();
  returnValue =1;
  
  return returnValue;
}
#endif

int send_34W539(int value)
{  
  int returnValue=0;
  // take the SS pin low to select the chip:
  digitalWrite(SoftMaster_CS, LOW);

  while(returnValue!=8)
  {
    while(digitalRead(SoftSlave_CS));

    if(((value & (1<<returnValue))>>returnValue) == 0)
    {
        digitalWrite(SOFT_MOSI_PIN, LOW);
    }
    else
    {
        digitalWrite(SOFT_MOSI_PIN, HIGH);
    }
    while(!digitalRead(SoftSlave_CS));

    returnValue++;
  }
  returnValue = 1;
  
  delayMicroseconds(10);
  
  // take the SS pin high to de-select the chip:
  digitalWrite(SoftMaster_CS, HIGH);
  digitalWrite(SOFT_MOSI_PIN, HIGH);
  
  return returnValue;
}


/**********************************************
Harwdare SPI Transfer
**********************************************/
int digitalHWSPIWrite(int value)
{
  int returnValue;
  
  SPI.beginTransaction(settingsHWSPI);
  // take the SS pin low to select the chip:
  digitalWrite(HardSlave_CS, LOW);
//  delayMicroseconds(45);
  returnValue = SPI.transfer(value);
//  delayMicroseconds(60);
  // take the SS pin high to de-select the chip:
  digitalWrite(HardSlave_CS, HIGH);
  SPI.endTransaction();
  
  return returnValue;
}

