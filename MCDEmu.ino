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

//For debug purposes
//#include <Debug.h>
#include <SoftwareSerial.h>

// include the SPI library:
#include <SPI.h>

//SPI 34W539
#define _STM_34W539_CS_PIN 15
#define DSTM_34W539_MISO_PIN 16
#define DMTS_34W539_MOSI_PIN 17
#define _SCK_34W539_CLK_PIN 18
#define _MTS_34W539_CS_PIN 19

//SPI 34W515
#define _STM_34W515_CS_PIN 10
#define DSTM_34W515_MOSI_PIN 11
#define DMTS_34W515_MISO_PIN 12
#define _SCK_34W515_CLK_PIN 13
#define _MTS_34W515_CS_PIN 14

//OTHER SIGNALS
// set pin 20 as the mute:
const int _MUTE = 20;
// set pin 21 as the cd detect:
const int DDCNT = 21;

// set up the speed, mode and endianness of each device
SPISettings settingsHWSPI(250000, LSBFIRST, SPI_MODE3); 

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
#define T_34W539_UNKNOWN_1ST_MSG {0xF5,0x01,0x09,0xA0,0x80,0x18}
#define T_34W539_UNKNOWN_2ND_MSG {0xF4,0x00,0x32,0x05,0x05,0x09,0x01,0x0D}
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
#define R_34W539_UNKNOWN_1ST_MSG {0x5F,0x01,0x09,0xA0,0x80,0x18}
#define R_34W539_UNKNOWN_2ND_MSG {0xAD}
#define R_34W539_ACK 0xDB
#define R_34W539_EJECT {0xE1}
#define R_34W539_STOP {0xE2}
#define R_34W539_PLAY {0xE4}
#define R_34W539_FAST_FORWARD {0xE6,0x02}
#define R_34W539_REWIND {0xE7,0x02}
#define R_34W539_NEXT_TRACK {0xEA}
#define R_34W539_CURRENT_BACK_TRACK {0xEB,0x01}
#define R_34W539_PREVIOUS_TRACK {0xEB,0x02}
#define R_34W539_ERROR_STOP {0xF7}
#define R_34W539_RANDOM_DISABLE {0xF0}
#define R_34W539_RANDOM_ENABLE {0xFA}
#define R_34W539_CD_INFO {0xFC,0x10,0x00}
#define R_34W539_DIRECTORY_SET(dirN) {0xC3,0x00,0x00,dirN,0x01}
#define R_34W539_DIRECTORY_SET_PREVIOUS {0xC5}
#define R_34W539_DIRECTORY_SET_NEXT {0xC6}
#define R_34W539_METADATA 0xC7
#define R_34W539_METADATA_DIRECTORY_NAME(dirN) {R_34W539_METADATA,0x01,0x02,0x02,0x00,0x00,dirN,0x01}
#define R_34W539_METADATA_TRACK_NAME(dirN,trkN) {R_34W539_METADATA,0x01,0x02,0x05,0x00,0x00,dirN,trkN}
#define R_34W539_METADATA_ARTIST_NAME(dirN,trkN) {R_34W539_METADATA,0x01,0x02,0x04,0x00,0x00,dirN,trkN}
#define R_34W539_METADATA_FILE_NAME(dirN,trkN) {R_34W539_METADATA,0x01,0x02,0x01,0x00,0x00,dirN,trkN}
#define R_34W539_OTHER_INFO_1 {0xC8,0x10,0x00}
#define R_34W539_OTHER_INFO_2 {0xC8,0x20,0x01}
#define R_34W539_UNKNOWN_1 {0xC9,0x01}
#define R_34W539_DIRECTORY_ROOT 0x01
#define R_34W539_DIRECTORY_1 0x03

int statusR_34W539[10]={0,0,0,0,0,0,0,0,0,0};

int gCurrentTrack = 0;
int gCurrentDirecory = 0;

const unsigned char initT_34W539[]=R_34W539_UNKNOWN_1ST_MSG;
const unsigned char infoDiscT_34W539[]=R_34W539_CD_INFO;
const unsigned char previousTrackT_34W539[]=R_34W539_PREVIOUS_TRACK;
const unsigned char metaDirNameT_34W539[]=R_34W539_METADATA_DIRECTORY_NAME(R_34W539_DIRECTORY_1);
const unsigned char metaTrackNameT_34W539[]=R_34W539_METADATA_TRACK_NAME(R_34W539_DIRECTORY_1,1);
const unsigned char metaArtNameT_34W539[]=R_34W539_METADATA_ARTIST_NAME(R_34W539_DIRECTORY_1,1);
const unsigned char otherInfo1T_34W539[]=R_34W539_OTHER_INFO_1;
const unsigned char otherInfo2T_34W539[]=R_34W539_OTHER_INFO_2;
const unsigned char playTrackT_34W539[]=R_34W539_PLAY;
const unsigned char pauseTrackT_34W539[]=R_34W539_STOP;
const unsigned char nextTrackT_34W539[]=R_34W539_NEXT_TRACK;
const unsigned char nextDirectoryT_34W539[]=R_34W539_DIRECTORY_SET_NEXT;
const unsigned char previousDirectoryT_34W539[]=R_34W539_DIRECTORY_SET_PREVIOUS;
const unsigned char ejectDiscT_34W539[]=R_34W539_EJECT;
const unsigned char infoTrackT_34W539[]=R_34W539_OTHER_INFO_2;
const unsigned char randomEnableT_34W539[]=R_34W539_RANDOM_ENABLE;
const unsigned char randomDisableT_34W539[]=R_34W539_RANDOM_DISABLE;
const unsigned char fastForwardT_34W539[]=R_34W539_FAST_FORWARD;
const unsigned char metaFileNameT_34W539[]=R_34W539_METADATA_FILE_NAME(R_34W539_DIRECTORY_1,1);
const unsigned char rewindT_34W539[]=R_34W539_REWIND;

const unsigned char sizeofinitT_34W539 = sizeof(initT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofinfoDiscT_34W539 = sizeof(infoDiscT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofpreviousTrackT_34W539 = sizeof(previousTrackT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofmetaDirNameT_34W539 = sizeof(metaDirNameT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofinfoTrackT_34W539 = sizeof(infoTrackT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofejectDiscT_34W539 = sizeof(ejectDiscT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofrandomEnableT_34W539 = sizeof(randomEnableT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofrandomDisableT_34W539 = sizeof(randomDisableT_34W539)/sizeof(const unsigned char);
const unsigned char sizeoffastForwardT_34W539 = sizeof(fastForwardT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofrewindT_34W539 = sizeof(rewindT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofpreviousDirectoryT_34W539 = sizeof(previousDirectoryT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofnextDirectoryT_34W539 = sizeof(nextDirectoryT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofnextTrackT_34W539 = sizeof(nextTrackT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofpauseTrackT_34W539 = sizeof(pauseTrackT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofmetaFileNameT_34W539 = sizeof(metaFileNameT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofplayTrackT_34W539 = sizeof(playTrackT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofotherInfo2T_34W539 = sizeof(otherInfo2T_34W539)/sizeof(const unsigned char);
const unsigned char sizeofmetaArtNameT_34W539 = sizeof(metaArtNameT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofmetaTrackNameT_34W539 = sizeof(metaTrackNameT_34W539)/sizeof(const unsigned char);
const unsigned char sizeofotherInfo1T_34W539 = sizeof(otherInfo1T_34W539)/sizeof(const unsigned char);

/**********************************************
SETUP
**********************************************/
void setup()
{
  //34W539
  pinMode(_MTS_34W539_CS_PIN, OUTPUT);
  pinMode(DMTS_34W539_MOSI_PIN, OUTPUT);
  pinMode(_STM_34W539_CS_PIN, INPUT);
  pinMode(DSTM_34W539_MISO_PIN, INPUT);
  pinMode(_SCK_34W539_CLK_PIN, INPUT);

  //34W515
  pinMode(_STM_34W515_CS_PIN, OUTPUT);
  pinMode(DSTM_34W515_MOSI_PIN, OUTPUT);
  pinMode(_SCK_34W515_CLK_PIN, OUTPUT);
  pinMode(_MTS_34W515_CS_PIN, INPUT);
  pinMode(DMTS_34W515_MISO_PIN, INPUT);

  // set the DDCNT as an output:
  pinMode(DDCNT, OUTPUT);
   // set the _MUTE as an output:
  pinMode(_MUTE, OUTPUT);
  // initialize SPI:
  SPI.begin();

  //for debug purposes
  Serial.begin(9600);

  digitalWriteFast(_MTS_34W539_CS_PIN, HIGH); 
  digitalWriteFast(DMTS_34W539_MOSI_PIN, HIGH);
}

/**********************************************
LOOP
**********************************************/
void loop()
{
//  MCDEmu_Slave_34W515();
//  MCDEmu_Generic_Commands();
  MCDEmu_Master_34W539();
}

/**********************************************
 * For debug purposes
**********************************************/
typedef struct
{
  bool printHelp;
  bool initReq;
  bool playTrackReq;
  bool pauseTrackReq;
  bool nextTrackReq;
  bool previousTrackReq;
  bool nextDirectoryReq;
  bool previousDirectoryReq;
  bool ejectDiscReq;
  bool infoDiscReq;
  bool infoTrackReq;
  bool randomEnableReq;
  bool randomDisableReq;
  bool fastForwardReq;
  bool rewindReq;
  bool metaDirNameReq;
  bool metaArtNameReq;
  bool metaTrackNameReq;
  bool metaFileNameReq;
  bool otherInfoReq;
}cmdRequest_s;

cmdRequest_s cmdRequest = {false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false,false};
char responseSize = 0;

typedef struct
{
  char serialCmd;
  bool *cmdRequest;
  const char *printSerialMsg;
}cmdtabRequest_s;

const cmdtabRequest_s tabSerialRequest[]=
{
  {'h',  &cmdRequest.printHelp,            "PRINTS HELP"},
  {'a',  &cmdRequest.initReq ,             "INIT"},
  {'e',  &cmdRequest.ejectDiscReq ,        "EJECT"},
  {'p',  &cmdRequest.pauseTrackReq ,       "PAUSE"},
  {'P',  &cmdRequest.playTrackReq ,        "PLAY"},
  {'N',  &cmdRequest.nextTrackReq ,        "NEXT TRACK"},
  {'n',  &cmdRequest.previousTrackReq ,    "PREVIOUS TRACK"},
  {'F',  &cmdRequest.fastForwardReq ,      "FAST FORWARD"},
  {'f',  &cmdRequest.rewindReq ,           "REWIND"},
  {'I',  &cmdRequest.infoDiscReq ,         "DISC INFO"},
  {'i',  &cmdRequest.infoTrackReq ,        "TRACK INFO"},
  {'R',  &cmdRequest.randomEnableReq ,     "RANDOM ON"},
  {'r',  &cmdRequest.randomDisableReq ,    "RANDOM OFF"},
  {'w',  &cmdRequest.metaDirNameReq ,      "DIRECTORY NAME"},
  {'x',  &cmdRequest.metaArtNameReq ,      "ARTIST NAME"},
  {'c',  &cmdRequest.metaTrackNameReq ,    "TRACK NAME"},
  {'v',  &cmdRequest.metaFileNameReq ,     "FILE NAME"},
  {'b',  &cmdRequest.otherInfoReq ,        "OTHER INFO"},
  {'D',  &cmdRequest.nextDirectoryReq ,    "NEXT DIRECTORY"},
  {'d',  &cmdRequest.previousDirectoryReq ,"PREVIOUS DIRECTORY"},
  //always last
  {NULL,  NULL ,                           "NOT FOUND"}
};

const int sizeof_tabcmdRequest = sizeof(tabSerialRequest)/sizeof(cmdtabRequest_s);

void printHelp(void)
{
   const cmdtabRequest_s *lcmdRequest;
   lcmdRequest = tabSerialRequest;
     
   while(lcmdRequest->serialCmd!=NULL)
   {
    Serial.print(lcmdRequest->serialCmd);
    Serial.print(" - ");
    Serial.println(lcmdRequest->printSerialMsg);
    lcmdRequest++;
   }
}

void serialEvent()
{
  int i;
  char receivedChar;
  const cmdtabRequest_s *lcmdRequest;

  receivedChar = Serial.read();
  lcmdRequest = tabSerialRequest;
  
  for(i=0;i<sizeof_tabcmdRequest;i++)
  {
    if(lcmdRequest->serialCmd == receivedChar
    || lcmdRequest->serialCmd == NULL)
    {
        Serial.println(lcmdRequest->printSerialMsg);
        if(lcmdRequest->cmdRequest != NULL)
        {
          *lcmdRequest->cmdRequest = true;
          break;
        }
    }
    lcmdRequest++;
  }
}

/**********************************************
(2) 34W515 Slave protocol emulator
**********************************************/

bool onReceiveStartMsg_34W515(void)
{
  bool result=true;
  bool OnGoingTransmission = false;
  int receivedValue[4] = {0,0,0,0};
  
  if(0)
  {
    OnGoingTransmission = true;
  }

  if(0)
  {
    for (int channel = 0; channel < sizeof(receivedValue)/sizeof(int); channel++)
    {
      receivedValue[channel] = digitalHWSPIWrite(SLAVE_ACK);
      while(0);
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

}

/**********************************************
(4) 34W539 Master protocol emulator
**********************************************/

typedef struct
{
  bool *request;
  const unsigned char *mesg;
  unsigned char transmitSize;
  unsigned char responseSize;
}transmitMsg_s;

const transmitMsg_s transmitMsg[]=
{
 //COMMAND LIST                     //MESSAGE TO SEND             //SIZE TO SEND                  //SIZE TO RECEIVE
  {&cmdRequest.initReq ,             initT_34W539,               sizeofinitT_34W539,               6},
  {&cmdRequest.ejectDiscReq ,        ejectDiscT_34W539,          sizeofejectDiscT_34W539,          9},
  {&cmdRequest.pauseTrackReq ,       pauseTrackT_34W539,         sizeofpauseTrackT_34W539,         9},
  {&cmdRequest.playTrackReq ,        playTrackT_34W539,          sizeofplayTrackT_34W539,          9},
  {&cmdRequest.nextTrackReq ,        nextTrackT_34W539,          sizeofnextTrackT_34W539,          9},
  {&cmdRequest.previousTrackReq ,    previousTrackT_34W539,      sizeofpreviousTrackT_34W539,      9},
  {&cmdRequest.fastForwardReq ,      fastForwardT_34W539,        sizeoffastForwardT_34W539,        9},
  {&cmdRequest.rewindReq ,           rewindT_34W539,             sizeofrewindT_34W539,             9},  
  {&cmdRequest.infoDiscReq ,         infoDiscT_34W539,           sizeofinfoDiscT_34W539,           9},
  {&cmdRequest.infoTrackReq ,        infoTrackT_34W539,          sizeofinfoTrackT_34W539,          9},
  {&cmdRequest.randomEnableReq ,     randomEnableT_34W539,       sizeofrandomEnableT_34W539,       9},
  {&cmdRequest.randomDisableReq ,    randomDisableT_34W539,      sizeofrandomDisableT_34W539,      9},
  {&cmdRequest.metaDirNameReq ,      metaDirNameT_34W539,        sizeofmetaDirNameT_34W539,        25},
  {&cmdRequest.metaArtNameReq ,      metaArtNameT_34W539,        sizeofmetaArtNameT_34W539,        25},
  {&cmdRequest.metaTrackNameReq ,    metaTrackNameT_34W539,      sizeofmetaTrackNameT_34W539,      25},
  {&cmdRequest.metaFileNameReq ,     metaFileNameT_34W539,       sizeofmetaFileNameT_34W539,       25},
  {&cmdRequest.otherInfoReq ,        otherInfo1T_34W539,         sizeofotherInfo1T_34W539,         9},
  {&cmdRequest.nextDirectoryReq ,    nextDirectoryT_34W539,      sizeofnextDirectoryT_34W539,      9},
  {&cmdRequest.previousDirectoryReq, previousDirectoryT_34W539,  sizeofpreviousDirectoryT_34W539,  9},
  //always last
  {NULL,NULL,0}
};

const unsigned char sizeof_transmitMsg = sizeof(transmitMsg)/sizeof(transmitMsg_s);

void MCDEmu_Master_34W539(void)
{
  unsigned char returnValue=0;
  unsigned char i;
  unsigned char byteIndex;
  unsigned char tabreturnValue[25]={"\0"};
  const transmitMsg_s * ltransmitMsg;

  ltransmitMsg = transmitMsg;
  
  if(cmdRequest.printHelp)
  {
    cmdRequest.printHelp=false;
    printHelp();
  }
  else
  {
    for(i=0;i<sizeof_transmitMsg;i++)
    {
      if(ltransmitMsg->request != NULL)
      {
        if(*ltransmitMsg->request == true)
        {
          *ltransmitMsg->request = false;
          for(byteIndex=0;byteIndex<ltransmitMsg->transmitSize;byteIndex++)
          {
            returnValue = digitalSWSPITransfer(ltransmitMsg->mesg[byteIndex]);
            if(byteIndex != (ltransmitMsg->transmitSize-1))
            {
              delay(2);
            }
          }
        }
      }
      ltransmitMsg++;
    }
  }

  if(!digitalReadFast(_STM_34W539_CS_PIN) && responseSize == 0)
  {
 //   responseSize = 6;
  }
  i=responseSize;
  while(responseSize!=0)
  {
    responseSize--;
    returnValue = digitalSWSPITransfer(R_34W539_ACK);
    if(returnValue < ' ' || returnValue > '}')
    {
      Serial.print(returnValue);
      Serial.print(" ");
    }
    else
    {
      tabreturnValue[i-responseSize] = returnValue;
      if(responseSize==0)Serial.print(*tabreturnValue);
    }
    if(responseSize==0)Serial.print("\n");
  }
}

/**********************************************
Software SPI Transfer
**********************************************/
unsigned char digitalSWSPITransfer(unsigned char valuetosend)
{
  unsigned char bitPosition;
  unsigned char valuetoreceive = 0;

  if(valuetosend != R_34W539_ACK)
  {
    while(!digitalReadFast(_STM_34W539_CS_PIN));
  }
  else
  {
    while(digitalReadFast(_STM_34W539_CS_PIN));
    delay(4);
  }

  // take the SS pin low to select the chip:
  digitalWriteFast(_MTS_34W539_CS_PIN, LOW);

  for(bitPosition=0;bitPosition<8;bitPosition++)
  {
    while(digitalReadFast(_SCK_34W539_CLK_PIN));

    //READ
    valuetoreceive |=digitalReadFast(DSTM_34W539_MISO_PIN)<<bitPosition;

    //WRITE
    if((valuetosend & (1<<bitPosition)) == 0)
    {
        digitalWriteFast(DMTS_34W539_MOSI_PIN, LOW);
    }
    else
    {
        digitalWriteFast(DMTS_34W539_MOSI_PIN, HIGH);
    }

    while(!digitalReadFast(_SCK_34W539_CLK_PIN));
  }

  delayMicroseconds(10);
  digitalWriteFast(DMTS_34W539_MOSI_PIN, HIGH);
  delayMicroseconds(890);
  
  // take the SS pin high to de-select the chip:
  digitalWriteFast(_MTS_34W539_CS_PIN, HIGH);

  return valuetoreceive;
}

/**********************************************
Harwdare SPI Transfer
**********************************************/
int digitalHWSPIWrite(int value)
{
  int returnValue;
  
  SPI.beginTransaction(settingsHWSPI);
  // take the SS pin low to select the chip:
  digitalWrite(_STM_34W515_CS_PIN, LOW);
//  delayMicroseconds(45);
  returnValue = SPI.transfer(value);
//  delayMicroseconds(60);
  // take the SS pin high to de-select the chip:
  digitalWrite(_STM_34W515_CS_PIN, HIGH);
  SPI.endTransaction();
  
  return returnValue;
}

