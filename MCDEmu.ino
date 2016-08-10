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
//#define NDEBUG
#include <SoftwareSerial.h>

// include the SPI library:
#include <SPI.h>

//SPI 34W539
#define _STM_34W539_CS_PIN 15 //A1
#define DSTM_34W539_MISO_PIN 16 //A2
#define DMTS_34W539_MOSI_PIN 17 //A3
#define _SCK_34W539_CLK_PIN 18 //A4
#define _MTS_34W539_CS_PIN 19 //A5

//SPI 34W515
#define _STM_34W515_CS_PIN SS //10
#define DSTM_34W515_MOSI_PIN MOSI //11
#define DMTS_34W515_MISO_PIN MISO //12
#define _SCK_34W515_CLK_PIN SCK //13
#define _MTS_34W515_CS_PIN 14 //A0

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
#define MASTER_ACK 0xDB
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
#define T_34W539_STATUS 0xF8
#define T_34W539_METADATA 0xFA
#define T_34W539_UNKNOWN_1ST_MSG {0xF5,0x01,0x09,0xA0,0x80,0x18}
#define T_34W539_UNKNOWN_2ND_MSG {0xF4,0x00,0x32,0x05,0x05,0x09,0x01,0x0D}
#define T_34W539_CD_INFO_RESPONSE {0xF2,0x10,0x00,0x05,0x01}
#define T_34W539_TRACK_INFO_RESPONSE {T_34W539_STATUS,0x25,0x09}
#define T_34W539_1ST_MESSAGE {T_34W539_STATUS,0x82}
#define T_34W539_2ND_MESSAGE {T_34W539_STATUS,0x04}
#define T_34W539_3RD_MESSAGE {T_34W539_STATUS,0x85}
#define T_34W539_4TH_MESSAGE_STD_CD {T_34W539_STATUS,0xA5,0x09}
#define T_34W539_4TH_MESSAGE_MP3_CD {T_34W539_STATUS,0x26,0x11}
#define T_34W539_DIRECTORY_SET_CMD_RESPONSE {T_34W539_STATUS,0xAB}
#define T_34W539_EJECT_CMD_RESPONSE_1 {T_34W539_STATUS,0xA1}
#define T_34W539_EJECT_CMD_RESPONSE_2 {T_34W539_STATUS,0x21}
#define T_34W539_EJECT_CMD_RESPONSE_3 {T_34W539_STATUS,0x01}
#define T_34W539_STOP_CMD_RESPONSE_1 {T_34W539_STATUS,0xA4}
#define T_34W539_STOP_CMD_RESPONSE_2 {T_34W539_STATUS,0x24}
#define T_34W539_ERROR_RESPONSE {0xF9,0x84}
#define T_34W539_OTHER_INFO_RESPONSE 0xFB

int statusT_34W539[10]={0,0,0,0,0,0,0,0,0,0};

uint8_t initR_34W539[]=T_34W539_UNKNOWN_1ST_MSG;

/**********************************************
Chrysler CD drive (34W539) Receive Definitions
**********************************************/
#define R_34W539_ACK MASTER_ACK

#define R_34W539_UNKNOWN_1ST_MSG {0x5F,0x01,0x09,0xA0,0x80,0x18}
#define R_34W539_UNKNOWN_2ND_MSG {0xAD}
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

const uint8_t initT_34W539[]=R_34W539_UNKNOWN_1ST_MSG;
const uint8_t infoDiscT_34W539[]=R_34W539_CD_INFO;
const uint8_t previousTrackT_34W539[]=R_34W539_PREVIOUS_TRACK;
const uint8_t metaDirNameT_34W539[]=R_34W539_METADATA_DIRECTORY_NAME(R_34W539_DIRECTORY_1);
const uint8_t metaTrackNameT_34W539[]=R_34W539_METADATA_TRACK_NAME(R_34W539_DIRECTORY_1,1);
const uint8_t metaArtNameT_34W539[]=R_34W539_METADATA_ARTIST_NAME(R_34W539_DIRECTORY_1,1);
const uint8_t otherInfo1T_34W539[]=R_34W539_OTHER_INFO_1;
const uint8_t otherInfo2T_34W539[]=R_34W539_OTHER_INFO_2;
const uint8_t playTrackT_34W539[]=R_34W539_PLAY;
const uint8_t pauseTrackT_34W539[]=R_34W539_STOP;
const uint8_t nextTrackT_34W539[]=R_34W539_NEXT_TRACK;
const uint8_t nextDirectoryT_34W539[]=R_34W539_DIRECTORY_SET_NEXT;
const uint8_t previousDirectoryT_34W539[]=R_34W539_DIRECTORY_SET_PREVIOUS;
const uint8_t ejectDiscT_34W539[]=R_34W539_EJECT;
const uint8_t infoTrackT_34W539[]=R_34W539_OTHER_INFO_2;
const uint8_t randomEnableT_34W539[]=R_34W539_RANDOM_ENABLE;
const uint8_t randomDisableT_34W539[]=R_34W539_RANDOM_DISABLE;
const uint8_t fastForwardT_34W539[]=R_34W539_FAST_FORWARD;
const uint8_t metaFileNameT_34W539[]=R_34W539_METADATA_FILE_NAME(R_34W539_DIRECTORY_1,1);
const uint8_t rewindT_34W539[]=R_34W539_REWIND;

const uint8_t sizeofinitT_34W539 = sizeof(initT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofinfoDiscT_34W539 = sizeof(infoDiscT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofpreviousTrackT_34W539 = sizeof(previousTrackT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofmetaDirNameT_34W539 = sizeof(metaDirNameT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofinfoTrackT_34W539 = sizeof(infoTrackT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofejectDiscT_34W539 = sizeof(ejectDiscT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofrandomEnableT_34W539 = sizeof(randomEnableT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofrandomDisableT_34W539 = sizeof(randomDisableT_34W539)/sizeof(const uint8_t);
const uint8_t sizeoffastForwardT_34W539 = sizeof(fastForwardT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofrewindT_34W539 = sizeof(rewindT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofpreviousDirectoryT_34W539 = sizeof(previousDirectoryT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofnextDirectoryT_34W539 = sizeof(nextDirectoryT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofnextTrackT_34W539 = sizeof(nextTrackT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofpauseTrackT_34W539 = sizeof(pauseTrackT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofmetaFileNameT_34W539 = sizeof(metaFileNameT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofplayTrackT_34W539 = sizeof(playTrackT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofotherInfo2T_34W539 = sizeof(otherInfo2T_34W539)/sizeof(const uint8_t);
const uint8_t sizeofmetaArtNameT_34W539 = sizeof(metaArtNameT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofmetaTrackNameT_34W539 = sizeof(metaTrackNameT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofotherInfo1T_34W539 = sizeof(otherInfo1T_34W539)/sizeof(const uint8_t);


static bool log_verbose = false;

/**********************************************
DEBUG MACROS
**********************************************/
#ifndef __dbg_h__
#define __dbg_h__

#include <stdio.h>
#include <errno.h>
#include <string.h>

#ifdef NDEBUG
#define debug(M, ...)
#else
#define debug(M, ...) ((log_verbose == true) ? (Serial.printf("DEBUG (%u:%s:%d:) " M "\n", millis(), __FILE__, __LINE__, ##__VA_ARGS__)) : 0 )
#endif

#define clean_errno() (errno == 0 ? "None" : strerror(errno))

#define log_err(M, ...) Serial.printf("[ERROR] (%s:%d: errno: %s) " M "\n", __FILE__, __LINE__, clean_errno(), ##__VA_ARGS__)

#define log_warn(M, ...) Serial.printf("[WARN] (%s:%d: errno: %s) " M "\n", __FILE__, __LINE__, clean_errno(), ##__VA_ARGS__)

#define log_info(M, ...) Serial.printf("[INFO] (%s:%d) " M "\n", __FILE__, __LINE__, ##__VA_ARGS__)

#define check(A, M, ...) if(!(A)) { log_err(M, ##__VA_ARGS__); errno=0; goto error; }

#define sentinel(M, ...)  { log_err(M, ##__VA_ARGS__); errno=0; goto error; }

#define check_mem(A) check((A), "Out of memory.")

#define check_debug(A, M, ...) if(!(A)) { debug(M, ##__VA_ARGS__); errno=0; goto error; }

#endif
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
  Serial.begin(57600);

  digitalWriteFast(_MTS_34W539_CS_PIN, HIGH); 
  digitalWriteFast(DMTS_34W539_MOSI_PIN, HIGH);

  delay(1000);

  printHelp();
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
  // internal commands
  bool printHelp;
  bool custom;
  bool debug;
  // drive commands
  bool init;
  bool playTrack;
  bool pauseTrack;
  bool nextTrack;
  bool previousTrack;
  bool nextDirectory;
  bool previousDirectory;
  bool ejectDisc;
  bool infoDisc;
  bool infoTrack;
  bool randomEnable;
  bool randomDisable;
  bool fastForward;
  bool rewind;
  bool metaDirName;
  bool metaArtName;
  bool metaTrackName;
  bool metaFileName;
  bool otherInfo;
}cmdtx_s;

cmdtx_s cmdtx = {false};

typedef struct
{
  bool printHelp;
  bool init;
  bool playTrack;
  bool pauseTrack;
  bool nextTrack;
  bool previousTrack;
  bool nextDirectory;
  bool previousDirectory;
  bool ejectDisc;
  bool infoDisc;
  bool infoTrack;
  bool randomEnable;
  bool randomDisable;
  bool fastForward;
  bool rewind;
  bool metaDirName;
  bool metaArtName;
  bool metaTrackName;
  bool metaFileName;
  bool otherInfo;
}cmdrx_s;

cmdrx_s cmdrx = {false};

typedef struct
{
  uint8_t cmd;
  bool *cmdEvent;
  const char *infoMsg;
}serialtx_s;

const serialtx_s serialtx[]=
{
  // internal commands
  {'h',  &cmdtx.printHelp ,        "PRINTS HELP"},
  {'s',  &cmdtx.custom ,           "CUSTOM CMD (\"s 0xAA 0xBB 0xCC...\")"},
  {'y',  &cmdtx.debug ,            "VERBOSE"},
  // drive commands
  {'a',  &cmdtx.init ,             "INIT"},
  {'e',  &cmdtx.ejectDisc ,        "EJECT"},
  {'p',  &cmdtx.pauseTrack ,       "PAUSE"},
  {'P',  &cmdtx.playTrack ,        "PLAY"},
  {'N',  &cmdtx.nextTrack ,        "NEXT TRACK"},
  {'n',  &cmdtx.previousTrack ,    "PREVIOUS TRACK"},
  {'F',  &cmdtx.fastForward ,      "FAST FORWARD"},
  {'f',  &cmdtx.rewind ,           "REWIND"},
  {'I',  &cmdtx.infoDisc ,         "DISC INFO"},
  {'i',  &cmdtx.infoTrack ,        "TRACK INFO"},
  {'R',  &cmdtx.randomEnable ,     "RANDOM ON"},
  {'r',  &cmdtx.randomDisable ,    "RANDOM OFF"},
  {'w',  &cmdtx.metaDirName ,      "DIRECTORY NAME"},
  {'x',  &cmdtx.metaArtName ,      "ARTIST NAME"},
  {'c',  &cmdtx.metaTrackName ,    "TRACK NAME"},
  {'v',  &cmdtx.metaFileName ,     "FILE NAME"},
  {'b',  &cmdtx.otherInfo ,        "OTHER INFO"},
  {'D',  &cmdtx.nextDirectory ,    "NEXT DIRECTORY"},
  {'d',  &cmdtx.previousDirectory ,"PREVIOUS DIRECTORY"}
};
const uint8_t sizeofserialtx = sizeof(serialtx)/sizeof(serialtx_s);

void printHelp(void)
{
  uint8_t i;
  const serialtx_s *pserialtx;
  pserialtx = serialtx;

  for(i=0;i<sizeofserialtx;i++)
  {
    Serial.printf("%c - %s\n",pserialtx->cmd,pserialtx->infoMsg);
    pserialtx++;
  }
}

void serialEvent()
{
  uint8_t i;
  const serialtx_s *pserialtx;
  pserialtx = serialtx;
  uint8_t receivedChar = Serial.read();
  
  for(i=0;i<sizeofserialtx;i++)
  {
    if(pserialtx->cmd == receivedChar)
    {
        Serial.printf(">%s\n",pserialtx->infoMsg);
        if(pserialtx->cmdEvent != NULL)
        {
          *pserialtx->cmdEvent = true;
          break;
        }
    }
    pserialtx++;
  }
}

uint8_t toHex(uint8_t hi, uint8_t lo)
{
 uint8_t b;
 hi = toupper(hi);
 if( isxdigit(hi) ) {
   if( hi > '9' ) hi -= 7;      // software offset for A-F
   hi -= 0x30;                  // subtract ASCII offset
   b = hi<<4;
   lo = toupper(lo);
   if( isxdigit(lo) ) {
     if( lo > '9' ) lo -= 7;  // software offset for A-F
     lo -= 0x30;              // subtract ASCII offset
     b = b + lo;
     return b;
   } // else error
 }  // else error
 return 0;
}

bool buildCmd(uint8_t *rbyte)
{
  static uint8_t i,hi,lo = 0;
  static bool hexFound = false;

  if(hexFound == false)
  {
    if(i == 0 && *rbyte == '0')i++;
    else
    {
      if(i != 0 && (*rbyte == 'x' || *rbyte == 'X')){ hexFound = true;}
      i = 0;
    }
  }
  else
  {
    if(i == 0){ hi = *rbyte;i++;}
    else
    {
      if(*rbyte == ' ' || *rbyte == '\t') { *rbyte = hi; hi = '0';}
      lo = *rbyte;
      *rbyte = toHex(hi,lo);
      hexFound = false;
      i = 0;
      return true;
    }
  }
  return false;
}
/**********************************************
(2) 34W515 Slave protocol emulator
**********************************************/

bool onReceiveStartMsg_34W515(void)
{
  bool error;
  bool result=true;
  bool OnGoingTransmission = false;
  uint8_t receivechar;
  
  if(0)
  {
    OnGoingTransmission = true;
  }

  if(0)
  {
    for (int channel = 0; channel < sizeof(receivechar)/sizeof(int); channel++)
    {
      error = digitalHWSPIWrite(SLAVE_ACK,&receivechar);
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
  uint8_t receivechar;
  bool error;


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
        error = digitalHWSPIWrite(StartMsgT_34W515[channel],&receivechar);
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
  bool *cmd;
  const uint8_t *msg;
  uint8_t size;
}Msg_s;

const Msg_s txMsg[]=
{
  //COMMAND LIST             //MESSAGE TO SEND           //SIZE TO SEND
  {&cmdtx.init ,             initT_34W539,               sizeofinitT_34W539},
  {&cmdtx.ejectDisc ,        ejectDiscT_34W539,          sizeofejectDiscT_34W539},
  {&cmdtx.pauseTrack ,       pauseTrackT_34W539,         sizeofpauseTrackT_34W539},
  {&cmdtx.playTrack ,        playTrackT_34W539,          sizeofplayTrackT_34W539},
  {&cmdtx.nextTrack ,        nextTrackT_34W539,          sizeofnextTrackT_34W539},
  {&cmdtx.previousTrack ,    previousTrackT_34W539,      sizeofpreviousTrackT_34W539},
  {&cmdtx.fastForward ,      fastForwardT_34W539,        sizeoffastForwardT_34W539},
  {&cmdtx.rewind ,           rewindT_34W539,             sizeofrewindT_34W539},
  {&cmdtx.infoDisc ,         infoDiscT_34W539,           sizeofinfoDiscT_34W539},
  {&cmdtx.infoTrack ,        infoTrackT_34W539,          sizeofinfoTrackT_34W539},
  {&cmdtx.randomEnable ,     randomEnableT_34W539,       sizeofrandomEnableT_34W539},
  {&cmdtx.randomDisable ,    randomDisableT_34W539,      sizeofrandomDisableT_34W539},
  {&cmdtx.metaDirName ,      metaDirNameT_34W539,        sizeofmetaDirNameT_34W539},
  {&cmdtx.metaArtName ,      metaArtNameT_34W539,        sizeofmetaArtNameT_34W539},
  {&cmdtx.metaTrackName ,    metaTrackNameT_34W539,      sizeofmetaTrackNameT_34W539},
  {&cmdtx.metaFileName ,     metaFileNameT_34W539,       sizeofmetaFileNameT_34W539},
  {&cmdtx.otherInfo ,        otherInfo1T_34W539,         sizeofotherInfo1T_34W539},
  {&cmdtx.nextDirectory ,    nextDirectoryT_34W539,      sizeofnextDirectoryT_34W539},
  {&cmdtx.previousDirectory, previousDirectoryT_34W539,  sizeofpreviousDirectoryT_34W539}
};
const uint8_t sizeoftxMsg = sizeof(txMsg)/sizeof(Msg_s);
/*
const Msg_s rxMsg[]=
{
  //COMMAND LIST             //MESSAGE TO RECEIVE        //SIZE TO RECEIVE
  {&cmdrx.init ,             initR_34W539,               sizeofinitT_34W539},
  {&cmdrx.ejectDisc ,        ejectDiscR_34W539,          sizeofejectDiscT_34W539},
  {&cmdrx.pauseTrack ,       pauseTrackR_34W539,         sizeofpauseTrackT_34W539},
  {&cmdrx.playTrack ,        playTrackR_34W539,          sizeofplayTrackT_34W539},
  {&cmdrx.nextTrack ,        nextTrackR_34W539,          sizeofnextTrackT_34W539},
  {&cmdrx.previousTrack ,    previousTrackR_34W539,      sizeofpreviousTrackT_34W539},
  {&cmdrx.fastForward ,      fastForwardR_34W539,        sizeoffastForwardT_34W539},
  {&cmdrx.rewind ,           rewindR_34W539,             sizeofrewindT_34W539},
  {&cmdrx.infoDisc ,         infoDiscT_34W539,           sizeofinfoDiscT_34W539},
  {&cmdrx.infoTrack ,        infoTrackT_34W539,          sizeofinfoTrackT_34W539},
  {&cmdrx.randomEnable ,     randomEnableT_34W539,       sizeofrandomEnableT_34W539},
  {&cmdrx.randomDisable ,    randomDisableT_34W539,      sizeofrandomDisableT_34W539},
  {&cmdrx.metaDirName ,      metaDirNameT_34W539,        sizeofmetaDirNameT_34W539},
  {&cmdrx.metaArtName ,      metaArtNameT_34W539,        sizeofmetaArtNameT_34W539},
  {&cmdrx.metaTrackName ,    metaTrackNameT_34W539,      sizeofmetaTrackNameT_34W539},
  {&cmdrx.metaFileName ,     metaFileNameT_34W539,       sizeofmetaFileNameT_34W539},
  {&cmdrx.otherInfo ,        otherInfo1T_34W539,         sizeofotherInfo1T_34W539},
  {&cmdrx.nextDirectory ,    nextDirectoryT_34W539,      sizeofnextDirectoryT_34W539},
  {&cmdrx.previousDirectory, previousDirectoryT_34W539,  sizeofpreviousDirectoryT_34W539}
};
const uint8_t sizeofrxMsg = sizeof(rxMsg)/sizeof(Msg_s);
*/

#define RECEIVE_FORMAT_RAW 0
#define RECEIVE_FORMAT_ASCII 1

void MCDEmu_Master_34W539(void)
{
  static uint8_t receivesize = 0;
  static uint8_t receiveformat = RECEIVE_FORMAT_RAW;
  
  bool error;
  uint8_t receivechar,sendchar;
  uint8_t lIdx;
  uint8_t lPosition = 0;
  const Msg_s *ptxMsg = NULL;
  uint8_t customCmd[20] = {0};
  uint8_t customChar;
  uint8_t size = 0;

  ptxMsg = txMsg;

  if(cmdtx.printHelp)
  {
    cmdtx.printHelp = false;
    printHelp();
  }

  if(cmdtx.custom)
  {
    cmdtx.custom = false;
    while(Serial.available())
    {
      customChar = Serial.read();
      if(buildCmd(&customChar))
      {
        customCmd[size] = customChar;
        size++;
      }
    }

    for(lPosition = 0; lPosition < size; lPosition++)
    {
      sendchar = customCmd[lPosition];
      receivechar = 0;

      error = digitalSWSPITransfer(sendchar,&receivechar);
      // error check
      if(error == true)
      {
        delay(10);
        break;
      }
      else if(receivechar != SLAVE_ACK)
      {
        delay(10);
        debug("tx 0x%02X: rx: 0x%02X", sendchar, receivechar);
        break;
      }
      //no error
      // 2ms between each byte
      if(lPosition != (size-1))
      {
        delay(2);
        Serial.printf("0x%02X\t",sendchar);
      }
      else
      {
        Serial.printf("0x%02X\n",sendchar);
      }
    }
  }

  if(cmdtx.debug)
  {
    cmdtx.debug = false;
    log_verbose = !log_verbose;
  }

  for(lIdx = 0; lIdx < sizeoftxMsg; lIdx++)
  {
    if(*ptxMsg->cmd == true)
    {
      for(lPosition = 0; lPosition < ptxMsg->size; lPosition++)
      {
        sendchar = ptxMsg->msg[lPosition];
        receivechar = 0;

        error = digitalSWSPITransfer(sendchar,&receivechar);
        // error check
        if(error == true)
        {
          delay(10);
          break;
        }
        else if(receivechar != SLAVE_ACK)
        {
          delay(10);
          debug("tx 0x%02X: rx: 0x%02X", sendchar, receivechar);
          break;
        }
        //no error
        // 2ms between each byte
        if(lPosition != (ptxMsg->size-1))
        {
          delay(2);
        }
        else
        {
          *ptxMsg->cmd = false;
          break;
        }
      }
      break;
    }
    ptxMsg++;
  }

  // no transmission going on
  if(!digitalReadFast(_STM_34W539_CS_PIN) && receivesize == 0)
  {
    // we force a read
    receivesize = 1;
  }

  // read is acknd
  while(receivesize != 0)
  {
    delay(1);
    sendchar = R_34W539_ACK;
    receivechar = 0;

    error = digitalSWSPITransfer(sendchar,&receivechar);

    if(receivechar == T_34W539_STATUS)
    {
      receivesize = 11;
      receiveformat = RECEIVE_FORMAT_RAW;
    }
    else if(receivechar == T_34W539_METADATA)
    {
      receivesize = 25;
      receiveformat = RECEIVE_FORMAT_ASCII;
    }

    // debug
    switch(receiveformat)
    {
      case RECEIVE_FORMAT_RAW:
      {
        ((log_verbose == true) ? (Serial.printf("0x%02X ",receivechar)) : 0);
        break;
      }
      case RECEIVE_FORMAT_ASCII:
      {
        ((log_verbose == true) ? (Serial.printf("%c ",receivechar)) : 0);
        break;        
      }
    }
    receivesize--;
    if(receivesize == 0) ((log_verbose == true) ? (Serial.printf("\n")) : 0);
  }
}

/**********************************************
Software SPI Transfer
**********************************************/
#define TIMEOUT_24MHZ_CS 1000
#define TIMEOUT_24MHZ_CLK 100
#define TIMEOUT_48MHZ_CS (TIMEOUT_24MHZ_CS*3)
#define TIMEOUT_48MHZ_CLK (TIMEOUT_24MHZ_CLK*3)
#define TIMEOUT_72MHZ_CS (TIMEOUT_48MHZ_CS*2)
#define TIMEOUT_72MHZ_CLK (TIMEOUT_48MHZ_CLK*2)
#define TIMEOUT_96MHZ_CS (TIMEOUT_72MHZ_CS*2)
#define TIMEOUT_96MHZ_CLK (TIMEOUT_72MHZ_CLK*2)

#if F_CPU < 48000000
#define TIMEOUT_CS TIMEOUT_24MHZ_CS
#define TIMEOUT_CLK TIMEOUT_24MHZ_CLK
#elif F_CPU < 72000000
#define TIMEOUT_CS TIMEOUT_48MHZ_CS
#define TIMEOUT_CLK TIMEOUT_48MHZ_CLK
#elif F_CPU < 96000000
#define TIMEOUT_CS TIMEOUT_72MHZ_CS
#define TIMEOUT_CLK TIMEOUT_72MHZ_CLK
#else
#define TIMEOUT_CS TIMEOUT_96MHZ_CS
#define TIMEOUT_CLK TIMEOUT_96MHZ_CLK
#endif

inline bool digitalSWSPITransfer(uint8_t sendchar, uint8_t *receivechar)
{
  uint8_t bitPosition = 0;
  uint8_t sbit[8] = {0};
  uint8_t rbit[8] = {0};
  uint32_t timeout = 0;

  // slave can accept transmission
  if(sendchar == R_34W539_ACK)
  while(digitalReadFast(_STM_34W539_CS_PIN));
  else
  while(!digitalReadFast(_STM_34W539_CS_PIN));
  
  // take the SS pin low to select the chip:
  digitalWriteFast(_MTS_34W539_CS_PIN, LOW);

  if(sendchar == R_34W539_ACK)
  while(!digitalReadFast(_STM_34W539_CS_PIN)){timeout++;if(timeout>TIMEOUT_CS)goto err_cs;}
  else
  while(digitalReadFast(_STM_34W539_CS_PIN)){timeout++;if(timeout>TIMEOUT_CS)goto err_cs;}

  for(bitPosition = 0; bitPosition < 8; bitPosition++)
  {
    sbit[bitPosition] = (((sendchar & (1<<bitPosition)) == 0) ? 0 : 1);
  }
  for(bitPosition = 0; bitPosition < 8; bitPosition++)
  {
    timeout = 0;
    while(digitalReadFast(_SCK_34W539_CLK_PIN)){timeout++;if(timeout>TIMEOUT_CLK)goto err_clk_f;}
    if(sbit[bitPosition] == 0)
    digitalWriteFast(DMTS_34W539_MOSI_PIN, LOW);
    else
    digitalWriteFast(DMTS_34W539_MOSI_PIN, HIGH);
    timeout = 0;
    while(!digitalReadFast(_SCK_34W539_CLK_PIN)){timeout++;if(timeout>TIMEOUT_CLK)goto err_clk_r;}
    rbit[bitPosition] = digitalReadFast(DSTM_34W539_MISO_PIN);
  }
  for(bitPosition = 0; bitPosition < 8; bitPosition++)
  {
    *receivechar |= ((rbit[bitPosition]) ? 1 : 0) << bitPosition;
  }

  delayMicroseconds(10);
  //reset MOSI pin
  digitalWriteFast(DMTS_34W539_MOSI_PIN, HIGH);
  if(sendchar == R_34W539_ACK)
  delayMicroseconds(50);
  else
  delayMicroseconds(890);

  // take the SS pin high to de-select the chip:
  digitalWriteFast(_MTS_34W539_CS_PIN, HIGH);

  return false;

err_cs:
  debug("timeout %u err_cs",timeout);
  goto err_end;

err_clk_f:
  debug("timeout %u err_clk_f bitPosition: %d",timeout,bitPosition);
  goto err_end;

err_clk_r:
  debug("timeout %u err_clk_r bitPosition: %d",timeout,bitPosition);
  goto err_end;

err_end:
  digitalWriteFast(DMTS_34W539_MOSI_PIN, HIGH);
  digitalWriteFast(_MTS_34W539_CS_PIN, HIGH);
  return true;
}

/**********************************************
Harwdare SPI Transfer
**********************************************/
bool digitalHWSPIWrite(uint8_t sendchar, uint8_t *receivechar)
{
  bool error = false;
  
  SPI.beginTransaction(settingsHWSPI);
  // take the SS pin low to select the chip:
  digitalWrite(_STM_34W515_CS_PIN, LOW);
//  delayMicroseconds(45);
  *receivechar = SPI.transfer(sendchar);
//  delayMicroseconds(60);
  // take the SS pin high to de-select the chip:
  digitalWrite(_STM_34W515_CS_PIN, HIGH);
  SPI.endTransaction();
  
  return error;
}
