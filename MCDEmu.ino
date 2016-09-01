/*
  MCDEmu - Mitsubishi Car CD Drive Emulator and Head Unit Emulator
  Project page: https://github.com/johnbutol/MCDEmu

  Basic Diagram:
  Master HU Volvo (1) <-----34W515 5-wire protocol-----> MCDEmu 34W515 Slave protocol Emulator (2) <-----MCDEmu protocol translator (3)-----> MCDEmu 34W539 Master protocol Emulator (4) <-----34W539 5-wire protocol-----> Slave Chrysler CD drive (5)

  Basic usage:
  The goal of this project is to be able to connect a Volvo P1 Chassis head unit (1) (34W515 made by Mitsubishi) with no cdmp3 support to a Chrysler CD Drive (5) (34W539 made by Mitsubishi) with mp3 support, and have a fully flawless working CD MP3 receiver.
  The software below is therefore composed of three parts:
  (2) 34W515 Slave protocol emulator : behaves as a standard 34W515 CD drive for the Volvo HU
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

#include "34w539.h"
#include "34w515.h"
#include "MCDEmu.h"

//SPI 34W539
#define _STM_34W539_CS_PIN    A1  //15
#define DSTM_34W539_MISO_PIN  A2  //16
#define DMTS_34W539_MOSI_PIN  A3  //17
#define _SCK_34W539_CLK_PIN   A4  //18
#define _MTS_34W539_CS_PIN    A5  //19

//SPI 34W515
#define _STM_34W515_CS_PIN    SS    //10
#define DSTM_34W515_MOSI_PIN  MOSI  //11
#define DMTS_34W515_MISO_PIN  MISO  //12
#define _SCK_34W515_CLK_PIN   SCK   //13
#define _MTS_34W515_CS_PIN    A0    //14

//OTHER SIGNALS
// set pin 6 as the mute:
#define _MUTE_34W539_PIN 6
// set pin 7 as the cd detect:
#define DDCNT_34W539_PIN 7
// set pin 8 as the mute:
#define _MUTE_34W515_PIN 8
// set pin 9 as the cd detect:
#define DDCNT_34W515_PIN 9

// set up the speed, mode and endianness of each device
SPISettings settingsHWSPI(250000, LSBFIRST, SPI_MODE3);

/**********************************************
Common Definitions
**********************************************/
#define MASTER_ACK 0xDB
#define SLAVE_ACK 0x5A

const uint8_t StartMsgT_34W515[]=R_34W515_UNKNOWN_1ST_MSG;

const uint8_t initT_34W539[]=R_34W539_UNKNOWN_1ST_MSG;
const uint8_t diskInfoT_34W539[]=R_34W539_DISKINFO_MSG;
const uint8_t previousTrackT_34W539[]=R_34W539_PREVIOUS_TRACK_MSG;
const uint8_t metaDirNameT_34W539[]=R_34W539_METADATA_DIRECTORY_NAME_MSG(R_34W539_DIRECTORY_1);
const uint8_t metaTrackNameT_34W539[]=R_34W539_METADATA_TRACK_NAME_MSG(0,R_34W539_DIRECTORY_1,1);
const uint8_t metaArtNameT_34W539[]=R_34W539_METADATA_ARTIST_NAME_MSG(0,R_34W539_DIRECTORY_1,1);
const uint8_t metaFileNameT_34W539[]=R_34W539_METADATA_FILE_NAME_MSG(0,R_34W539_DIRECTORY_1,1);
const uint8_t diskStructureT_34W539[]=R_34W539_DISKSTRUCTURE_MSG;
const uint8_t folderStructureT_34W539[]=R_34W539_FOLDERSTRUCTURE_MSG(1);
const uint8_t playTrackT_34W539[]=R_34W539_PLAY_MSG;
const uint8_t stopTrackT_34W539[]=R_34W539_STOP_MSG;
const uint8_t pauseTrackT_34W539[]=R_34W539_PAUSE_MSG;
const uint8_t nextTrackT_34W539[]=R_34W539_NEXT_TRACK_MSG;
const uint8_t nextDirectoryT_34W539[]=R_34W539_DIRECTORY_SET_NEXT_MSG;
const uint8_t previousDirectoryT_34W539[]=R_34W539_DIRECTORY_SET_PREVIOUS_MSG;
const uint8_t ejectDiskT_34W539[]=R_34W539_EJECT_MSG;
const uint8_t randomEnableT_34W539[]=R_34W539_RANDOM_ENABLE_MSG;
const uint8_t randomDisableT_34W539[]=R_34W539_RANDOM_DISABLE_MSG;
const uint8_t fastForwardT_34W539[]=R_34W539_FAST_FORWARD_MSG;
const uint8_t rewindT_34W539[]=R_34W539_REWIND_MSG;

const uint8_t sizeofinitT_34W539 = sizeof(initT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofdiskInfoT_34W539 = sizeof(diskInfoT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofdiskStructureT_34W539 = sizeof(diskStructureT_34W539)/sizeof(const uint8_t);
const uint8_t sizeoffolderStructureT_34W539 = sizeof(folderStructureT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofpreviousTrackT_34W539 = sizeof(previousTrackT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofmetaDirNameT_34W539 = sizeof(metaDirNameT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofejectDiskT_34W539 = sizeof(ejectDiskT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofrandomEnableT_34W539 = sizeof(randomEnableT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofrandomDisableT_34W539 = sizeof(randomDisableT_34W539)/sizeof(const uint8_t);
const uint8_t sizeoffastForwardT_34W539 = sizeof(fastForwardT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofrewindT_34W539 = sizeof(rewindT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofpreviousDirectoryT_34W539 = sizeof(previousDirectoryT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofnextDirectoryT_34W539 = sizeof(nextDirectoryT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofnextTrackT_34W539 = sizeof(nextTrackT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofpauseTrackT_34W539 = sizeof(pauseTrackT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofstopTrackT_34W539 = sizeof(stopTrackT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofmetaFileNameT_34W539 = sizeof(metaFileNameT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofplayTrackT_34W539 = sizeof(playTrackT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofmetaArtNameT_34W539 = sizeof(metaArtNameT_34W539)/sizeof(const uint8_t);
const uint8_t sizeofmetaTrackNameT_34W539 = sizeof(metaTrackNameT_34W539)/sizeof(const uint8_t);

static bool log_verbose = false;

/**********************************************
DEBUG MACROS
**********************************************/
#ifndef __dbg_h__
#define __dbg_h__

#include <stdio.h>
#include <errno.h>
#include <string.h>

#define __FILENAME__ (strrchr(__FILE__, '\\') ? strrchr(__FILE__, '\\') + 1 : __FILE__)
#ifdef NDEBUG
#define debug(M, ...)
#else
#define debug(M, ...) ((log_verbose == true) ? (Serial.printf("\nDEBUG (%u:%s:%d:) " M "\n", millis(), __FILENAME__, __LINE__, ##__VA_ARGS__)) : 0 )
#endif

#define clean_errno() (errno == 0 ? "None" : strerror(errno))

#define log_err(M, ...) Serial.printf("[ERROR] (%s:%d: errno: %s) " M "\n", __FILENAME__, __LINE__, clean_errno(), ##__VA_ARGS__)

#define log_warn(M, ...) Serial.printf("[WARN] (%s:%d: errno: %s) " M "\n", __FILENAME__, __LINE__, clean_errno(), ##__VA_ARGS__)

#define log_info(M, ...) Serial.printf("[INFO] (%s:%d) " M "\n", __FILENAME__, __LINE__, ##__VA_ARGS__)

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

  // set the DDCNT_34W515 as an output:
  pinMode(DDCNT_34W515_PIN, OUTPUT);
   // set the _MUTE_34W515 as an output:
  pinMode(_MUTE_34W515_PIN, OUTPUT);

  // set the DDCNT_34W539 as an input:
  pinMode(DDCNT_34W539_PIN, INPUT);
   // set the _MUTE_34W539 as an input:
  pinMode(_MUTE_34W539_PIN, INPUT);

   // initialize SPI:
  SPI.begin();

  //for debug purposes
  Serial.begin(57600);

  //to Chrysler drive
  digitalWriteFast(_MTS_34W539_CS_PIN, HIGH);
  digitalWriteFast(DMTS_34W539_MOSI_PIN, HIGH);

  //to car
  digitalWriteFast(_STM_34W515_CS_PIN, HIGH);
  digitalWriteFast(DSTM_34W515_MOSI_PIN, HIGH);

  //to car
  digitalWriteFast(DDCNT_34W515_PIN, HIGH);
  digitalWriteFast(_MUTE_34W515_PIN, HIGH);

  delay(1000);

  printHelp();
}

/**********************************************
LOOP
**********************************************/
void loop()
{
  bool error;
//  MCDEmu_slave_34W515();
  error = MCDEmu_generic_commands();
  if(error == false)
  error = MCDEmu_master_34W539();
}

cmdtx_s tx34w539 = {false};
cmdtx_s rx34w515 = {false};

typedef struct
{
  uint8_t cmd;
  bool *cmdEvent;
  const char *infoMsg;
}serialtx_s;

const serialtx_s serialtxcommon[]=
{
  // common commands
  {'h',  &tx34w539.printHelp ,        "PRINTS HELP"},
  {'s',  &tx34w539.custom ,           "CUSTOM CMD (\"s 0xAA 0xBB 0xCC...\")"},
  {'y',  &tx34w539.debug ,            "VERBOSE"},
  {NULL}
};

const serialtx_s serialtx34w539[]=
{
  // drive commands
  {'a',  &tx34w539.init ,             "INIT"},
  {'e',  &tx34w539.ejectDisk ,        "EJECT"},
  {'m',  &tx34w539.stopTrack ,        "STOP"},
  {'p',  &tx34w539.pauseTrack ,       "PAUSE"},
  {'P',  &tx34w539.playTrack ,        "PLAY"},
  {'N',  &tx34w539.nextTrack ,        "NEXT TRACK"},
  {'n',  &tx34w539.previousTrack ,    "PREVIOUS TRACK"},
  {'F',  &tx34w539.fastForward ,      "FAST FORWARD"},
  {'f',  &tx34w539.rewind ,           "REWIND"},
  {'I',  &tx34w539.diskInfo ,         "DISK INFO"},
  {'i',  &tx34w539.diskStructure ,    "DISK STRUCTURE"},
  {'R',  &tx34w539.randomEnable ,     "RANDOM ON"},
  {'r',  &tx34w539.randomDisable ,    "RANDOM OFF"},
  {'w',  &tx34w539.metaDirName ,      "DIRECTORY NAME"},
  {'x',  &tx34w539.metaArtName ,      "ARTIST NAME"},
  {'c',  &tx34w539.metaTrackName ,    "TRACK NAME"},
  {'v',  &tx34w539.metaFileName ,     "FILE NAME"},
  {'b',  &tx34w539.folderStructure ,  "FOLDER STRUCTURE"},
  {'D',  &tx34w539.nextDirectory ,    "NEXT DIRECTORY"},
  {'d',  &tx34w539.previousDirectory ,"PREVIOUS DIRECTORY"},
  {NULL}
};

const serialtx_s serialtx34w515[]=
{
  // 34w515 drive commands
  {'a',  &rx34w515.init ,             "INIT"},
  {'e',  &rx34w515.ejectDisk ,        "EJECT"},
  {'m',  &rx34w515.stopTrack ,        "STOP"},
  {'p',  &rx34w515.pauseTrack ,       "PAUSE"},
  {'P',  &rx34w515.playTrack ,        "PLAY"},
  {'N',  &rx34w515.nextTrack ,        "NEXT TRACK"},
  {'n',  &rx34w515.previousTrack ,    "PREVIOUS TRACK"},
  {'F',  &rx34w515.fastForward ,      "FAST FORWARD"},
  {'f',  &rx34w515.rewind ,           "REWIND"},
  {'I',  &rx34w515.diskInfo ,         "DISK INFO"},
  {'i',  &rx34w515.diskStructure ,    "DISK STRUCTURE"},
  {'R',  &rx34w515.randomEnable ,     "RANDOM ON"},
  {'r',  &rx34w515.randomDisable ,    "RANDOM OFF"},
  {'w',  &rx34w515.metaDirName ,      "DIRECTORY NAME"},
  {'x',  &rx34w515.metaArtName ,      "ARTIST NAME"},
  {'c',  &rx34w515.metaTrackName ,    "TRACK NAME"},
  {'v',  &rx34w515.metaFileName ,     "FILE NAME"},
  {'b',  &rx34w515.folderStructure ,  "FOLDER STRUCTURE"},
  {'D',  &rx34w515.nextDirectory ,    "NEXT DIRECTORY"},
  {'d',  &rx34w515.previousDirectory ,"PREVIOUS DIRECTORY"},
  {NULL}
};

typedef struct
{
  uint8_t cmd;
  const serialtx_s *tableptr;
  const char *infoMsg;
}serialtable_s;

const serialtable_s serialtabletop[]=
{
  {'a',  serialtxcommon ,        "INTERNAL"},
  {'e',  serialtx34w539 ,        "34W539 CMD"},
  {'m',  serialtx34w515 ,        "34W515 CMD"},
};

const uint8_t sizeofserialtabletop = sizeof(serialtabletop) / sizeof(serialtable_s);

void printHelp(void)
{
  uint8_t i;
  const serialtable_s *pserialtabletop;
  const serialtx_s *pserialtx;
  pserialtabletop = serialtabletop;
  pserialtx = pserialtabletop->tableptr;

  for(i = 0; i < sizeofserialtabletop; i++)
  {
    Serial.printf("List of available commands for module %s (%c)\n", pserialtabletop->infoMsg,pserialtabletop->cmd);
    while(1)
    {
      Serial.printf("%c - %s\n", pserialtx->cmd, pserialtx->infoMsg);
      pserialtx++;
      if(pserialtx->cmd == NULL)break;
    }
    pserialtabletop++;
    pserialtx = pserialtabletop->tableptr;
  }
}

void serialEvent()
{
  uint8_t i;
  const serialtable_s *pserialtabletop;
  const serialtx_s *pserialtx;
  pserialtabletop = serialtabletop;
  pserialtx = pserialtabletop->tableptr;
  uint8_t receivedChar = Serial.read();

  for(i = 0;i < sizeofserialtabletop; i++)
  {
    while(1)
    {
      if(pserialtx->cmd == receivedChar)
      {
          Serial.printf(">%s\n", pserialtx->infoMsg);
          if(pserialtx->cmdEvent != NULL)
          {
            *pserialtx->cmdEvent = true;
            break;
          }
      }
      pserialtx++;
      if(pserialtx->cmd == NULL)break;
    }
    pserialtabletop++;
    pserialtx = pserialtabletop->tableptr;
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
  static uint8_t i = 0, hi = 0, lo = 0;
  static bool hexFound = false;

  if(hexFound == false)
  {
    if(i == 0 && *rbyte == '0') i++;
    else
    {
      if(i != 0 && (*rbyte == 'x' || *rbyte == 'X')){ hexFound = true;}
      i = 0;
    }
  }
  else
  {
    if(i == 0){ hi = *rbyte; i++;}
    else
    {
      if(*rbyte == ' ' || *rbyte == '\t') { *rbyte = hi; hi = '0';}
      lo = *rbyte;
      *rbyte = toHex(hi, lo);
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
  bool result = true;
  bool OnGoingTransmission = false;
  uint8_t receivechar;

  if(0)
  {
    OnGoingTransmission = true;
  }

  if(0)
  {
    for (int channel = 0; channel < sizeof(receivechar) / sizeof(int); channel++)
    {
      error = digitalHWSPIWrite(SLAVE_ACK, &receivechar);
      while(0);
    }
  }
  return(result);
}

void MCDEmu_slave_34W515(void)
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

    for (int channel = 0; channel < sizeof(StartMsgT_34W515) / sizeof(int); channel++)
    {
        error = digitalHWSPIWrite(StartMsgT_34W515[channel], &receivechar);
        delayMicroseconds(850);
    }
  }
    // wait at the top:
    //delay(500);
}

/**********************************************
(3) Generic Layer: connecting the dots
**********************************************/
bool MCDEmu_generic_serial_cmd(void)
{
  bool error = false;
  uint8_t size = 0;
  uint8_t customcmd[20] = {0};
  uint8_t receivechar, sendchar;
  uint8_t i;

  if(tx34w539.printHelp)
  {
    tx34w539.printHelp = false;
    printHelp();
  }
  if(tx34w539.custom)
  {
    tx34w539.custom = false;
    while(Serial.available())
    {
      sendchar = Serial.read();
      if(buildCmd(&sendchar))
      {
        customcmd[size] = sendchar;
        size++;
      }
    }

    for(i = 0; i < size; i++)
    {
      sendchar = customcmd[i];
      receivechar = 0;

      error = digitalSWSPITransfer(sendchar, &receivechar);
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
      if(i == 0) Serial.printf(">");
      // 2ms between each byte
      if(i != (size - 1))
      {
        delay(2);
        Serial.printf("%d:0x%02X ", i, sendchar);
      }
      else
      {
        Serial.printf("%d:0x%02X\n", i, sendchar);
      }
    }
  }
  if(tx34w539.debug)
  {
    tx34w539.debug = false;
    log_verbose = !log_verbose;
    Serial.printf("<VERBOSE\t %d\n", log_verbose);
  }
  return error;
}

typedef enum
{
  E_34W539_NO_CD = 0,
  E_34W539_CD_DETECTION = 1,
  E_34W539_CD_IN_STOP = 2,
  E_34W539_CD_BUSY = 3,
  E_34W539_CD_PLAYING = 4,
  E_34W539_CD_PAUSED = 5,
  E_34W539_CD_FFWD = 6,
  E_34W539_CD_REW = 7
}_e_34w539status;

typedef struct
{
  unsigned char first_folder;
  unsigned char last_folder;
  unsigned char total_folders;
  unsigned char total_files;
  unsigned char total_tracks;
  unsigned char total_min;
  unsigned char total_sec;
  unsigned char current_folder;
  unsigned char current_track;
  unsigned char current_min;
  unsigned char current_sec;
  unsigned char track_name[2* T_34W539_METADATA_BYTE_SIZE];
  unsigned char artist_name[2* T_34W539_METADATA_BYTE_SIZE];
  unsigned char folder_name[2* T_34W539_METADATA_BYTE_SIZE];
  unsigned char file_name[2* T_34W539_METADATA_BYTE_SIZE];
  _e_34w539status current_status;
  bool ismp3disk;
  bool israndomenabled;
}_s_genericstatus;

_s_genericstatus genericstatus;

bool MCDEmu_generic_status_update(void)
{
  bool error = false;

  return error;
}

bool MCDEmu_generic_commands(void)
{
  bool error;
  error = MCDEmu_generic_serial_cmd();
  if(error == false)
  error = MCDEmu_generic_status_update();
  return error;
}

/**********************************************
(4) 34W539 Master protocol emulator
**********************************************/

const Msg_s tx34w539Msg[]=
{
  //34W539 COMMAND LIST      //MESSAGE TO SEND           //SIZE TO SEND
  {&tx34w539.init ,             initT_34W539,               sizeofinitT_34W539},
  {&tx34w539.ejectDisk ,        ejectDiskT_34W539,          sizeofejectDiskT_34W539},
  {&tx34w539.stopTrack ,        stopTrackT_34W539,          sizeofstopTrackT_34W539},
  {&tx34w539.pauseTrack ,       pauseTrackT_34W539,         sizeofpauseTrackT_34W539},
  {&tx34w539.playTrack ,        playTrackT_34W539,          sizeofplayTrackT_34W539},
  {&tx34w539.nextTrack ,        nextTrackT_34W539,          sizeofnextTrackT_34W539},
  {&tx34w539.previousTrack ,    previousTrackT_34W539,      sizeofpreviousTrackT_34W539},
  {&tx34w539.fastForward ,      fastForwardT_34W539,        sizeoffastForwardT_34W539},
  {&tx34w539.rewind ,           rewindT_34W539,             sizeofrewindT_34W539},
  {&tx34w539.diskInfo ,         diskInfoT_34W539,           sizeofdiskInfoT_34W539},
  {&tx34w539.diskStructure ,    diskStructureT_34W539,      sizeofdiskStructureT_34W539},
  {&tx34w539.folderStructure ,  folderStructureT_34W539,    sizeoffolderStructureT_34W539},
  {&tx34w539.randomEnable ,     randomEnableT_34W539,       sizeofrandomEnableT_34W539},
  {&tx34w539.randomDisable ,    randomDisableT_34W539,      sizeofrandomDisableT_34W539},
  {&tx34w539.metaDirName ,      metaDirNameT_34W539,        sizeofmetaDirNameT_34W539},
  {&tx34w539.metaArtName ,      metaArtNameT_34W539,        sizeofmetaArtNameT_34W539},
  {&tx34w539.metaTrackName ,    metaTrackNameT_34W539,      sizeofmetaTrackNameT_34W539},
  {&tx34w539.metaFileName ,     metaFileNameT_34W539,       sizeofmetaFileNameT_34W539},
  {&tx34w539.nextDirectory ,    nextDirectoryT_34W539,      sizeofnextDirectoryT_34W539},
  {&tx34w539.previousDirectory, previousDirectoryT_34W539,  sizeofpreviousDirectoryT_34W539}
};
const uint8_t sizeoftx34w539Msg = sizeof(tx34w539Msg)/sizeof(Msg_s);

bool MCDEmu_master_34W539_tx(void)
{
  bool error = false;
  uint8_t i = 0, j = 0;
  uint8_t receivechar, sendchar;
  const Msg_s *ptxMsg = NULL;

  ptxMsg = tx34w539Msg;

  for(j = 0; j < sizeoftx34w539Msg; j++)
  {
    if(*ptxMsg->cmd == true)
    {
      for(i = 0; i < ptxMsg->size; i++)
      {
        sendchar = ptxMsg->msg[i];
        receivechar = 0;

        error = digitalSWSPITransfer(sendchar, &receivechar);
        // error check
        if(error == true || receivechar != SLAVE_ACK)
        {
          delay(10);
          if(receivechar != SLAVE_ACK)
          debug("tx 0x%02X: rx: 0x%02X", sendchar, receivechar);
          break;
        }
        // no error
        if(i == 0) ((log_verbose == true) ? (Serial.printf(">")) : 0);
        // last message
        if(i != (ptxMsg->size - 1))
        {
          // 2ms between each byte
          delay(2);
          ((log_verbose == true) ? (Serial.printf("%d:0x%02X ", i, sendchar)) : 0);
        }
        else
        {
          *ptxMsg->cmd = false;
          ((log_verbose == true) ? (Serial.printf("%d:0x%02X\n", i, sendchar)) : 0);
          break;
        }
      }
      break;
    }
    ptxMsg++;
  }
  return error;
}

/*
const Msg_s rx34w539Msg[]=
{
  //34W539 COMMAND LIST      //MESSAGE TO RECEIVE        //SIZE TO RECEIVE
  {&rx34w539.init ,             initR_34W539,               sizeofinitT_34W539},
};
const uint8_t sizeofrx34w539Msg = sizeof(rx34w539Msg)/sizeof(Msg_s);
*/

bool MCDEmu_master_34W539_status_update(unsigned char *receivedchars)
{
  bool error = false;
  uint8_t i = 0;

  switch(*receivedchars)
  {
    case T_34W539_CMD_DISKINFO:
    {
      switch(*(receivedchars + T_34W539_DISK_INFO_1_BYTE))
      {
        case T_34W539_DISK_INFO_CMD_DISK:
        {
          genericstatus.total_tracks = *(receivedchars + T_34W539_DISKINFO_BYTE_5_NB_TRACKS);
          genericstatus.total_min = *(receivedchars + T_34W539_DISKINFO_BYTE_6_TOTAL_MIN);
          genericstatus.total_sec = *(receivedchars + T_34W539_DISKINFO_BYTE_7_TOTAL_SEC);
          break;
        }
        case T_34W539_DISK_INFO_CMD_FOLDER:
        {
          break;
        }
      }
    }
    case T_34W539_CMD_STATUS:
    {
/*      switch(*(receivedchars + T_34W539_STATUS_1_BYTE))
      {
        case:
        {
          break;
        }
      }
 */     break;
    }
    case T_34W539_CMD_METADATA:
    {
      switch(*(receivedchars + T_34W539_METADATA_BYTE_1_DATA_EXIST))
      {
        case T_34W539_METADATA_DATA_EXIST:
        {
          switch ((*(receivedchars + T_34W539_METADATA_BYTE_4_META_CMD)) & T_34W539_METADATA_BYTE_4_META_CMD_BMSK)
          {
            case T_34W539_METADATA_CMD_FILE:
            {
              memset(genericstatus.file_name, 0, 2 * T_34W539_METADATA_BYTE_SIZE);
              for(i = 0; i < T_34W539_METADATA_BYTE_SIZE; i++)
              {
                genericstatus.file_name[i + ((*(receivedchars + T_34W539_METADATA_BYTE_3_CURRENT_PAGE) - 1) * T_34W539_METADATA_BYTE_SIZE)] = (*(receivedchars + T_34W539_METADATA_BYTE_3_CURRENT_PAGE)) + i;
              }
              break;
            }
            case T_34W539_METADATA_CMD_FOLDER:
            {
              memset(genericstatus.folder_name, 0, 2 * T_34W539_METADATA_BYTE_SIZE);
              for(i = 0; i < T_34W539_METADATA_BYTE_SIZE; i++)
              {
                genericstatus.folder_name[i + ((*(receivedchars + T_34W539_METADATA_BYTE_3_CURRENT_PAGE) - 1) * T_34W539_METADATA_BYTE_SIZE)] = (*(receivedchars + T_34W539_METADATA_BYTE_3_CURRENT_PAGE)) + i;
              }
              break;
            }
            case T_34W539_METADATA_CMD_ARTIST:
            {
              memset(genericstatus.artist_name, 0, 2 * T_34W539_METADATA_BYTE_SIZE);
              for(i = 0; i < T_34W539_METADATA_BYTE_SIZE; i++)
              {
                genericstatus.artist_name[i + ((*(receivedchars + T_34W539_METADATA_BYTE_3_CURRENT_PAGE) - 1) * T_34W539_METADATA_BYTE_SIZE)] = (*(receivedchars + T_34W539_METADATA_BYTE_3_CURRENT_PAGE)) + i;
              }
              break;
            }
            case T_34W539_METADATA_CMD_TRACK:
            {
              memset(genericstatus.track_name, 0, 2 * T_34W539_METADATA_BYTE_SIZE);
              for(i = 0; i < T_34W539_METADATA_BYTE_SIZE; i++)
              {
                genericstatus.track_name[i + ((*(receivedchars + T_34W539_METADATA_BYTE_3_CURRENT_PAGE) - 1) * T_34W539_METADATA_BYTE_SIZE)] = (*(receivedchars + T_34W539_METADATA_BYTE_3_CURRENT_PAGE)) + i;
              }
              break;
            }
          }
          break;
        }
        case T_34W539_METADATA_DATA_NOT_EXIST:
        {
          break;
        }
      }
      break;
    }
    case T_34W539_CMD_DISK_STRUCTURE:
    {
      genericstatus.total_folders = *(receivedchars + T_34W539_DISKSTRUCTURE_BYTE_2_NB_DIRS);
      genericstatus.total_files = *(receivedchars + T_34W539_DISKSTRUCTURE_BYTE_3_NB_FILES);
      genericstatus.first_folder = *(receivedchars + T_34W539_DISKSTRUCTURE_BYTE_6_FIRST_DIR);
      genericstatus.last_folder = *(receivedchars + T_34W539_DISKSTRUCTURE_BYTE_7_LAST_DIR);
      break;
    }
  }
  return error;
}

bool MCDEmu_master_34W539_rx(void)
{
  bool error = false, receiveenable = false;
  uint8_t receivechar, sendchar;
  uint8_t receivecnt = 0;
  uint8_t receivedchars[T_34W539_MAX_SIZE_TO_RECEIVE] = {0};
  static uint8_t receivesize = T_34W539_MAX_SIZE_TO_RECEIVE;

  // no transmission going on and no error from last command
  if(!digitalReadFast(_STM_34W539_CS_PIN) && error == false)
  {
    // we force a read
    receiveenable = true;
  }

  while(receiveenable == true)
  {
    // 2ms between each byte
    delay(2);

    sendchar = R_34W539_ACK;
    receivechar = 0;

    error = digitalSWSPITransfer(sendchar, &receivechar);

    if(error == false)
    {
      receivedchars[receivecnt] = receivechar;

      // This switch is only here to find the size of the received packet
      switch(receivedchars[T_34W539_BYTE_0_CMD])
      {
        case T_34W539_CMD_STATUS:
        {
          if(receivecnt > 0)
          {
            //we can guess the size with the double 0xCC mark at the end of a frame here
            if(receivedchars[T_34W539_STATUS_1_BYTE] != (T_34W539_STATUS_1_HIMASK_CD_LOADED | T_34W539_STATUS_1_LOMASK_PLAYING))
            {
              if(receivedchars[receivecnt] == T_34W539_CMD_STATUS_END)
              {
                if(receivedchars[receivecnt - 1] == T_34W539_CMD_STATUS_END) receivesize = receivecnt;
                else if((receivedchars[receivecnt - 1] != T_34W539_CMD_STATUS_END) && receivecnt == receivesize) receivesize++;
              }
            }
            else
            {
              if(receivedchars[T_34W539_STATUS_2_BYTE_CDID_1] == T_34W539_CDID_1_NORMAL_CD) receivesize = T_34W539_SIZE_NORMAL_CD - 1;
              else receivesize = T_34W539_SIZE_MP3_CD - 1;
            }
          }
          break;
        }
        case T_34W539_CMD_DISKINFO:
        {
          receivesize = T_34W539_SIZE_DISKINFO - 1;
          break;
        }
        case T_34W539_CMD_METADATA:
        {
          if(receivecnt > 0)
          {
            if(receivedchars[T_34W539_STATUS_1_BYTE] != T_34W539_METADATA_DATA_EXIST) receivesize = T_34W539_SIZE_METADATA_NOK - 1;
            else receivesize = T_34W539_SIZE_METADATA_OK - 1;
          }

          if(receivecnt >= T_34W539_METADATA_BYTE_9_CHAR_0)
          {
            if(receivecnt == T_34W539_METADATA_BYTE_9_CHAR_0) Serial.printf("<");
            Serial.printf("%c", receivedchars[receivecnt]);
            if(receivecnt == receivesize) Serial.printf("\n");
          }
          if(receivesize == T_34W539_SIZE_METADATA_NOK - 1)
          {
            Serial.printf("<NO METADATA\n");
          }
          break;
        }
        case T_34W539_CMD_UNKNOWN_1:
        {
          receivesize = T_34W539_SIZE_UNKNOWN_1ST_MSG - 1;
          break;
        }
        case T_34W539_CMD_UNKNOWN_2:
        {
          receivesize = T_34W539_SIZE_UNKNOWN_2ND_MSG - 1;
          break;
        }
        case T_34W539_CMD_DISK_STRUCTURE:
        {
          receivesize = T_34W539_SIZE_DISK_STRUCTURE - 1;
          break;
        }
        case T_34W539_CMD_ERROR:
        {
          receivesize = T_34W539_SIZE_UNKNOWN_1 - 1;
          break;
        }
      }

      if(receivecnt == receivesize)
      {
        receiveenable = false;
        for(receivecnt = 0; receivecnt <= receivesize; receivecnt++)
        {
          if(receivecnt == 0) ((log_verbose == true) ? (Serial.printf("<")) : 0);
          ((log_verbose == true) ? (Serial.printf("%d:0x%02X ", receivecnt, receivedchars[receivecnt])) : 0);
          if(receivecnt == receivesize) ((log_verbose == true) ? (Serial.printf("\n")) : 0);
        }
        error = MCDEmu_master_34W539_status_update(receivedchars);
      }
      else
      {
        receivecnt++;
      }
    }
    else
    {
      receiveenable = false;
      debug("receivecnt = %d", receivecnt);
    }
  }
  return error;
}

bool MCDEmu_master_34W539(void)
{
  bool error;

  error = MCDEmu_master_34W539_tx();
  if(error == false)
  error = MCDEmu_master_34W539_rx();

  return error;
}
/**********************************************
Software SPI Transfer
**********************************************/
#define TIMEOUT_24MHZ_CS 1000
#define TIMEOUT_24MHZ_CLK 100
#define TIMEOUT_48MHZ_CS (TIMEOUT_24MHZ_CS * 3)
#define TIMEOUT_48MHZ_CLK (TIMEOUT_24MHZ_CLK * 3)
#define TIMEOUT_72MHZ_CS (TIMEOUT_48MHZ_CS * 2)
#define TIMEOUT_72MHZ_CLK (TIMEOUT_48MHZ_CLK * 2)
#define TIMEOUT_96MHZ_CS (TIMEOUT_72MHZ_CS * 2)
#define TIMEOUT_96MHZ_CLK (TIMEOUT_72MHZ_CLK * 2)

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
  uint8_t i = 0;
  uint8_t sbit[8] = {0};
  uint8_t rbit[8] = {0};
  uint32_t timeout = 0;

  // slave can accept transmission
  if(sendchar == R_34W539_ACK)
  while(digitalReadFast(_STM_34W539_CS_PIN)){ timeout++; if(timeout>TIMEOUT_CS) goto err_cs_pre;}
  else
  while(!digitalReadFast(_STM_34W539_CS_PIN)){ timeout++; if(timeout>TIMEOUT_CS) goto err_cs_pre;}

  // take the SS pin low to select the chip:
  digitalWriteFast(_MTS_34W539_CS_PIN, LOW);

  timeout = 0;

  if(sendchar == R_34W539_ACK)
  while(!digitalReadFast(_STM_34W539_CS_PIN)){ timeout++; if(timeout>TIMEOUT_CS) goto err_cs_post;}
  else
  while(digitalReadFast(_STM_34W539_CS_PIN)){ timeout++; if(timeout>TIMEOUT_CS) goto err_cs_post;}

  for(i = 0; i < 8; i++)
  {
    sbit[i] = (((sendchar & (1 << i)) == 0) ? 0 : 1);
  }

  for(i = 0; i < 8; i++)
  {
    timeout = 0;
    while(digitalReadFast(_SCK_34W539_CLK_PIN)){ timeout++; if(timeout>TIMEOUT_CLK) goto err_clk_f;}
    if(sbit[i] == 0)
    digitalWriteFast(DMTS_34W539_MOSI_PIN, LOW);
    else
    digitalWriteFast(DMTS_34W539_MOSI_PIN, HIGH);
    timeout = 0;
    while(!digitalReadFast(_SCK_34W539_CLK_PIN)){ timeout++; if(timeout>TIMEOUT_CLK) goto err_clk_r;}
    rbit[i] = digitalReadFast(DSTM_34W539_MISO_PIN);
  }
  for(i = 0; i < 8; i++)
  {
    *receivechar |= ((rbit[i]) ? 1 : 0) << i;
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

err_cs_pre:
  debug("err_cs_pre timeout %u sendchar 0x%02X", timeout, sendchar);
  goto err_end;

err_cs_post:
  debug("err_cs_post timeout %u sendchar 0x%02X", timeout, sendchar);
  goto err_end;

err_clk_f:
  debug("err_clk_f timeout %u sendchar 0x%02X bit %d", timeout, sendchar, i);
  goto err_end;

err_clk_r:
  debug("err_clk_r timeout %u sendchar 0x%02X bit %d", timeout, sendchar, i);
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
  digitalWriteFast(_STM_34W515_CS_PIN, LOW);
//  delayMicroseconds(45);
  *receivechar = SPI.transfer(sendchar);
//  delayMicroseconds(60);
  // take the SS pin high to de-select the chip:
  digitalWriteFast(_STM_34W515_CS_PIN, HIGH);
  SPI.endTransaction();

  return error;
}
