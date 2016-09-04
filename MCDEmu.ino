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

#include "cd34w539.h"
#include "cd34w515.h"
#include "MCDEmu.h"

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

bool log_verbose = false;

/**********************************************
SETUP
**********************************************/
void setup()
{
  //debug spi
#ifdef DEBUG_515
//34W539
  pinMode(_MTS_34W515_CS_PIN, OUTPUT);
  pinMode(DMTS_34W515_MOSI_PIN, OUTPUT);
  pinMode(_STM_34W515_CS_PIN, INPUT);
  pinMode(DSTM_34W515_MISO_PIN, INPUT);
  pinMode(_SCK_34W515_CLK_PIN, INPUT);
#else
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
#endif
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

    //debug spi
#ifdef DEBUG_515
  //to Volvo drive
  digitalWriteFast(_MTS_34W515_CS_PIN, HIGH);
  digitalWriteFast(DMTS_34W515_MOSI_PIN, HIGH);
#else
  //to Chrysler drive
  digitalWriteFast(_MTS_34W539_CS_PIN, HIGH);
  digitalWriteFast(DMTS_34W539_MOSI_PIN, HIGH);

  //to car
  digitalWriteFast(_STM_34W515_CS_PIN, HIGH);
  digitalWriteFast(DSTM_34W515_MOSI_PIN, HIGH);
  digitalWriteFast(_SCK_34W515_CLK_PIN, HIGH);

#endif

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
  bool error = false;

  error = MCDEmu_slave_34W515();
  if(error == false)
  error = MCDEmu_generic_commands();
  if(error == false)
  error = MCDEmu_master_34W539();
}

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
  // 34w539 drive commands
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
  // 34w515 tx drive commands
  {'a',  &tx34w515.init ,             "INIT"},
  {'e',  &tx34w515.ejectDisk ,        "EJECT"},
  {'m',  &tx34w515.stopTrack ,        "STOP"},
  {'p',  &tx34w515.pauseTrack ,       "PAUSE"},
  {'P',  &tx34w515.playTrack ,        "PLAY"},
  {'N',  &tx34w515.nextTrack ,        "NEXT TRACK"},
  {'n',  &tx34w515.previousTrack ,    "PREVIOUS TRACK"},
  {'F',  &tx34w515.fastForward ,      "FAST FORWARD"},
  {'f',  &tx34w515.rewind ,           "REWIND"},
  {'I',  &tx34w515.diskInfo ,         "DISK INFO"},
  {'i',  &tx34w515.diskStructure ,    "DISK STRUCTURE"},
  {'R',  &tx34w515.randomEnable ,     "RANDOM ON"},
  {'r',  &tx34w515.randomDisable ,    "RANDOM OFF"},
  {NULL}
};

const serialtx_s serialrx34w515[]=
{
  // 34w515 rx drive commands
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
#ifdef DEBUG_515
  {'e',  serialtx34w515 ,        "34W515 TX CMD"},
#else
  {'e',  serialtx34w539 ,        "34W539 TX CMD"},
#endif
  {'m',  serialrx34w515 ,        "34W515 RX CMD"},
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
    Serial.printf("List of available commands for module %s (%c)\n", pserialtabletop->infoMsg, pserialtabletop->cmd);
    while(1)
    {
      Serial.printf("%c - %s\n", pserialtx->cmd, pserialtx->infoMsg);
      pserialtx++;
      if(pserialtx->cmd == NULL) break;
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
      if(pserialtx->cmd == NULL) break;
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
  delayMicroseconds(45);
  *receivechar = SPI.transfer(sendchar);
  delayMicroseconds(60);
  // take the SS pin high to de-select the chip:
  digitalWriteFast(_STM_34W515_CS_PIN, HIGH);
  SPI.endTransaction();

  return error;
}


