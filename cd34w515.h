/*******
MCDEmu 34W515 header file
*******/

#include "Arduino.h"
#include "MCDEmu.h"

//debug spi
#ifdef DEBUG_515
#define _STM_34W515_CS_PIN    A1  //15
#define DSTM_34W515_MISO_PIN  A2  //16
#define DMTS_34W515_MOSI_PIN  A3  //17
#define _SCK_34W515_CLK_PIN   A4  //18
#define _MTS_34W515_CS_PIN    A5  //19
#else
//SPI 34W515
#define _STM_34W515_CS_PIN    SS    //10
#define DSTM_34W515_MOSI_PIN  MOSI  //11
#define DMTS_34W515_MISO_PIN  MISO  //12
#define _SCK_34W515_CLK_PIN   SCK   //13
#define _MTS_34W515_CS_PIN    A0    //14
#endif

#define R_34W515_ACK 0x74

/**********************************************
CD-P1L (34W515) Transmit Definitions
**********************************************/
#define T_34W515_STATUS_0_HIMASK_STATUS 0x60
#define T_34W515_STATUS_0_HIMASK_ERROR (T_34W515_STATUS_0_HIMASK_STATUS - 0x20)

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
#define T_34W515_CD_INFO_RESPONSE(nb_tracks, total_time) {0x6E, 0x01, nb_tracks, total_time, 0x01, 0x01}

/**********************************************
CD-P1L (34W515) Receive Definitions
**********************************************/
#define R_34W515_UNKNOWN_1ST_MSG {0x5F, 0x50, 0xFE, 0x3B}
#define R_34W515_EJECT_MSG {0xE1}
#define R_34W515_STOP_MSG {0xE2}
#define R_34W515_PLAY_MSG {0xE4}
#define R_34W515_SCAN_DISABLE_MSG {0xE4}
#define R_34W515_SCAN_ENABLE_MSG {0xE5}
#define R_34W515_FAST_FORWARD_MSG {0xE6}
#define R_34W515_REWIND_MSG {0xE7}
#define R_34W515_RANDOM_ENABLE_MSG {0xEA}
#define R_34W515_PAUSE_MSG {0xEC}
#define R_34W515_GOTO_TRACK_MSG(track_number) {0xF4, track_number}
#define R_34W515_ERROR_INFO_MSG {0xF7}
#define R_34W515_TRACK_INFO_MSG {0xF8}
#define R_34W515_RANDOM_DISABLE_MSG {0xFA}
#define R_34W515_DISKINFO_MSG {0xFC}

#define T_34W515_MAX_SIZE_TO_RECEIVE      30

typedef struct
{
  // internal commands
  bool printHelp;
  bool custom;
  bool debug;
  // drive commands
  bool init;
  bool playTrack;
  bool stopTrack;
  bool pauseTrack;
  bool nextTrack;
  bool previousTrack;
  bool ejectDisk;
  bool diskInfo;
  bool diskStructure;
  bool folderStructure;
  bool randomEnable;
  bool randomDisable;
  bool fastForward;
  bool rewind;
}cmdtx34w515_s;

typedef struct
{
  bool printHelp;
  bool init;
  bool playTrack;
  bool pauseTrack;
  bool stopTrack;
  bool nextTrack;
  bool previousTrack;
  bool ejectDisk;
  bool infoDisk;
  bool infoTrack;
  bool randomEnable;
  bool diskStructure;
  bool randomDisable;
  bool diskInfo;
  bool fastForward;
  bool rewind;
  bool otherInfo;
}cmdrx34w515_s;

extern cmdtx34w515_s tx34w515;
extern cmdrx34w515_s rx34w515;

bool MCDEmu_slave_34W515(void);

