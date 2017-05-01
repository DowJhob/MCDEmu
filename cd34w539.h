/*******
MCDEmu 34W539 header file
*******/
#include "MCDEmu.h"
#ifndef WRAPPER
#include "Arduino.h"
#endif

//SPI 34W539
#ifdef WRAPPER
#define _STM_34W539_CS_PIN   13 //A1  //15
#define DSTM_34W539_MISO_PIN 19 //A2  //16
#define DMTS_34W539_MOSI_PIN 6 //A3  //17
#define _SCK_34W539_CLK_PIN  26 //A4  //18
#define _MTS_34W539_CS_PIN   5 //A5  //19
#else
#define _STM_34W539_CS_PIN    A1  //15
#define DSTM_34W539_MISO_PIN  A2  //16
#define DMTS_34W539_MOSI_PIN  A3  //17
#define _SCK_34W539_CLK_PIN   A4  //18
#define _MTS_34W539_CS_PIN    A5  //19
#endif
/**********************************************
Chrysler CD drive (34W539) Transmit Definitions
**********************************************/
#define T_34W539_CMD_DISKINFO                  0xF2
#define T_34W539_CMD_UNKNOWN_2                 0xF4
#define T_34W539_CMD_UNKNOWN_1                 0xF5
#define T_34W539_CMD_STATUS                    0xF8
#define T_34W539_CMD_ERROR                     0xF9
#define T_34W539_CMD_METADATA                  0xFA
#define T_34W539_CMD_DISK_STRUCTURE            0xFB

#define T_34W539_CMD_STATUS_END                0xCC

#define T_34W539_BYTE_0_CMD                    0

#define T_34W539_STATUS_1_LOMASK_EJECTED       0x01
#define T_34W539_STATUS_1_LOMASK_INSERTING     0x02
#define T_34W539_STATUS_1_LOMASK_STOPPED       0x04
#define T_34W539_STATUS_1_LOMASK_PLAYING       0x05
#define T_34W539_STATUS_1_LOMASK_PAUSED        0x06
#define T_34W539_STATUS_1_LOMASK_FFWD          0x07
#define T_34W539_STATUS_1_LOMASK_REW           0x08
#define T_34W539_STATUS_1_LOMASK_SEEKTRACK     0x0B
#define T_34W539_STATUS_1_LOMASK_EJECTING      0x0D
#define T_34W539_STATUS_1_LOMASK_SEEKING       0x0F
#define T_34W539_STATUS_1_HIMASK_CD_NOT_LOADED 0x00
#define T_34W539_STATUS_1_HIMASK_CD_LOADED     0x20
#define T_34W539_STATUS_1_HIMASK_BUSY          0x80
#define T_34W539_STATUS_3_RANDOM_OFF           0x32
#define T_34W539_STATUS_3_RANDOM_ON            0x3D
#define T_34W539_STATUS_1_BYTE                 1

#define T_34W539_CDID_1_UNKNOWN_CD             0x01
#define T_34W539_CDID_1_NORMAL_CD              0x09
#define T_34W539_CDID_1_MP3_NOT_LOADED_CD      0x11
#define T_34W539_CDID_1_MP3_LOADED_CD          0x12
#define T_34W539_STATUS_2_BYTE_CDID_1          2

#define T_34W539_CDID_2_UNKNOWN_CD             0x10
#define T_34W539_CDID_2_NORMAL_CD              0x10
#define T_34W539_CDID_2_MP3_CD                 0x30
#define T_34W539_STATUS_3_BYTE_CDID_2          3

#define T_34W539_STATUS_MP3_5_BYTE_TRACK_100PLUS   5
#define T_34W539_STATUS_MP3_6_BYTE_DIRECTORY       6
#define T_34W539_STATUS_MP3_7_BYTE_TRACK_1_TO_99   7
#define T_34W539_STATUS_MP3_9_BYTE_MIN             9
#define T_34W539_STATUS_MP3_10_BYTE_SEC            10

#define T_34W539_STATUS_STD_6_BYTE_TRACK           6
#define T_34W539_STATUS_STD_8_BYTE_MIN             8
#define T_34W539_STATUS_STD_9_BYTE_SEC             9

#define T_34W539_SIZE_NORMAL_CD           10
#define T_34W539_SIZE_MP3_CD              11
#define T_34W539_SIZE_METADATA_OK         25
#define T_34W539_SIZE_METADATA_NOK        2
#define T_34W539_SIZE_DISKINFO            9
#define T_34W539_SIZE_UNKNOWN_1ST_MSG     6
#define T_34W539_SIZE_UNKNOWN_2ND_MSG     8
#define T_34W539_SIZE_DISK_STRUCTURE      8
#define T_34W539_SIZE_UNKNOWN_1           6

#define T_34W539_DISKINFO_BYTE_5_NB_TRACKS     5
#define T_34W539_DISKINFO_BYTE_6_TOTAL_MIN     6
#define T_34W539_DISKINFO_BYTE_7_TOTAL_SEC     7

#define T_34W539_METADATA_DATA_EXIST           0x00
#define T_34W539_METADATA_DATA_NOT_EXIST       0x01
#define T_34W539_METADATA_BYTE_1_DATA_EXIST    1
#define T_34W539_METADATA_BYTE_2_TOTAL_PAGE    2
#define T_34W539_METADATA_BYTE_3_CURRENT_PAGE  3
#define T_34W539_METADATA_BYTE_4_META_CMD_BMSK 0x07
#define T_34W539_METADATA_BYTE_4_META_CMD      4
#define T_34W539_METADATA_BYTE_7_DIR_NB        7
#define T_34W539_METADATA_BYTE_8_TRACK_NB      8
#define T_34W539_METADATA_BYTE_9_CHAR_0        9
#define T_34W539_METADATA_BYTE_24_CHAR_15      24
#define T_34W539_METADATA_BYTE_SIZE            ((T_34W539_METADATA_BYTE_24_CHAR_15 - T_34W539_METADATA_BYTE_9_CHAR_0)+1)

#define T_34W539_DISKSTRUCTURE_BYTE_2_NB_DIRS   2
#define T_34W539_DISKSTRUCTURE_BYTE_3_NB_FILES  3
#define T_34W539_DISKSTRUCTURE_BYTE_6_FIRST_DIR 6
#define T_34W539_DISKSTRUCTURE_BYTE_7_LAST_DIR  7

#define T_34W539_DISK_INFO_CMD_DISK    R_34W539_DISK_INFO_CMD_DISK
#define T_34W539_DISK_INFO_CMD_FOLDER  R_34W539_DISK_INFO_CMD_FOLDER
#define T_34W539_DISK_INFO_1_BYTE      R_34W539_DISK_INFO_1_BYTE

#define T_34W539_METADATA_CMD_FILE        R_34W539_METADATA_CMD_FILE
#define T_34W539_METADATA_CMD_FOLDER      R_34W539_METADATA_CMD_FOLDER
#define T_34W539_METADATA_CMD_ARTIST      R_34W539_METADATA_CMD_ARTIST
#define T_34W539_METADATA_CMD_TRACK       R_34W539_METADATA_CMD_TRACK

#define T_34W539_MAX_SIZE_TO_RECEIVE      30

#define T_34W539_UNKNOWN_1ST_MSG {T_34W539_CMD_UNKNOWN_1, 0x01, 0x09, 0xA0, 0x80, 0x18}
#define T_34W539_UNKNOWN_2ND_MSG {T_34W539_CMD_UNKNOWN_2, 0x00, 0x32, 0x05, 0x05, 0x09, 0x01, 0x0D}
#define T_34W539_DISKINFO_MSG {T_34W539_CMD_DISKINFO, 0x10, 0x00, 0x05, 0x01}
#define T_34W539_ERROR_RESPONSE {T_34W539_CMD_ERROR, 0x84}

/**********************************************
Chrysler CD drive (34W539) Receive Definitions
**********************************************/
#define R_34W539_ACK MASTER_ACK

#define R_34W539_CMD_METADATA           0xC7
#define R_34W539_CMD_DISK_STRUCTURE     0xC8
#define R_34W539_CMD_UNKNOWN_SET_FLDR   0xC9
#define R_34W539_CMD_UNKNOWN_1          0xF7
#define R_34W539_CMD_DISK_INFO          0xFC

#define R_34W539_DISK_INFO_CMD_DISK    0x10
#define R_34W539_DISK_INFO_CMD_FOLDER  0x20
#define R_34W539_DISK_INFO_1_BYTE      1

#define R_34W539_METADATA_CMD_FILE        0x01
#define R_34W539_METADATA_CMD_FOLDER      0x02
#define R_34W539_METADATA_CMD_ARTIST      0x04
#define R_34W539_METADATA_CMD_TRACK       0x05

#define R_34W539_UNKNOWN_1ST_MSG {0x5F, 0x01, 0x09, 0xA0, 0x80, 0x18}
#define R_34W539_UNKNOWN_2ND_MSG {0xAD}
#define R_34W539_EJECT_MSG {0xE1}
#define R_34W539_STOP_MSG {0xE2}
#define R_34W539_PLAY_MSG {0xE4}
#define R_34W539_FAST_FORWARD_MSG {0xE6, 0x02}
#define R_34W539_REWIND_MSG {0xE7, 0x02}
#define R_34W539_NEXT_TRACK_MSG {0xEA}
#define R_34W539_CURRENT_BACK_TRACK_MSG {0xEB, 0x01}
#define R_34W539_PREVIOUS_TRACK_MSG {0xEB, 0x02}
#define R_34W539_PAUSE_MSG {0xEC}
#define R_34W539_ERROR_STOP_MSG {0xF7}
#define R_34W539_RANDOM_DISABLE_MSG {0xF0}
#define R_34W539_RANDOM_ENABLE_MSG {0xFA}
#define R_34W539_DISKINFO_MSG {R_34W539_CMD_DISK_INFO, 0x10, 0x00}
#define R_34W539_DIRECTORY_SET_MSG(dirN) {0xC3, 0x00, 0x00, dirN, 0x01}
#define R_34W539_DIRECTORY_SET_PREVIOUS_MSG {0xC6}
#define R_34W539_DIRECTORY_SET_NEXT_MSG {0xC5}
#define R_34W539_METADATA_DIRECTORY_NAME_MSG(dirN) {R_34W539_CMD_METADATA, 0x01, 0x02, R_34W539_METADATA_CMD_FOLDER, 0x00, 0x00, dirN, 0x01}
#define R_34W539_METADATA_TRACK_NAME_MSG(trkN_100, dirN, trkN_up_to_99) {R_34W539_CMD_METADATA, 0x01, 0x02, R_34W539_METADATA_CMD_TRACK, 0x00, trkN_100, dirN, trkN_up_to_99}
#define R_34W539_METADATA_ARTIST_NAME_MSG(trkN_100, dirN, trkN_up_to_99) {R_34W539_CMD_METADATA, 0x01, 0x02, R_34W539_METADATA_CMD_ARTIST, 0x00, trkN_100, dirN, trkN_up_to_99}
#define R_34W539_METADATA_FILE_NAME_MSG(trkN_100, dirN, trkN_up_to_99) {R_34W539_CMD_METADATA, 0x01, 0x02, R_34W539_METADATA_CMD_FILE, 0x00, trkN_100, dirN, trkN_up_to_99}
#define R_34W539_DISKSTRUCTURE_MSG {R_34W539_CMD_DISK_STRUCTURE, R_34W539_DISK_INFO_CMD_DISK, 0x00}
#define R_34W539_FOLDERSTRUCTURE_MSG(dirN) {R_34W539_CMD_DISK_STRUCTURE, R_34W539_DISK_INFO_CMD_FOLDER, dirN}
#define R_34W539_UNKNOWN_1_MSG {R_34W539_CMD_UNKNOWN_SET_FLDR, 0x01}
#define R_34W539_DIRECTORY_ROOT 0x01
#define R_34W539_DIRECTORY_1 0x03

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
  bool nextDirectory;
  bool previousDirectory;
  bool ejectDisk;
  bool diskInfo;
  bool diskStructure;
  bool folderStructure;
  bool randomEnable;
  bool randomDisable;
  bool fastForward;
  bool rewind;
  bool metaDirName;
  bool metaArtName;
  bool metaTrackName;
  bool metaFileName;
}cmdtx34w539_s;

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
  bool ejectDisk;
  bool infoDisk;
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
}cmdrx34w539_s;


extern cmdtx34w539_s tx34w539;

bool MCDEmu_master_34W539(void);

