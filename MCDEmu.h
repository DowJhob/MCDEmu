/*******
MCDEmu header file
*******/


/**********************************************
Common Definitions
**********************************************/
#define MASTER_ACK 0xDB
#define SLAVE_ACK 0x5A


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
}cmdtx_s;

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
}cmdrx_s;

typedef struct
{
  bool *cmd;
  const uint8_t *msg;
  uint8_t size;
}Msg_s;
