/*******
MCDEmu header file
*******/
#ifndef MCDEmu
#define MCDEmu

//#define DEBUG_515

#include <stdbool.h>
#include <stdint.h>

extern bool log_verbose;

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
  unsigned char track_name[32];
  unsigned char artist_name[32];
  unsigned char folder_name[32];
  unsigned char file_name[32];
  _e_34w539status current_status;
  bool ismp3disk;
  bool israndomenabled;
}_s_genericstatus;

extern _s_genericstatus genericstatus;

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
Common Definitions
**********************************************/
#define MASTER_ACK 0xDB
#define SLAVE_ACK 0x5A

typedef struct
{
  uint8_t cmd;
  bool *cmdEvent;
  const char *infoMsg;
}serialtx_s;

typedef struct
{
  bool *cmd;
  const uint8_t *msg;
  uint8_t size;
}Msg_s;

bool digitalHWSPIWrite(uint8_t sendchar, uint8_t *receivechar);
inline bool digitalSWSPITransfer(uint8_t sendchar, uint8_t *receivechar);

#endif

