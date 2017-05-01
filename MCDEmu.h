/*******
MCDEmu header file
*******/
#ifndef MCDEmu
#define MCDEmu

//#define DEBUG_515

#include <stdbool.h>
#include <stdint.h>


/*WRAPPER*/
//#define WRAPPER
/*WRAPPER*/
#ifdef WRAPPER
#define OUTPUT 1
#define INPUT 0
#define pinMode(pin,mode) (mode==OUTPUT) ? (PI_GPIO_config(pin,BCM_GPIO_OUT)) : (PI_GPIO_config(pin,BCM_GPIO_IN))
#define HIGH 1
#define LOW 0
#define digitalWriteFast(pin,mode) (mode==HIGH) ? (PI_GPIO_set_n(pin)) : (PI_GPIO_clr_n(pin))
#define digitalReadFast(pin) PI_GPIO_lev_n(pin)
#define _printf(arg, ...) printf(arg, ##__VA_ARGS__)
#undef delay
#define delay(x) usleep(x * 1000)
#undef delayMicroseconds
#define delayMicroseconds(x) usleep(x)
#define millis() 0
#else
//#define pinMode(pin,mode)
//#define digitalWriteFast(pin,mode)
//#define digitalReadFast(pin) 0


#define _printf(arg, ...) Serial.printf(arg, ##__VA_ARGS__)
#endif

extern bool log_verbose;

#ifndef uint8_t
typedef unsigned char uint8_t;
#endif

typedef enum
{
  E_NO_CD = 0,
  E_CD_DETECTION = 1,
  E_CD_IN_STOP = 2,
  E_CD_BUSY = 3,
  E_CD_PLAYING = 4,
  E_CD_PAUSED = 5,
  E_CD_FFWD = 6,
  E_CD_REW = 7
}_e_cdstatus;

typedef struct
{
  uint8_t first_folder;
  uint8_t last_folder;
  uint8_t total_folders;
  uint8_t total_files;
  uint8_t total_tracks;
  uint8_t total_min;
  uint8_t total_sec;
  uint8_t current_folder;
  uint8_t current_track;
  uint8_t current_min;
  uint8_t current_sec;
  uint8_t track_name[32];
  uint8_t artist_name[32];
  uint8_t folder_name[32];
  uint8_t file_name[32];
  _e_cdstatus current_status;
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
#define debug(M, ...) ((log_verbose == true) ? (_printf("\nDEBUG (%u:%s:%d:) " M "\n", millis(), __FILENAME__, __LINE__, ##__VA_ARGS__)) : 0 )
#endif

#define clean_errno() (errno == 0 ? "None" : strerror(errno))

#define log_err(M, ...) _printf("[ERROR] (%s:%d: errno: %s) " M "\n", __FILENAME__, __LINE__, clean_errno(), ##__VA_ARGS__)

#define log_warn(M, ...) _printf("[WARN] (%s:%d: errno: %s) " M "\n", __FILENAME__, __LINE__, clean_errno(), ##__VA_ARGS__)

#define log_info(M, ...) _printf("[INFO] (%s:%d) " M "\n", __FILENAME__, __LINE__, ##__VA_ARGS__)

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
bool digitalSWSPITransfer(uint8_t sendchar, uint8_t *receivechar);

#endif
