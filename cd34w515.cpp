#include "cd34w515.h"

const uint8_t initT_34W515[]=R_34W515_UNKNOWN_1ST_MSG;
const uint8_t diskInfoT_34W515[]=R_34W515_DISKINFO_MSG;
const uint8_t previousTrackT_34W515[]=R_34W515_GOTO_TRACK_MSG(2);
const uint8_t diskStructureT_34W515[]=R_34W515_TRACK_INFO_MSG;
const uint8_t folderStructureT_34W515[]=R_34W515_ERROR_INFO_MSG;
const uint8_t playTrackT_34W515[]=R_34W515_PLAY_MSG;
const uint8_t stopTrackT_34W515[]=R_34W515_STOP_MSG;
const uint8_t pauseTrackT_34W515[]=R_34W515_PAUSE_MSG;
const uint8_t nextTrackT_34W515[]=R_34W515_GOTO_TRACK_MSG(3);
const uint8_t ejectDiskT_34W515[]=R_34W515_EJECT_MSG;
const uint8_t randomEnableT_34W515[]=R_34W515_RANDOM_ENABLE_MSG;
const uint8_t randomDisableT_34W515[]=R_34W515_RANDOM_DISABLE_MSG;
const uint8_t fastForwardT_34W515[]=R_34W515_FAST_FORWARD_MSG;
const uint8_t rewindT_34W515[]=R_34W515_REWIND_MSG;

const uint8_t sizeofinitT_34W515 = sizeof(initT_34W515)/sizeof(const uint8_t);
const uint8_t sizeofdiskInfoT_34W515 = sizeof(diskInfoT_34W515)/sizeof(const uint8_t);
const uint8_t sizeofdiskStructureT_34W515 = sizeof(diskStructureT_34W515)/sizeof(const uint8_t);
const uint8_t sizeoffolderStructureT_34W515 = sizeof(folderStructureT_34W515)/sizeof(const uint8_t);
const uint8_t sizeofpreviousTrackT_34W515 = sizeof(previousTrackT_34W515)/sizeof(const uint8_t);
const uint8_t sizeofejectDiskT_34W515 = sizeof(ejectDiskT_34W515)/sizeof(const uint8_t);
const uint8_t sizeofrandomEnableT_34W515 = sizeof(randomEnableT_34W515)/sizeof(const uint8_t);
const uint8_t sizeofrandomDisableT_34W515 = sizeof(randomDisableT_34W515)/sizeof(const uint8_t);
const uint8_t sizeoffastForwardT_34W515 = sizeof(fastForwardT_34W515)/sizeof(const uint8_t);
const uint8_t sizeofrewindT_34W515 = sizeof(rewindT_34W515)/sizeof(const uint8_t);
const uint8_t sizeofnextTrackT_34W515 = sizeof(nextTrackT_34W515)/sizeof(const uint8_t);
const uint8_t sizeofpauseTrackT_34W515 = sizeof(pauseTrackT_34W515)/sizeof(const uint8_t);
const uint8_t sizeofstopTrackT_34W515 = sizeof(stopTrackT_34W515)/sizeof(const uint8_t);
const uint8_t sizeofplayTrackT_34W515 = sizeof(playTrackT_34W515)/sizeof(const uint8_t);

cmdtx34w515_s tx34w515 = {false};
cmdrx34w515_s rx34w515 = {false};

bool digitalHWSPIWrite(uint8_t sendchar, uint8_t *receivechar);

/**********************************************
(2) 34W515 Slave protocol emulator
**********************************************/
bool MCDEmu_slave_34W515_tx(void)
{
  bool error = false;

  return error;
}

bool MCDEmu_slave_34W515_rx(void)
{
  static bool masterFallingEdge = false;
  static bool masterHigh = false;
  uint8_t receivechar;
  bool error = false;

  if(!masterFallingEdge)
  {
    if(!masterHigh)
    {
      if(digitalReadFast(_MTS_34W515_CS_PIN))
      {
        masterHigh = true;
      }
    }
    if(masterHigh)
    {
      if(!digitalReadFast(_MTS_34W515_CS_PIN))
      {
        masterFallingEdge = true;
      }
    }
  }

  if(masterFallingEdge)
  {
    delayMicroseconds(40);
    error = digitalHWSPIWrite(SLAVE_ACK, &receivechar);
    debug("tx 0x%02X: rx: 0x%02X", SLAVE_ACK, receivechar);
    masterFallingEdge = false;
  }

  return error;
}

bool MCDEmu_slave_34W515(void)
{
  bool error = false;

  error = MCDEmu_slave_34W515_tx();
  if(error == false)
  error = MCDEmu_slave_34W515_rx();

  return error;
}

const Msg_s tx34w515Msg[]=
{
  //34W515 COMMAND LIST      //MESSAGE TO SEND           //SIZE TO SEND
  {&tx34w515.init ,             initT_34W515,               sizeofinitT_34W515},
  {&tx34w515.ejectDisk ,        ejectDiskT_34W515,          sizeofejectDiskT_34W515},
  {&tx34w515.stopTrack ,        stopTrackT_34W515,          sizeofstopTrackT_34W515},
  {&tx34w515.pauseTrack ,       pauseTrackT_34W515,         sizeofpauseTrackT_34W515},
  {&tx34w515.playTrack ,        playTrackT_34W515,          sizeofplayTrackT_34W515},
  {&tx34w515.nextTrack ,        nextTrackT_34W515,          sizeofnextTrackT_34W515},
  {&tx34w515.previousTrack ,    previousTrackT_34W515,      sizeofpreviousTrackT_34W515},
  {&tx34w515.fastForward ,      fastForwardT_34W515,        sizeoffastForwardT_34W515},
  {&tx34w515.rewind ,           rewindT_34W515,             sizeofrewindT_34W515},
  {&tx34w515.diskInfo ,         diskInfoT_34W515,           sizeofdiskInfoT_34W515},
  {&tx34w515.diskStructure ,    diskStructureT_34W515,      sizeofdiskStructureT_34W515},
  {&tx34w515.folderStructure ,  folderStructureT_34W515,    sizeoffolderStructureT_34W515},
  {&tx34w515.randomEnable ,     randomEnableT_34W515,       sizeofrandomEnableT_34W515},
  {&tx34w515.randomDisable ,    randomDisableT_34W515,      sizeofrandomDisableT_34W515}
};
const uint8_t sizeoftx34w515Msg = sizeof(tx34w515Msg)/sizeof(Msg_s);

bool MCDEmu_master_34W515_tx(void)
{
  bool error = false;
  uint8_t i = 0, j = 0;
  uint8_t receivechar, sendchar;
  const Msg_s *ptxMsg = NULL;

  ptxMsg = tx34w515Msg;

  for(j = 0; j < sizeoftx34w515Msg; j++)
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

  //debug spi
#ifdef DEBUG_515
bool MCDEmu_master_34W515_rx(void)
{
  bool error = false, receiveenable = false;
  uint8_t receivechar;
  uint8_t receivecnt = 0;
  uint8_t receivedchars[T_34W515_MAX_SIZE_TO_RECEIVE] = {0};
  static uint8_t receivesize = T_34W515_MAX_SIZE_TO_RECEIVE;

  // no transmission going on and no error from last command
  if(!digitalReadFast(_STM_34W515_CS_PIN) && error == false)
  {
    // we force a read
    receiveenable = true;
  }

  while(receiveenable == true)
  {
    // 2ms between each byte
    delay(2);

    receivechar = 0;

    error = digitalSWSPITransfer(R_34W515_ACK, &receivechar);

    if(error == false)
    {
      receivedchars[receivecnt] = receivechar;

      // This switch is only here to find the size of the received packet
      switch(receivedchars[T_34W515_BYTE_0_CMD])
      {
        case T_34W515_CMD_STATUS:
        {
          if(receivecnt > 0)
          {
            //we can guess the size with the double 0xCC mark at the end of a frame here
            if(receivedchars[T_34W515_STATUS_1_BYTE] != (T_34W515_STATUS_1_HIMASK_CD_LOADED | T_34W515_STATUS_1_LOMASK_PLAYING))
            {
              if(receivedchars[receivecnt] == T_34W515_CMD_STATUS_END)
              {
                if(receivedchars[receivecnt - 1] == T_34W515_CMD_STATUS_END) receivesize = receivecnt;
                else if((receivedchars[receivecnt - 1] != T_34W515_CMD_STATUS_END) && receivecnt == receivesize) receivesize++;
              }
            }
            else
            {
              if(receivedchars[T_34W515_STATUS_2_BYTE_CDID_1] == T_34W515_CDID_1_NORMAL_CD) receivesize = T_34W515_SIZE_NORMAL_CD - 1;
              else receivesize = T_34W515_SIZE_MP3_CD - 1;
            }
          }
          break;
        }
        case T_34W515_CMD_DISKINFO:
        {
          receivesize = T_34W515_SIZE_DISKINFO - 1;
          break;
        }
        case T_34W515_CMD_METADATA:
        {
          if(receivecnt > 0)
          {
            if(receivedchars[T_34W515_STATUS_1_BYTE] != T_34W515_METADATA_DATA_EXIST) receivesize = T_34W515_SIZE_METADATA_NOK - 1;
            else receivesize = T_34W515_SIZE_METADATA_OK - 1;
          }

          if(receivecnt >= T_34W515_METADATA_BYTE_9_CHAR_0)
          {
            if(receivecnt == T_34W515_METADATA_BYTE_9_CHAR_0) Serial.printf("<");
            Serial.printf("%c", receivedchars[receivecnt]);
            if(receivecnt == receivesize) Serial.printf("\n");
          }
          if(receivesize == T_34W515_SIZE_METADATA_NOK - 1)
          {
            Serial.printf("<NO METADATA\n");
          }
          break;
        }
        case T_34W515_CMD_UNKNOWN_1:
        {
          receivesize = T_34W515_SIZE_UNKNOWN_1ST_MSG - 1;
          break;
        }
        case T_34W515_CMD_UNKNOWN_2:
        {
          receivesize = T_34W515_SIZE_UNKNOWN_2ND_MSG - 1;
          break;
        }
        case T_34W515_CMD_DISK_STRUCTURE:
        {
          receivesize = T_34W515_SIZE_DISK_STRUCTURE - 1;
          break;
        }
        case T_34W515_CMD_ERROR:
        {
          receivesize = T_34W515_SIZE_UNKNOWN_1 - 1;
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
#endif

//debug spi
#ifdef DEBUG_515
bool MCDEmu_master_34W515(void)
{
  bool error;

  error = MCDEmu_master_34W515_tx();
  if(error == false)
  error = MCDEmu_master_34W515_rx();

  return error;
}
#endif

