#include "cd34w539.h"

const uint8_t initT_34W539[] = R_34W539_UNKNOWN_1ST_MSG;
const uint8_t diskInfoT_34W539[] = R_34W539_DISKINFO_MSG;
const uint8_t previousTrackT_34W539[] = R_34W539_PREVIOUS_TRACK_MSG;
const uint8_t metaDirNameT_34W539[] = R_34W539_METADATA_DIRECTORY_NAME_MSG(R_34W539_DIRECTORY_1);
const uint8_t metaTrackNameT_34W539[] = R_34W539_METADATA_TRACK_NAME_MSG(0, R_34W539_DIRECTORY_1, 1);
const uint8_t metaArtNameT_34W539[] = R_34W539_METADATA_ARTIST_NAME_MSG(0, R_34W539_DIRECTORY_1, 1);
const uint8_t metaFileNameT_34W539[] = R_34W539_METADATA_FILE_NAME_MSG(0, R_34W539_DIRECTORY_1, 1);
const uint8_t diskStructureT_34W539[] = R_34W539_DISKSTRUCTURE_MSG;
const uint8_t folderStructureT_34W539[] = R_34W539_FOLDERSTRUCTURE_MSG(1);
const uint8_t playTrackT_34W539[] = R_34W539_PLAY_MSG;
const uint8_t stopTrackT_34W539[] = R_34W539_STOP_MSG;
const uint8_t pauseTrackT_34W539[] = R_34W539_PAUSE_MSG;
const uint8_t nextTrackT_34W539[] = R_34W539_NEXT_TRACK_MSG;
const uint8_t nextDirectoryT_34W539[] = R_34W539_DIRECTORY_SET_NEXT_MSG;
const uint8_t previousDirectoryT_34W539[] = R_34W539_DIRECTORY_SET_PREVIOUS_MSG;
const uint8_t ejectDiskT_34W539[] = R_34W539_EJECT_MSG;
const uint8_t randomEnableT_34W539[] = R_34W539_RANDOM_ENABLE_MSG;
const uint8_t randomDisableT_34W539[] = R_34W539_RANDOM_DISABLE_MSG;
const uint8_t fastForwardT_34W539[] = R_34W539_FAST_FORWARD_MSG;
const uint8_t rewindT_34W539[] = R_34W539_REWIND_MSG;

const uint8_t sizeofinitT_34W539 = sizeof(initT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofdiskInfoT_34W539 = sizeof(diskInfoT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofdiskStructureT_34W539 = sizeof(diskStructureT_34W539) / sizeof(const uint8_t);
const uint8_t sizeoffolderStructureT_34W539 = sizeof(folderStructureT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofpreviousTrackT_34W539 = sizeof(previousTrackT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofmetaDirNameT_34W539 = sizeof(metaDirNameT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofejectDiskT_34W539 = sizeof(ejectDiskT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofrandomEnableT_34W539 = sizeof(randomEnableT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofrandomDisableT_34W539 = sizeof(randomDisableT_34W539) / sizeof(const uint8_t);
const uint8_t sizeoffastForwardT_34W539 = sizeof(fastForwardT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofrewindT_34W539 = sizeof(rewindT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofpreviousDirectoryT_34W539 = sizeof(previousDirectoryT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofnextDirectoryT_34W539 = sizeof(nextDirectoryT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofnextTrackT_34W539 = sizeof(nextTrackT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofpauseTrackT_34W539 = sizeof(pauseTrackT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofstopTrackT_34W539 = sizeof(stopTrackT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofmetaFileNameT_34W539 = sizeof(metaFileNameT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofplayTrackT_34W539 = sizeof(playTrackT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofmetaArtNameT_34W539 = sizeof(metaArtNameT_34W539) / sizeof(const uint8_t);
const uint8_t sizeofmetaTrackNameT_34W539 = sizeof(metaTrackNameT_34W539) / sizeof(const uint8_t);

cmdtx34w539_s tx34w539 = {false};

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


/**********************************************
(4 bis) 34W515 Master protocol emulator
**********************************************/
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

bool MCDEmu_master_34W539_status_update(uint8_t *receivedchars)
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
  uint8_t receivechar;
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

    receivechar = 0;

    error = digitalSWSPITransfer(R_34W539_ACK, &receivechar);

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

