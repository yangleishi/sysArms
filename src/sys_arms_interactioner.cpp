/******************************************************************************
**
* Copyright (c)2021 SHI YANGLEI
* All Rights Reserved
*
*
******************************************************************************/

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "sys_arms_loger.hpp"
#include "sys_arms_defs.h"
#include "sys_arms_conf.hpp"
#include "sys_arms_interactioner.hpp"
#include "sys_arms_daemon.hpp"


static BASE::ReadConfData mParames[DEF_SYS_ARMS_NUMS] = {0};

namespace INTERACTIONER {

static char printfC[100] = {0};
pthread_mutex_t *mPrintQueueMutex = NULL;


static int initServer(BASE::ARMS_THREAD_INFO *pTModule);
static void setFdTimeout(int sockfd, const int mSec, const int mUsec);

static void printLoger(const char* sLog);
static void sendMsgToUpper(BASE::INTERACTION_THREAD_INFO *pTModule, const uint16_t StatusWord, const uint16_t StatusCode, const char* pData, int dataSize);

static void changeState(BASE::INTERACTION_THREAD_INFO *pTModule, uint16_t mCmd);

//config file
static int readConfig(BASE::ReadConfData  * mParame);
static int writeConfig(BASE::ReadConfData * mParame, const int mPNum);
//config handle move
static int sendMsgToSupr(BASE::INTERACTION_THREAD_INFO *pTModule, int pNewRecType);

static int sendCycLiftDatas(BASE::INTERACTION_THREAD_INFO *pTModule, uint16_t mCmdCyc);

////////////////////////////////////////////////////////////////////////////////
///////internal interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static void setFdTimeout(int sockfd, const int mSec, const int mUsec)
{
  struct timeval timeout;
  timeout.tv_sec = mSec;//秒
  timeout.tv_usec = mUsec;//微秒
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1)
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "interaction set fd error\n");
  }
}


static int initServer(BASE::INTERACTION_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;

  if((pTModule->mSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      LOGER::PrintfLog(BASE::S_APP_LOGER, "socket creat Failed");
      return -1;
  }

  //setFdNonblocking(pTModule->mSocket);
  setFdTimeout(pTModule->mSocket, CONF::SERVER_UDP_INTERACTION_TIMEOUT_S, CONF::SERVER_UDP_INTERACTION_TIMEOUT_US);

  bzero(&(pTModule->mPeerAddr), sizeof(pTModule->mPeerAddr));

  bzero(&(pTModule->mSerAddr), sizeof(pTModule->mSerAddr));
  pTModule->mSerAddr.sin_family = AF_INET;
  pTModule->mSerAddr.sin_port = htons(pTModule->mMyPort);
  pTModule->mSerAddr.sin_addr.s_addr = inet_addr(pTModule->mMyIpV4Str);

  iRet = bind(pTModule->mSocket, (struct sockaddr*)&(pTModule->mSerAddr), sizeof(pTModule->mSerAddr));

  if(iRet < 0)
    return iRet;

  return 0;
}


static int moduleEndUp(BASE::INTERACTION_THREAD_INFO *pTModule)
{
  if(pTModule->mSocket >= 0)
  {
    close(pTModule->mSocket);
    pTModule->mSocket = -1;
  }
  LOGER::PrintfLog(BASE::S_APP_LOGER, "interaction endup \n");
  return 0;
}


static int readConfig(BASE::ReadConfData * mParame)
{
  int iRet = 0;
  BASE::ReadConfData * mTParame = mParame;
  FILE *pFile = fopen(CONF::MN_INTERACTION_CONF_FILE, "r");

  if(pFile == NULL)
  {
    if((pFile = fopen(CONF::MN_INTERACTION_CONF_FILE, "w")) == NULL)
    {
      LOGER::PrintfLog(BASE::S_APP_LOGER, "--------------\n");
      return  -1;
    }

    for (int i=0; i<DEF_SYS_ARMS_NUMS; i++)
    {
      fprintf(pFile, "%f %f %f %f %f %f\n", CONF::IN_MAX_TENSION[i], CONF::IN_OFFSET_X[i], CONF::IN_OFFSET_Y[i], CONF::IN_OFFSET_Z[i], CONF::IN_OFFSET_W[i], CONF::IN_OFFSET_ANGLE[i]);
      mTParame->mConfSaveWeight = CONF::IN_MAX_TENSION[i];
      mTParame->mConfSaveEncoderX = CONF::IN_OFFSET_X[i];
      mTParame->mConfSaveEncoderY = CONF::IN_OFFSET_Y[i];
      mTParame->mConfSaveEncoderZ = CONF::IN_OFFSET_Z[i];
      mTParame->mConfSaveEncoderP = CONF::IN_OFFSET_W[i];
      mTParame->mConfSaveEncoderT = CONF::IN_OFFSET_ANGLE[i];
      mTParame++;
    }
    fclose(pFile);
    return 0;
  }

  memset((char*)mTParame, 0, sizeof(BASE::ReadConfData)*DEF_SYS_ARMS_NUMS);
  for (int i=0; i<DEF_SYS_ARMS_NUMS; i++)
  {

    fscanf(pFile, "%f %f %f %f %f %f\n",
                    &(mTParame->mConfSaveWeight),
                    &(mTParame->mConfSaveEncoderX),
                    &(mTParame->mConfSaveEncoderY),
                    &(mTParame->mConfSaveEncoderZ),
                    &(mTParame->mConfSaveEncoderP),
                    &(mTParame->mConfSaveEncoderT)
                    );

    //printf("%f %f %f %f %f %f\n",mTParame->mConfSaveWeight, mTParame->mConfSaveEncoderX, mTParame->mConfSaveEncoderY,
    //                             mTParame->mConfSaveEncoderZ, mTParame->mConfSaveEncoderP, mTParame->mConfSaveEncoderT);
    mTParame++;
  }

  fclose(pFile);
  return iRet;
}

static int writeConfig(BASE::SaveConfData * sParame, const int mPNum)
{
  int iRet = 0;
  FILE *pFile = fopen(CONF::MN_INTERACTION_CONF_FILE, "rw+");

  if(pFile == NULL)
    return -1;

  readConfig(mParames);
  for (int i=0; i<DEF_SYS_ARMS_NUMS;i++)
  {
      if(sParame[i].mIsValid == 1)
      {
          mParames[i].mConfSaveWeight = sParame[i].mConfSaveWeight;
          mParames[i].mConfSaveEncoderX = sParame[i].mConfSaveEncoderX;
          mParames[i].mConfSaveEncoderY = sParame[i].mConfSaveEncoderY;
          mParames[i].mConfSaveEncoderZ = sParame[i].mConfSaveEncoderZ;
          mParames[i].mConfSaveEncoderP = sParame[i].mConfSaveEncoderP;
          mParames[i].mConfSaveEncoderT = sParame[i].mConfSaveEncoderT;
      }
  }

  for (int i=0; i<mPNum; i++)
  {
    fprintf(pFile, "%f %f %f %f %f %f\n",
                   mParames[i].mConfSaveWeight,
                   mParames[i].mConfSaveEncoderX,
                   mParames[i].mConfSaveEncoderY,
                   mParames[i].mConfSaveEncoderZ,
                   mParames[i].mConfSaveEncoderP,
                   mParames[i].mConfSaveEncoderT);
    printf("%f %f %f %f %f %f  %d\n",mParames[i].mConfSaveWeight, mParames[i].mConfSaveEncoderX, mParames[i].mConfSaveEncoderY,
                                     mParames[i].mConfSaveEncoderZ, mParames[i].mConfSaveEncoderP, mParames[i].mConfSaveEncoderT, mParames[i].mIsValid);
  }

  fclose(pFile);
  return iRet;
}

static void sendMsgToUpper(BASE::INTERACTION_THREAD_INFO *pTModule, const uint16_t StatusWord, const uint16_t StatusCode, const char* pData, int dataSize)
{
    BASE::MArmsUpData mSendMsg;
    mSendMsg.StatusWord = StatusWord;
    mSendMsg.StatusCode = StatusCode;
    if(dataSize > 0 && dataSize < MSG_UP_DATA_MAX)
        memcpy(mSendMsg.Datas, pData, dataSize);

    sendto(pTModule->mSocket, (char*)&mSendMsg, sizeof(BASE::MArmsUpData), 0, (sockaddr*)&pTModule->mPeerAddr, sizeof(pTModule->mPeerAddr));
}

static void changeState(BASE::INTERACTION_THREAD_INFO *pTModule, uint16_t mCmd)
{
    /*
    switch (pTModule->mState)
    {
      case BASE::M_STATE_INIT:
      {
        if((mCmd == BASE::CMD_SAVE_CONF) || (mCmd == BASE::CMD_READ_CONF))
        {
            pTModule->mNewState = BASE::M_STATE_CONF;
            pTModule->mIsStataChange = true;
        }
        break;
      }
      case BASE::M_STATE_CONF:
      {
        if(mCmd == BASE::CMD_RUN_START)
        {
            pTModule->mNewState = BASE::M_STATE_RUN;
            pTModule->mIsStataChange = true;
        }
        break;
      }
      case BASE::M_STATE_RUN:
      {
        break;
      }
      case BASE::M_STATE_STOP:
      {
        break;
      }
      default:
      {
        break;
      }
    }
    */
    ;
}

static int sendMsgToSupr(BASE::INTERACTION_THREAD_INFO *pTModule, int pNewRecType)
{
    LOGER::PrintfLog(BASE::S_APP_LOGER, "interaction CMD %d", pNewRecType);
    pthread_mutex_lock(&pTModule->mInterToSuprDatas->mInteractionRecMutex);
    pTModule->mInterToSuprDatas->mIsNewRec = pNewRecType;
    memcpy((char*)&pTModule->mInterToSuprDatas->mRecMsg, (char*)&pTModule->mRecMsg, sizeof(pTModule->mRecMsg));
    pthread_mutex_unlock(&pTModule->mInterToSuprDatas->mInteractionRecMutex);
}


static int sendCycLiftDatas(BASE::INTERACTION_THREAD_INFO *pTModule, uint16_t mCmdCyc)
{
  switch (mCmdCyc)
  {
    case BASE::CMD_CYC_READ_LIFT_DATAS:
    {
      BASE::ReadLiftSigalNowData *mSendDatas = (BASE::ReadLiftSigalNowData *)malloc(sizeof(BASE::ReadLiftSigalNowData)*DEF_SYS_USE_ARMS_NUMS);
      pthread_mutex_lock(&pTModule->mSuprDatasToInterasction->mArmsNowDatasMutex);
      memcpy((char*)mSendDatas, (char*)pTModule->mSuprDatasToInterasction->mReadLiftNowDatas, sizeof(BASE::ReadLiftSigalNowData)*DEF_SYS_USE_ARMS_NUMS);
      pthread_mutex_unlock(&pTModule->mSuprDatasToInterasction->mArmsNowDatasMutex);
      printf("read lift\n");

      sendMsgToUpper(pTModule, BASE::CMD_ACK_READ_LIFT_DATAS, 0, (char*)mSendDatas, sizeof(BASE::ReadLiftSigalNowData)*DEF_SYS_USE_ARMS_NUMS);
      free(mSendDatas);
      break;
    }
    case BASE::CMD_CYC_READ_SYS_DELAYED:
    {
      BASE::ReadLiftHzData *mSendDatas = (BASE::ReadLiftHzData *)malloc(sizeof(BASE::ReadLiftHzData)*DEF_SYS_ARMS_NUMS);
      pthread_mutex_lock(&pTModule->mSuprDatasToInterasction->mArmsNowDatasMutex);
      memcpy((char*)mSendDatas, (char*)pTModule->mSuprDatasToInterasction->mReadLiftHzDatas, sizeof(BASE::ReadLiftHzData)*DEF_SYS_ARMS_NUMS);
      pthread_mutex_unlock(&pTModule->mSuprDatasToInterasction->mArmsNowDatasMutex);

      printf("delayed\n");
      sendMsgToUpper(pTModule, BASE::CMD_ACK_READ_DELAYED_DATAS, 0, (char*)mSendDatas, sizeof(BASE::ReadLiftHzData)*DEF_SYS_ARMS_NUMS);
      free(mSendDatas);
      break;
    }
    default:
    {
      break;
    }
  }

}
////////////////////////////////////////////////////////////////////////////////
///////external interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void* threadEntry(void* pModule)
{
  BASE::INTERACTION_THREAD_INFO *pTModule =(BASE::INTERACTION_THREAD_INFO *) pModule;
  if(NULL == pTModule)
  {
    return 0;
  }
  BASE::hiSetCpuAffinity(pTModule->mCpuAffinity);

  mPrintQueueMutex = pTModule->mPrintQueueMutex;

  //read conf
  readConfig(mParames);

  if(initServer(pTModule) != 0)
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "interaction  bind server ip failed, check network again !");
    moduleEndUp(pTModule);
    pTModule->mWorking = false;
    return 0;
  }

  socklen_t mun = sizeof(pTModule->mPeerAddr);

  //running state
  pTModule->mState = BASE::M_STATE_CONF;

  LOGER::PrintfLog(BASE::S_APP_LOGER, "interaction running!");
  while(pTModule->mWorking)
  {
    //rec UDP************TODU
    int size = recvfrom(pTModule->mSocket , (char*)&(pTModule->mRecMsg), sizeof(BASE::MArmsDownData), 0, (sockaddr*)&(pTModule->mPeerAddr), &mun);
    //TODU
    switch (pTModule->mRecMsg.CmdIdentify)
    {
      case BASE::CMD_LINK:
      {
        //LOGER::PrintfLog(BASE::S_APP_LOGER, "test log hear");
        pTModule->mNewState = BASE::M_STATE_CONF;
        pTModule->mIsStataChange = true;
        pTModule->mState = BASE::M_STATE_CONF;

        printf("interaction CMD link\n");
        //TUDO  send msg to arms concl
        memset((char*)mParames, 0, sizeof(BASE::ReadConfData)*DEF_SYS_ARMS_NUMS);
        readConfig(mParames);
        sendMsgToUpper(pTModule, BASE::CMD_ACK_LINK_OK, 0, (char*)mParames, sizeof(BASE::ReadConfData)*DEF_SYS_ARMS_NUMS);
        break;
      }
      case BASE::CMD_UNLINK:
      {
        printf("interaction CMD unlink\n");
        //改变整个系统的运行状态到STOP
        pTModule->mNewState = BASE::M_STATE_STOP;
        pTModule->mIsStataChange = true;
        pTModule->mState = BASE::M_STATE_STOP;

        sendMsgToUpper(pTModule, BASE::CMD_ACK_UNLINK_OK, 0, 0, 0);
        break;
      }
      case BASE::CMD_SAVE_CONF:
      {
        //BASE::SaveConfData *mSavaConf = (BASE::SaveConfData *)pTModule->mRecMsg.Datas;
        writeConfig((BASE::SaveConfData *)pTModule->mRecMsg.Datas, DEF_SYS_ARMS_NUMS);
        printf("interaction CMD Sava conf\n");
        break;
      }
      case BASE::CMD_READ_CONF:
      {
        memset((char*)mParames, 0, sizeof(BASE::ReadConfData)*DEF_SYS_ARMS_NUMS);
        readConfig(mParames);
        printf("interaction CMD read conf\n");
        sendMsgToUpper(pTModule, BASE::CMD_ACK_READCONF_OK, 0, (char*)mParames, sizeof(BASE::ReadConfData)*DEF_SYS_ARMS_NUMS);
        break;
      }
      case BASE::CMD_HAND_MOVE_START:
      {
        if(pTModule->mState == BASE::M_STATE_CONF)
            sendMsgToSupr(pTModule, BASE::CMD_HAND_MOVE_START);
        break;
      }
      case BASE::CMD_CYC_READ_LIFT_DATAS:
      {
        if(pTModule->mState == BASE::M_STATE_CONF)
            sendCycLiftDatas(pTModule, BASE::CMD_CYC_READ_LIFT_DATAS);
        break;
      }
      case BASE::CMD_CYC_READ_SYS_DELAYED:
      {
        sendCycLiftDatas(pTModule, BASE::CMD_CYC_READ_SYS_DELAYED);
        break;
      }
      case BASE::CMD_HAND_MOVE_STOP:
      {
        if(pTModule->mState == BASE::M_STATE_CONF)
            sendMsgToSupr(pTModule, BASE::CMD_HAND_MOVE_STOP);
        break;
      }
      case BASE::CMD_ALL_MOVE_START:
      {
        if(pTModule->mState == BASE::M_STATE_CONF)
            sendMsgToSupr(pTModule, BASE::CMD_ALL_MOVE_START);
        printf("interaction CMD all move start\n");
        break;
      }
      case BASE::CMD_ALL_MOVE_STOP:
      {
        if(pTModule->mState == BASE::M_STATE_CONF)
            sendMsgToSupr(pTModule, BASE::CMD_ALL_MOVE_STOP);
        printf("interaction CMD all move stop\n");
        break;
      }
      case BASE::CMD_ALL_PULL_START:
      {
        if(pTModule->mState == BASE::M_STATE_CONF)
        {
            //tension turn kg
            BASE::PullLiftAllData *mAllPull = (BASE::PullLiftAllData *)pTModule->mRecMsg.Datas;
            for (int i=0; i<DEF_SYS_ARMS_NUMS; i++)
            {
                if(mAllPull[i].mIsValid)
                    mAllPull[i].mHandPull *= ((mParames[i].mConfSaveWeight)/100.0);
            }
            sendMsgToSupr(pTModule, BASE::CMD_ALL_PULL_START);
        }
        printf("interaction CMD all pull start\n");
        break;
      }
      case BASE::CMD_ALL_PULL_STOP:
      {
        if(pTModule->mState == BASE::M_STATE_CONF)
            sendMsgToSupr(pTModule, BASE::CMD_ALL_PULL_STOP);
        printf("interaction CMD all pull stop\n");
        break;
      }
      case BASE::CMD_RUN_START:
      {
        //改变整个系统的运行状态到RUN
        if(pTModule->mState == BASE::M_STATE_CONF)
        {
          pTModule->mNewState = BASE::M_STATE_RUN;
          pTModule->mIsStataChange = true;
          pTModule->mState = BASE::M_STATE_RUN;
        }

        //在RUN状态下修改，Start命令发送
        sendMsgToSupr(pTModule, BASE::CMD_RUN_START);
        printf("interaction CMD run start\n");
        break;
      }
      case BASE::CMD_CYC_READ_RUNNING_DATAS:
      {
        printf("interaction CMD run start\n");
        break;
      }
      case BASE::CMD_CYC_READ_SHOWDE_DATAS:
      {
        printf("interaction CMD run start\n");
        break;
      }
      case BASE::CMD_RUN_STOP:
      {
        sendMsgToSupr(pTModule, BASE::CMD_RUN_STOP);
        printf("all modules stop...\n");
        break;
      }
      case BASE::CMD_RUN_STOP_E:
      {
        if(pTModule->mState == BASE::M_STATE_RUN)
            sendMsgToSupr(pTModule, BASE::CMD_RUN_STOP);
        printf("interaction CMD run  stop E\n");
        break;
      }
      case BASE::CMD_QUIT:
      {
        sendMsgToSupr(pTModule, BASE::CMD_QUIT);
        printf("interaction CMD quit... \n");
        break;
      }
      default:
      {
        break;
      }
    }

    memset((char*)&(pTModule->mRecMsg), 0, sizeof(BASE::MArmsDownData));
  }

  moduleEndUp(pTModule);
}

} //namespace
