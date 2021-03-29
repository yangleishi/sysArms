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

//sys config parame
struct configItem
{
  //4 ecoder
  float Encoder_X_Calibration;
  float Encoder_Y_Calibration;
  float Encoder_Z_Calibration;
  float Encoder_W_Calibration;

  float Angle_Sensor_Calibration;

  float Tension_Max_Value;
};

static configItem mParames[DEF_SYS_ARMS_NUMS] = {0};

namespace INTERACTIONER {

static char printfC[100] = {0};
pthread_mutex_t *mPrintQueueMutex = NULL;

static int initServer(BASE::ARMS_THREAD_INFO *pTModule);
static void setFdTimeout(int sockfd, const int mSec, const int mUsec);

static void printLoger(const char* sLog);

//config file
static int readConfig(configItem  * mParame, const int mPNum);
static int writeConfig(configItem * mParame, const int mPNum);
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
  pTModule->mSerAddr.sin_port = htons(pTModule->mSerPort);
  pTModule->mSerAddr.sin_addr.s_addr = inet_addr(pTModule->mIpV4Str);

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


static int readConfig(configItem * mParame, const int mPNum)
{
  int iRet = 0;
  configItem * mTParame = mParame;
  FILE *pFile = fopen(CONF::MN_INTERACTION_CONF_FILE, "r");

  if(pFile == NULL)
  {
    if((pFile = fopen(CONF::MN_INTERACTION_CONF_FILE, "w")) == NULL)
    {
        LOGER::PrintfLog(BASE::S_APP_LOGER, "--------------\n");
        return  -1;
    }

    for (int i=0; i<mPNum; i++)
    {
      memset((char*)mTParame, 0, sizeof(configItem));
      fprintf(pFile, "%f %f %f %f %f %f\n",
                      mTParame->Encoder_X_Calibration,
                      mTParame->Encoder_Y_Calibration,
                      mTParame->Encoder_Z_Calibration,
                      mTParame->Encoder_W_Calibration,
                      mTParame->Angle_Sensor_Calibration,
                      mTParame->Tension_Max_Value);
      mTParame++;
    }

    fclose(pFile);
    return 0;
  }

  for (int i=0; i<mPNum; i++)
  {

    fscanf(pFile, "%f %f %f %f %f %f\n",
                    &(mTParame->Encoder_X_Calibration),
                    &(mTParame->Encoder_Y_Calibration),
                    &(mTParame->Encoder_Z_Calibration),
                    &(mTParame->Encoder_W_Calibration),
                    &(mTParame->Angle_Sensor_Calibration),
                    &(mTParame->Tension_Max_Value));

    mTParame++;
  }

  fclose(pFile);
  return iRet;
}

static int writeConfig(configItem * mParame, const int mPNum)
{
  int iRet = 0;
  FILE *pFile = fopen(CONF::MN_INTERACTION_CONF_FILE, "w");

  if(pFile == NULL)
    return -1;

  configItem * mTParame = mParame;
  for (int i=0; i<mPNum; i++)
  {
    fprintf(pFile, "%f %f %f %f %f %f\n",
                   mTParame->Encoder_X_Calibration,
                   mTParame->Encoder_Y_Calibration,
                   mTParame->Encoder_Z_Calibration,
                   mTParame->Encoder_W_Calibration,
                   mTParame->Angle_Sensor_Calibration,
                   mTParame->Tension_Max_Value);

    mTParame++;
  }

  fclose(pFile);
  return iRet;
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

  readConfig(mParames, DEF_SYS_ARMS_NUMS);

  if(initServer(pTModule) != 0)
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "interaction  bind server ip failed, check network again !");
    moduleEndUp(pTModule);
    pTModule->mWorking = false;
    return 0;
  }

  socklen_t mun = sizeof(pTModule->mPeerAddr);

  BASE::PRINT_STR mLog;
  //running state
  pTModule->mState = BASE::M_STATE_RUN;

  LOGER::PrintfLog(BASE::S_APP_LOGER, "interaction running!");
  while(pTModule->mWorking)
  {
    //rec UDP************TODU
    //int size = recvfrom(pTModule->mSocket , (char*)&(pTModule->mRecMsg), sizeof(BASE::ARMS_R_MSG), 0, (sockaddr*)&(pTModule->mPeerAddr), &mun);
    //TUDO*****
    //lArmsStateCode = (size != sizeof(BASE::ARMS_R_MSG)) ? BASE::ST_SYS_REC_ERROR : pTModule->mRecMsg.mSysState;
    sleep(1);
    //TODU
    switch (pTModule->mState)
    {
      case BASE::M_STATE_INIT:
      {
        LOGER::PrintfLog(BASE::S_APP_LOGER, "test log hear");
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

  }

  moduleEndUp(pTModule);
}

} //namespace
