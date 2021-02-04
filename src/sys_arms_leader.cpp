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
#include <errno.h>



#include "sys_arms_leader.hpp"
#include "sys_arms_defs.h"
namespace LEADER {

static int  initServer(BASE::ARMS_THREAD_INFO *pTModule);
static void setFdNonblocking(int sockfd);
static void setFdTimeout(int sockfd, const int mSec, const int mUsec);

static int moduleEndUp(BASE::ARMS_THREAD_INFO *pTModule);

///msg functions/////////////////
static int packageFrame(BASE::ARMS_S_MSG* pMsg, uint8_t mCtrl, BASE::MOTORS &mMotors, uint16_t mCrcCode);

//pTmodule cmd or data send to client
static int motorMoveCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors, uint8_t mCtrl, uint16_t mCrcCode);
////////////////////////////////////////////////////////////////////////////////
///////internal interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//设置非阻塞
static void setFdNonblocking(int sockfd)
{
    int flag = fcntl(sockfd, F_GETFL, 0);
    if (flag < 0) {
        printf("fcntl F_GETFL fail");
        return;
    }
    if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) {
        printf("fcntl F_SETFL fail");
    }
}


static void setFdTimeout(int sockfd, const int mSec, const int mUsec)
{
  struct timeval timeout;
  timeout.tv_sec = mSec;//秒
  timeout.tv_usec = mUsec;//微秒
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1)
  {
    printf("setsockopt failed:");
  }
}

static int initServer(BASE::ARMS_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;

  if((pTModule->mSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      printf("socket creat Failed");
      return -1;
  }

  //setFdNonblocking(pTModule->mSocket);
  setFdTimeout(pTModule->mSocket, CONF::SERVER_UDP_TIMEOUT_S, CONF::SERVER_UDP_TIMEOUT_US);

  bzero(&(pTModule->mPeerAddr), sizeof(pTModule->mPeerAddr));

  bzero(&(pTModule->mSerAddr), sizeof(pTModule->mSerAddr));
  pTModule->mSerAddr.sin_family = AF_INET;
  pTModule->mSerAddr.sin_port = htons(pTModule->mSerPort);
  pTModule->mSerAddr.sin_addr.s_addr = inet_addr(pTModule->mIpV4Str);

  iRet = bind(pTModule->mSocket, (struct sockaddr*)&(pTModule->mSerAddr), sizeof(pTModule->mSerAddr));

  if(iRet < 0)
  {
    printf("server :%d bind faild", pthread_self());
    return iRet;
  }

  return 0;
}

static int moduleEndUp(BASE::ARMS_THREAD_INFO *pTModule)
{
  if(pTModule->mSocket >= 0)
  {
    close(pTModule->mSocket);
    pTModule->mSocket = -1;
  }
  printf("endup  ");
  return 0;
}


///msg functions/////////////////
static int packageFrame(BASE::ARMS_S_MSG* pMsg, uint8_t mCtrl, BASE::MOTORS &mMotors, uint16_t mCrcCode)
{
  int32_t iRet = 0;
  if(pMsg == NULL)
  {
    return -1;
  }

  struct timespec now;

  pMsg->mIdentifier = 0x88;
  pMsg->mCtrl  = mCtrl;
  memcpy((char*)&(pMsg->mMotors), (char*)&mMotors, sizeof(mMotors));
  pMsg->mSysState = 0;

  clock_gettime(CLOCK_MONOTONIC, &now);
  pMsg->mSysTime.mSysTimeS  = now.tv_sec;
  pMsg->mSysTime.mSysTimeUs = now.tv_nsec/1000;

  pMsg->mDataLength = sizeof(BASE::MOTORS);

  pMsg->mCrcCode = mCrcCode;

  return iRet;
}

static int motorMoveCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors, uint8_t mCtrl, uint16_t mCrcCode)
{
  int32_t iRet = 0;
  packageFrame(&pTModule->mSendMsg, mCtrl, mMotors, mCrcCode);
  iRet = sendto(pTModule->mSocket, &pTModule->mSendMsg, sizeof(BASE::ARMS_S_MSG), 0, (struct sockaddr *)&(pTModule->mPeerAddr), sizeof(pTModule->mPeerAddr));
  return  iRet;
}
////////////////////////////////////////////////////////////////////////////////
///////external interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
static inline int64_t calcdiff_ns(struct timespec t1, struct timespec t2)
{
    int64_t diff;
    diff = NSEC_PER_SEC * (int64_t)((int) t1.tv_sec - (int) t2.tv_sec);
    diff += ((int) t1.tv_nsec - (int) t2.tv_nsec);
    return diff;
}
void* threadEntry(void* pModule)
{
  BASE::ARMS_THREAD_INFO *pTModule =(BASE::ARMS_THREAD_INFO *) pModule;
  if(NULL == pTModule)
  {
    return 0;
  }

  if(initServer(pTModule) != 0)
  {
    printf("bind server ip failed, check network again !\n");
    moduleEndUp(pTModule);
    pTModule->mWorking = false;
    return 0;
  }

  struct timespec  pfTime1, pfTime2;
  int64_t diff;

  socklen_t mun = sizeof(pTModule->mPeerAddr);

  //BASE::ARMS_S_MSG mRec, mSendMsg;
  //BASE::MOTORS   mMotors;

  int lossF = 0;
  uint16_t  cRc = 0;
  clock_gettime(CLOCK_MONOTONIC, &pfTime1);

  static int64_t maxDiff = 0;

  //uint32_t mArmsState = -1;
  uint16_t mCrc = 0;
  //motors data
  BASE::MOTORS lMotors;
  uint32_t     lArmsStateCode;

  //running state
  while(pTModule->mWorking)
  {
    //lock,wait signal
    pthread_mutex_lock(&pTModule->mArmsMsgMutex);

    pthread_cond_wait(&pTModule->mArmsMsgReady, &pTModule->mArmsMsgMutex);

    pthread_mutex_unlock(&pTModule->mArmsMsgMutex);

    //rec UDP
    int size = recvfrom(pTModule->mSocket , (char*)&(pTModule->mRecMsg), sizeof(BASE::ARMS_R_MSG), 0, (sockaddr*)&(pTModule->mPeerAddr), &mun);
    //TUDO*****
    lArmsStateCode = (size != sizeof(BASE::ARMS_R_MSG)) ? BASE::ST_SYS_REC_ERROR : pTModule->mRecMsg.mSysState;

    //send
    switch (pTModule->mState)
    {
      case BASE::M_STATE_INIT:
      {
        if(lArmsStateCode == BASE::ST_SYS_POWERON_OK)
        {
          //TUDO send 0 let arms wait
          memset((char*)&lMotors, 0, sizeof(lMotors));
          motorMoveCmd(pTModule, lMotors, BASE::CT_SYS_FIRE, mCrc);
          pTModule->mState = BASE::M_STATE_RUN;
          break;
        }
        //power on
        int iRet =  motorMoveCmd(pTModule, lMotors, BASE::CT_SYS_POWERON, mCrc);

        break;
      }
      case BASE::M_STATE_RUN:
      {
        //TUDO
        if(lArmsStateCode == BASE::ST_SYS_FIRE_OK)
        {
          //TUDO cal******

          motorMoveCmd(pTModule, lMotors, BASE::CT_SYS_FIRE, mCrc);
        }
        else
        {
          printf("error code:%d\n", lArmsStateCode);
          motorMoveCmd(pTModule, lMotors, BASE::CT_SYS_UNFIRE, mCrc);
          pTModule->mState = BASE::M_STATE_STOP;
        }
        break;
      }
      case BASE::M_STATE_STOP:
      {
        motorMoveCmd(pTModule, lMotors, BASE::CT_SYS_UNFIRE, mCrc);

        //stop ok and recMsg error,then endup.
        if(lArmsStateCode == BASE::ST_SYS_STOP_OK || lArmsStateCode == BASE::ST_SYS_REC_ERROR)
        {
          pTModule->mWorking = false;
          break;
        }
        break;
      }
      default:
      {
        break;
      }
    }

    mCrc++;
  }

  moduleEndUp(pTModule);
}

} //namespace
