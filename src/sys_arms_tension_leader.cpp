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

#include "sys_arms_tension_leader.hpp"
#include "sys_arms_defs.h"
#include "sys_arms_conf.hpp"
#include "sys_arms_loger.hpp"


namespace TENSIONLEADER {


BASE::TENSIONS_S_MSG    mSendTensionMsg;
BASE::TENSIONS_R_MSG    mRecTensionMsg;

static int  initServer(BASE::TENSIONS_THREAD_INFO *pTModule);
static void setFdTimeout(int sockfd, const int mSec, const int mUsec);
static int moduleEndUp(BASE::TENSIONS_THREAD_INFO *pTModule);

static int motorCmd(BASE::TENSIONS_THREAD_INFO *pTModule, uint8_t mCmd);
static int packageFrame(BASE::TENSIONS_S_MSG* pMsg, uint8_t mCmd);
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
    LOGER::PrintfLog("setsockopt failed:");
  }
}


static int initServer(BASE::TENSIONS_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;

  if((pTModule->mSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      LOGER::PrintfLog("socket creat Failed");
      return -1;
  }

  //setFdNonblocking(pTModule->mSocket);
  setFdTimeout(pTModule->mSocket, CONF::SERVER_UDP_TENSION_TIMEOUT_S, CONF::SERVER_UDP_TENSION_TIMEOUT_US);

  bzero(&(pTModule->mSerAddr), sizeof(pTModule->mSerAddr));
  pTModule->mSerAddr.sin_family = AF_INET;
  pTModule->mSerAddr.sin_port = htons(pTModule->mSerPort);
  pTModule->mSerAddr.sin_addr.s_addr = inet_addr(pTModule->mIpV4Str);

  iRet = bind(pTModule->mSocket, (struct sockaddr*)&(pTModule->mSerAddr), sizeof(pTModule->mSerAddr));

  if(iRet < 0)
    return iRet;

  return 0;
}


static int moduleEndUp(BASE::TENSIONS_THREAD_INFO *pTModule)
{
  //stop rec
  motorCmd(pTModule, 1);

  if(pTModule->mSocket >= 0)
  {
    close(pTModule->mSocket);
    pTModule->mSocket = -1;
  }
  LOGER::PrintfLog("tension leader endup");
  return 0;
}

///msg functions/////////////////
static int packageFrame(BASE::TENSIONS_S_MSG* pMsg, uint8_t mCmd)
{
  int32_t iRet = 0;
  if(pMsg == NULL)
  {
    return -1;
  }

  struct timespec now;

  pMsg->mFrameStart = 0x1ACF;
  pMsg->mIdentifier = 0xFF;
  pMsg->mApid = 0x04;
  pMsg->mType = 0x01;

  clock_gettime(CLOCK_MONOTONIC, &now);
  pMsg->mSysTime.mSysTimeS  = now.tv_sec;
  pMsg->mSysTime.mSysTimeUs = now.tv_nsec/1000;

  pMsg->mCmd = mCmd;
  pMsg->mCrcCode = 0;

  return iRet;
}


static int motorCmd(BASE::TENSIONS_THREAD_INFO *pTModule, uint8_t mCmd)
{
  int32_t iRet = 0;
  packageFrame(&mSendTensionMsg,  mCmd);
  iRet = sendto(pTModule->mSocket, &mSendTensionMsg, sizeof(BASE::TENSIONS_S_MSG), 0, (struct sockaddr *)&(pTModule->mPeerAddr), sizeof(pTModule->mPeerAddr));
  return  iRet;
}
////////////////////////////////////////////////////////////////////////////////
///////external interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void* threadEntry(void* pModule)
{
  BASE::TENSIONS_THREAD_INFO *pTModule =(BASE::TENSIONS_THREAD_INFO *) pModule;
  if(NULL == pTModule)
  {
    return 0;
  }

  if(initServer(pTModule) != 0)
  {
    LOGER::PrintfLog("%s  bind server ip failed, check network again !", pTModule->mThreadName);
    moduleEndUp(pTModule);
    return 0;
  }

  socklen_t mun = sizeof(pTModule->mPeerAddr);

  //motors data
  uint32_t     lArmsStateCode;

  BASE::TENSIONS_R_MSG mRecMsg;
  //running state
  LOGER::PrintfLog("%s running!",pTModule->mThreadName);

  uint8_t  lTensionStateCode = 0;
  uint8_t  bFirstRec = 1;

  while(pTModule->mWorking)
  {
    //rec UDP
    int size = recvfrom(pTModule->mSocket , (char*)&(mRecMsg), sizeof(BASE::TENSIONS_R_MSG), 0, (sockaddr*)&(pTModule->mPeerAddr), &mun);

    //TUDO*****
    if(size != sizeof(BASE::TENSIONS_R_MSG))
    {
      LOGER::PrintfLog("tension thread rec overtime or error!");
      continue;
    }

    //rec tension data
    lTensionStateCode = mRecMsg.mStateCode;

    //error*******
    if((lTensionStateCode%2) != 0)
    {
      LOGER::PrintfLog("hard error. state code:%d", mRecMsg.mTensionsStateCode);
      continue;
    }

    //tension data
    if(bFirstRec)
    {
      //start send msg. 0 :start.  1: stop
      motorCmd(pTModule, 0);
      bFirstRec = 0;
      continue;
    }

    //copy tensins data to supr
    pTModule->mNowTensionMsg[mRecMsg.mIdentifier].mTensions = mRecMsg.mTensions;
    pTModule->mNowTensionMsg[mRecMsg.mIdentifier].iNewTensions = true;

  }

  moduleEndUp(pTModule);
}

} //namespace
