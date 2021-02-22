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


namespace INTERACTIONER {


static int initServer(BASE::ARMS_THREAD_INFO *pTModule);
static void setFdTimeout(int sockfd, const int mSec, const int mUsec);

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



///////////////////////////////////
static int initServer(BASE::INTERACTION_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;

  if((pTModule->mSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      LOGER::PrintfLog("socket creat Failed");
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
    LOGER::PrintfLog("server :%d bind faild", pthread_self());
    return iRet;
  }

  return 0;
}


static int moduleEndUp(BASE::INTERACTION_THREAD_INFO *pTModule)
{
  if(pTModule->mSocket >= 0)
  {
    close(pTModule->mSocket);
    pTModule->mSocket = -1;
  }
  LOGER::PrintfLog("endup  ");
  return 0;
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

  if(initServer(pTModule) != 0)
  {
    LOGER::PrintfLog("bind server ip failed, check network again !\n");
    //printf("bind server ip failed, check network again !\n");
    moduleEndUp(pTModule);
    pTModule->mWorking = false;
    return 0;
  }

  BASE::PRINT_STR mLog;
  //running state
  pTModule->mState = BASE::M_STATE_RUN;

  LOGER::PrintfLog("%s running!",pTModule->mThreadName);
  while(pTModule->mWorking)
  {
    sleep(1);
  }

  moduleEndUp(pTModule);
}

} //namespace
