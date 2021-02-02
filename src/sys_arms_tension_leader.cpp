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


namespace TENSIONLEADER {


static int  initServer(BASE::ARMS_THREAD_INFO *pTModule);
static void setFdTimeout(int sockfd, const int mSec, const int mUsec);
static int moduleEndUp(BASE::ARMS_THREAD_INFO *pTModule);

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
  setFdTimeout(pTModule->mSocket, 0, CONF::SERVER_UDP_TENSION_TIMEOUT);

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

////////////////////////////////////////////////////////////////////////////////
///////external interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
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
    return 0;
  }

  socklen_t mun = sizeof(pTModule->mPeerAddr);

  //motors data
  uint32_t     lArmsStateCode;

  BASE::ARMS_TENSIONS_MSG mRecMsg;
  //running state
  while(pTModule->mWorking)
  {
    //rec UDP
    int size = recvfrom(pTModule->mSocket , (char*)&(mRecMsg), sizeof(BASE::ARMS_TENSIONS_MSG), 0, (sockaddr*)&(pTModule->mPeerAddr), &mun);

    //TUDO*****
    if(size != sizeof(BASE::ARMS_TENSIONS_MSG))
      continue;

    //rec tension data

  }

  moduleEndUp(pTModule);
}

} //namespace
