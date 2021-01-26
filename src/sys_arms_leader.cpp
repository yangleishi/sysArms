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



#include "sys_arms_leader.hpp"
#include "sys_arms_defs.h"
namespace LEADER {

static int initServer(BASE::ARMS_THREAD_INFO *pTModule);
static void setFdNonblocking(int sockfd);
static void setFdTimeout(int sockfd, const int mSec, const int mUsec);

static int moduleEndUp(BASE::ARMS_THREAD_INFO *pTModule);

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
  setFdTimeout(pTModule->mSocket, 0, CONF::SERVER_UDP_TIMEOUT);

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
    return 0;
  }

  struct timespec  pfTime1, pfTime2;
  int64_t diff;

  socklen_t mun = sizeof(pTModule->mPeerAddr);

  BASE::ARMS_MSG mRec;

  int lossF = 0;
  uint16_t  cRc = 0;
  clock_gettime(CLOCK_MONOTONIC, &pfTime1);

  static int64_t maxDiff = 0;

  //running state
  while(pTModule->mWorking)
  {
    //lock,wait signal
    pthread_mutex_lock(&pTModule->mArmsMsgMutex);

    pthread_cond_wait(&pTModule->mArmsMsgReady, &pTModule->mArmsMsgMutex);

    pthread_mutex_unlock(&pTModule->mArmsMsgMutex);

    clock_gettime(CLOCK_MONOTONIC, &pfTime2);

    diff = calcdiff_ns(pfTime2, pTModule->startTime);
    diff /= 1000;
    if(diff > maxDiff)
        maxDiff = diff;
    //printf("** diff:%d   maxdiff:%d\n", diff, maxDiff);


    //rec UDP
    int size = recvfrom(pTModule->mSocket , (char*)&mRec, sizeof(BASE::ARMS_MSG), 0, (sockaddr*)&(pTModule->mPeerAddr), &mun);

    if(size > 0)
    {
      if(size != sizeof(BASE::ARMS_MSG))
      {
         lossF++;
         printf("*******************************size***********: %d", size);
         continue;
      }

      if(cRc != mRec.mCrcCode && mRec.mCrcCode != 0)
      {
          lossF++;
          cRc = mRec.mCrcCode;
      }
      printf("size:%d, say: %d  loss:%d\n", size, mRec.mCrcCode,lossF);
      sendto(pTModule->mSocket, &mRec, sizeof(BASE::ARMS_MSG), 0, (struct sockaddr *)&(pTModule->mPeerAddr),sizeof(pTModule->mPeerAddr));
      cRc++;
    }
    else {
      //printf("timeout size:%d", size);
    }
  }

  moduleEndUp(pTModule);
}

} //namespace
