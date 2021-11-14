/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_tension_leader.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:拉立计模块，由于此系统中拉力计数据单独传输，因此此模块负责收发拉力计相关的消息。
消息通知其他模块。
*
********************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>

#include "sys_arms_tension_leader.hpp"
#include "sys_arms_defs.h"
#include "sys_arms_conf.hpp"
#include "sys_arms_loger.hpp"
#include "sys_arms_daemon.hpp"


namespace TENSIONLEADER {


BASE::TENSIONS_S_MSG    mSendTensionMsg;
BASE::TENSIONS_R_MSG    mRecTensionMsg;

static int  initServer(BASE::TENSIONS_THREAD_INFO *pTModule);
static void setFdTimeout(int sockfd, const int mSec, const int mUsec);
static int moduleEndUp(BASE::TENSIONS_THREAD_INFO *pTModule);

static int tensionCmd(BASE::TENSIONS_THREAD_INFO *pTModule, uint8_t mCmd);
static int packageFrame(BASE::TENSIONS_S_MSG* pMsg, uint8_t mCmd);
////////////////////////////////////////////////////////////////////////////////
///////internal interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
* 功能：设置fd套接字非阻塞模式下延时
* @param sockfd : sockfd是socket套接字
* @param mSec : mSec是阻塞延时 妙
* @param mUsec : mUsec是阻塞延时微妙
* @return Descriptions
******************************************************************************/
static void setFdTimeout(int sockfd, const int mSec, const int mUsec)
{
  struct timeval timeout;
  timeout.tv_sec = mSec;//秒
  timeout.tv_usec = mUsec;//微秒
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1)
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "setsockopt failed:");
  }
}

/******************************************************************************
* 功能：初始化函数，tension作为服务器端，一直接收拉力计信息
* @param pTModule : pTModule是线程信息
* @return Descriptions
******************************************************************************/
static int initServer(BASE::TENSIONS_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;

  if((pTModule->mSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      LOGER::PrintfLog(BASE::S_APP_LOGER, "socket creat Failed");
      return -1;
  }

  //setFdNonblocking(pTModule->mSocket);
  setFdTimeout(pTModule->mSocket, CONF::SERVER_UDP_TENSION_TIMEOUT_S, CONF::SERVER_UDP_TENSION_TIMEOUT_US);

  bzero(&(pTModule->mSerAddr), sizeof(pTModule->mSerAddr));
  pTModule->mSerAddr.sin_family = AF_INET;
  pTModule->mSerAddr.sin_port = htons(pTModule->mMyPort);
  pTModule->mSerAddr.sin_addr.s_addr = inet_addr(pTModule->mMyIpV4Str);

  iRet = bind(pTModule->mSocket, (struct sockaddr*)&(pTModule->mSerAddr), sizeof(pTModule->mSerAddr));

  if(iRet < 0)
    return iRet;

  return 0;
}

/******************************************************************************
* 功能：模块释放函数，线程结束时调用此函数
* @param pTModule : pTModule是线程信息
* @return Descriptions
******************************************************************************/
static int moduleEndUp(BASE::TENSIONS_THREAD_INFO *pTModule)
{
  //stop rec
  tensionCmd(pTModule, 1);

  if(pTModule->mSocket >= 0)
  {
    close(pTModule->mSocket);
    pTModule->mSocket = -1;
  }
  LOGER::PrintfLog(BASE::S_APP_LOGER, "tension leader endup");
  return 0;
}

//TUDO
///msg functions/////////////////
/******************************************************************************
* 功能：组包函数，下发到拉力计模块
* @param pMsg : pMsg是拉力计发送消息
* @param mCmd : mCmd是控制命令
* @return Descriptions
******************************************************************************/
static int packageFrame(BASE::TENSIONS_S_MSG* pMsg, uint8_t mCmd)
{
  int32_t iRet = 0;
  if(pMsg == NULL)
  {
    return -1;
  }

  struct timespec now;

  pMsg->mIdentifier = 0xFF;

  clock_gettime(CLOCK_MONOTONIC, &now);
  pMsg->mSysTime.mSysTimeS  = now.tv_sec;
  pMsg->mSysTime.mSysTimeUs = now.tv_nsec/1000;

  pMsg->mCmd = mCmd;
  pMsg->mCrcCode = 0;

  return iRet;
}

/******************************************************************************
* 功能：拉力计下发送控制指令函数
* @param pTModule : pTModule是线程信息，包含收发拉力计消息
* @param mCmd : mCmd是控制命令
* @return Descriptions
******************************************************************************/
static int tensionCmd(BASE::TENSIONS_THREAD_INFO *pTModule, uint8_t mCmd)
{
  int32_t iRet = 0;
  packageFrame(&mSendTensionMsg,  mCmd);
  iRet = sendto(pTModule->mSocket, &mSendTensionMsg, sizeof(BASE::TENSIONS_S_MSG), 0, (struct sockaddr *)&(pTModule->mPeerAddr), sizeof(pTModule->mPeerAddr));
  return  iRet;
}
////////////////////////////////////////////////////////////////////////////////
///////external interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
* 功能：线程入口函数，此模块的生命周期就在此函数中。
* @param pTModule : pTModule是线程信息指针，里边包含发送/接收消息，socket等信息
* @return Descriptions
******************************************************************************/
void* threadEntry(void* pModule)
{
  BASE::TENSIONS_THREAD_INFO *pTModule =(BASE::TENSIONS_THREAD_INFO *) pModule;
  if(NULL == pTModule)
  {
    return 0;
  }

  //leader run in cpux
  BASE::hiSetCpuAffinity(pTModule->mCpuAffinity);

  if(initServer(pTModule) != 0)
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "tension  bind server ip failed, check network again !");
    moduleEndUp(pTModule);
    return 0;
  }

  socklen_t mun = sizeof(pTModule->mPeerAddr);

  //motors data
  uint32_t     lArmsStateCode;

  BASE::TENSIONS_R_MSG mRecMsg;
  //running state
  LOGER::PrintfLog(BASE::S_APP_LOGER, "tension running!");

  uint8_t  lTensionStateCode = 0;
  uint8_t  bFirstRec = 1;

  while(pTModule->mWorking)
  {
    //rec UDP
    int size = recvfrom(pTModule->mSocket , (char*)&(mRecMsg), sizeof(BASE::TENSIONS_R_MSG), 0, (sockaddr*)&(pTModule->mPeerAddr), &mun);

    //TUDO*****
    if(size != sizeof(BASE::TENSIONS_R_MSG))
    {
      LOGER::PrintfLog(BASE::S_APP_LOGER, "%s tension thread rec overtime or error!", pTModule->mThreadName);
      continue;
    }

    //rec tension data
    lTensionStateCode = mRecMsg.mStateCode;

    //error*******
    if((lTensionStateCode%2) != 0)
    {
      LOGER::PrintfLog(BASE::S_APP_LOGER, "hard error. state code:%d", lTensionStateCode);
      continue;
    }

    //tension data
    if(bFirstRec)
    {
      //start send msg. 0 :start.  1: stop
      tensionCmd(pTModule, 0);
      bFirstRec = 0;
      continue;
    }

    //copy tensins data to supr
    pTModule->mNowTension[mRecMsg.mIdentifier].mTensions = mRecMsg.mTensions;
    pTModule->mNowTension[mRecMsg.mIdentifier].iNewTensions = true;

  }

  moduleEndUp(pTModule);
}

} //namespace
