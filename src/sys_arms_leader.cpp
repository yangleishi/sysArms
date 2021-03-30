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
#include "sys_arms_loger.hpp"
#include "sys_arms_daemon.hpp"

namespace LEADER {

static pthread_mutex_t *mPrintQueueMutex = NULL;

static int  initServer(BASE::ARMS_THREAD_INFO *pTModule);
static void setFdNonblocking(int sockfd);
static void setFdTimeout(int sockfd, const int mSec, const int mUsec);

static int moduleEndUp(BASE::ARMS_THREAD_INFO *pTModule);

///msg functions/////////////////
static int packageFrame(BASE::ARMS_S_MSG* pMsg,  BASE::MOTORS &mMotors);

static uint16_t checkMotorsState(BASE::ARMS_R_MSG &mRecMsg);
static int32_t checkHardError(uint16_t mStatusCode);
//pTmodule cmd or data send to client
static int motorMoveCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors, uint8_t mCtrl, uint8_t mDirection, uint8_t mPosOrVel);
static int motorCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors);

static float readTensionValue(BASE::ARMS_THREAD_INFO *pTModule, int devInt);

//app running
static int32_t initFire(BASE::ARMS_THREAD_INFO *pTModule);
static int32_t confFire(BASE::ARMS_THREAD_INFO *pTModule);
static int32_t runFire(BASE::ARMS_THREAD_INFO *pTModule);
////////////////////////////////////////////////////////////////////////////////
///////internal interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//设置非阻塞
static void setFdNonblocking(int sockfd)
{
    int flag = fcntl(sockfd, F_GETFL, 0);
    if (flag < 0) {
        LOGER::PrintfLog(BASE::S_APP_LOGER, "fcntl F_GETFL fail");
        return;
    }
    if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) {
        LOGER::PrintfLog(BASE::S_APP_LOGER, "fcntl F_SETFL fail");
    }
}


static void setFdTimeout(int sockfd, const int mSec, const int mUsec)
{
  struct timeval timeout;
  timeout.tv_sec = mSec;//秒
  timeout.tv_usec = mUsec;//微秒
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1)
    LOGER::PrintfLog(BASE::S_APP_LOGER, "setsockopt failed:");
}

static int initServer(BASE::ARMS_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;

  if((pTModule->mSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      LOGER::PrintfLog(BASE::S_APP_LOGER, "socket creat Failed");
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
    return iRet;

  return 0;
}

static int moduleEndUp(BASE::ARMS_THREAD_INFO *pTModule)
{
  if(pTModule->mSocket >= 0)
  {
    close(pTModule->mSocket);
    pTModule->mSocket = -1;
  }
  LOGER::PrintfLog(BASE::S_APP_LOGER, "%s leader endup", pTModule->mThreadName);
  return 0;
}


///msg functions/////////////////
static int packageFrame(BASE::ARMS_S_MSG* pMsg,  BASE::MOTORS &mMotors)
{
  int32_t iRet = 0;
  if(pMsg == NULL)
    return -1;

  struct timespec now;

  pMsg->mFrameStart = 0x1ACF;
  pMsg->mIdentifier = 0xFF;
  pMsg->mApid = 0x02;

  memcpy((char*)&(pMsg->mMotors), (char*)&mMotors, sizeof(mMotors));

  clock_gettime(CLOCK_MONOTONIC, &now);
  pMsg->mSysTime.mSysTimeS  = now.tv_sec;
  pMsg->mSysTime.mSysTimeUs = now.tv_nsec/1000;

  pMsg->mCrcCode = 0;

  return iRet;
}

static int motorMoveCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors, uint8_t mCtrl, uint8_t mDirection, uint8_t mPosOrVel)
{
  int32_t iRet = 0;

  //ctrl cmd motor
  for(int i=0; i<4; i++)
  {
    mMotors.mMotorsCmd[i].mCmd = mCtrl;

    if(mDirection)
      mMotors.mMotorsCmd[i].mCmd |= (0x0100);

    if(mPosOrVel)
      mMotors.mMotorsCmd[i].mCmd |= (0x0200);
  }
  iRet = motorCmd(pTModule, mMotors);

  return  iRet;
}

static int motorCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors)
{
  int32_t iRet = 0;
  packageFrame(&pTModule->mSendMsg,  mMotors);
  iRet = sendto(pTModule->mSocket, &pTModule->mSendMsg, sizeof(BASE::ARMS_S_MSG), 0, (struct sockaddr *)&(pTModule->mPeerAddr), sizeof(pTModule->mPeerAddr));
  return  iRet;
}

static uint16_t checkMotorsState(BASE::ARMS_R_MSG &mRecMsg)
{
  uint16_t iRet = 0;
  uint16_t motorBit = 1;
  //check all start
  for(int i=0; i<4; i++)
  {
    if(((mRecMsg.mMotors[i].mMotorStateCode>>1) % 2))
      iRet |= motorBit;

    motorBit << 1;
  }
  return iRet;
}

static int32_t checkHardError(uint16_t mStatusCode)
{
  int32_t iRet = 0;
  if(mStatusCode%2)
    iRet = 1;
  return iRet;
}

static float readTensionValue(BASE::ARMS_THREAD_INFO *pTModule, int devInt)
{
  return  pTModule->mNowTensionMsg[devInt].iNewTensions ? pTModule->mNowTensionMsg[devInt].mTensions : -1.0;
}

static int32_t initFire(BASE::ARMS_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;
  //link ok .check hard error
  if(checkHardError(pTModule->mRecMsg.mStateCode))
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "hard error.statues code :%d", pTModule->mRecMsg.mStateCode);
    //stop all modules
    pTModule->mState = BASE::M_STATE_STOP;
    //TODU
    return -1;
  }

  BASE::MOTORS lMotors = {0};
  uint16_t mMotorState = checkMotorsState(pTModule->mRecMsg);
  LOGER::PrintfLog(BASE::S_APP_LOGER, "motor state :%d", mMotorState);
  //check motor is power on
  if(mMotorState == 0) // motor 0000  all start,then change state
  {
    //TUDO send 0 let arms wait
    motorMoveCmd(pTModule, lMotors, BASE::CT_MOTOR_RUN, 0, 0);

    if(pTModule->mAckState != BASE::ACK_STATE_INIT_OK)
      pTModule->mAckState = BASE::ACK_STATE_INIT_OK;
  }
  else //some motors is stop
  {
    for (int i=0;i<4;i++)
    {
      //i motor is stop,then send cmd to start
      lMotors.mMotorsCmd[i].mCmd = (mMotorState%2) ? BASE::CT_MOTOR_POWERON : BASE::CT_MOTOR_RUN;

      mMotorState = (mMotorState>>1);
    }
    //if motor is stop then power on else motor running (0,0)wait
    motorCmd(pTModule, lMotors);
    LOGER::PrintfLog(BASE::S_APP_LOGER, "power on motors!");
  }
  return iRet;
}

static int32_t confFire(BASE::ARMS_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;
  //check hard error
  if(checkHardError(pTModule->mRecMsg.mStateCode))
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "***conf state: hard error.statues code :%d", pTModule->mRecMsg.mStateCode);
    //stop all modules
    pTModule->mState = BASE::M_STATE_STOP;
    //TODU
    return -1;
  }

  //check motor if running
  uint16_t mMotorState = checkMotorsState(pTModule->mRecMsg);
  if(mMotorState != 0) // some motor stop. then stop all modules
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "***conf state: have motor stop. motor state:%d", mMotorState);
    //stop all modules
    pTModule->mState = BASE::M_STATE_STOP;
    return -2;
  }
  //TUDU send cmd to motor
  BASE::MOTORS lMotors = {0};

  //if Manual ctrl move mode than move (xyz), else move (0,0,0)(relative position)
  motorMoveCmd(pTModule, lMotors, BASE::CT_MOTOR_RUN, 0, 0);
  return iRet;
}

static int32_t runFire(BASE::ARMS_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;
  //check hard error
  if(checkHardError(pTModule->mRecMsg.mStateCode))
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "***run state: hard error.statues code :%d", pTModule->mRecMsg.mStateCode);
    //stop all modules
    pTModule->mState = BASE::M_STATE_STOP;
    //TODU
    return -1;
  }

  //check motor if running
  uint16_t mMotorState = checkMotorsState(pTModule->mRecMsg);
  if(mMotorState != 0) // some motor stop. then stop all modules
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "***run state: have motor stop. motor state:%d", mMotorState);
    //stop all modules
    pTModule->mState = BASE::M_STATE_STOP;
    return -2;
  }

  //TUDO PID ctrl move to (x y z)
  BASE::MOTORS lMotors = {0};
  pTModule->mNewRecMsg = true;
  float tensiosV = readTensionValue(pTModule,  pTModule->mRecMsg.mIdentifier);

  motorMoveCmd(pTModule, lMotors, BASE::CT_MOTOR_RUN, 0, 0);
  return  iRet;
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
  //leader run in cpux
  BASE::hiSetCpuAffinity(pTModule->mCpuAffinity);

  if(initServer(pTModule) != 0)
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "leader bind server ip failed, check network again !");
    moduleEndUp(pTModule);
    pTModule->mWorking = false;
    return 0;
  }

  socklen_t mun = sizeof(pTModule->mPeerAddr);

  //motors data
  BASE::MOTORS lMotors;

  LOGER::PrintfLog(BASE::S_APP_LOGER, "%s leader running!", pTModule->mThreadName);
  //running state
  while(pTModule->mWorking)
  {
    //lock,wait signal
    pthread_mutex_lock(&pTModule->mArmsMsgMutex);
    pthread_cond_wait(&pTModule->mArmsMsgReady, &pTModule->mArmsMsgMutex);
    pthread_mutex_unlock(&pTModule->mArmsMsgMutex);

    //rec msg
    int size = recvfrom(pTModule->mSocket , (char*)&(pTModule->mRecMsg), sizeof(BASE::ARMS_R_MSG), 0, (sockaddr*)&(pTModule->mPeerAddr), &mun);

    switch (pTModule->mState)
    {
      case BASE::M_STATE_INIT:
      {
        //check if have client link
        if(size != sizeof(BASE::ARMS_R_MSG))
        {
          LOGER::PrintfLog(BASE::S_APP_LOGER, "init state: %s, no client link", pTModule->mThreadName);
          break;
        }
        initFire(pTModule);
        break;
      }
      case BASE::M_STATE_CONF:
      {
        //TUDO**** rec error msg then stop app
        if(size != sizeof(BASE::ARMS_R_MSG))
        {
          LOGER::PrintfLog(BASE::S_APP_LOGER, "conf state: %s, client overtime or lost link. stop app", pTModule->mThreadName);
          //stop all modules
          pTModule->mState = BASE::M_STATE_STOP;
          break;
        }
        confFire(pTModule);
        break;
      }
      case BASE::M_STATE_RUN:
      {
        //TUDO**** rec error msg then stop app
        if(size != sizeof(BASE::ARMS_R_MSG))
        {
          LOGER::PrintfLog(BASE::S_APP_LOGER, "run state: %s, client overtime or lost link. stop app", pTModule->mThreadName);
          //stop all modules
          pTModule->mState = BASE::M_STATE_STOP;
          break;
        }
        runFire(pTModule);
        break;
      }
      case BASE::M_STATE_STOP:
      {
        motorMoveCmd(pTModule, lMotors, BASE::CT_MOTOR_POWERDOWN, 0, 0);
        pTModule->mWorking = false;
        break;
      }
      default:
      {
        break;
      }
    }

    //mCrc++;
  }

  moduleEndUp(pTModule);
}

} //namespace
