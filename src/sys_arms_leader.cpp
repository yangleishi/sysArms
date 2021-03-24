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

namespace LEADER {

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
////////////////////////////////////////////////////////////////////////////////
///////internal interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

//设置非阻塞
static void setFdNonblocking(int sockfd)
{
    int flag = fcntl(sockfd, F_GETFL, 0);
    if (flag < 0) {
        LOGER::PrintfLog("fcntl F_GETFL fail");
        return;
    }
    if (fcntl(sockfd, F_SETFL, flag | O_NONBLOCK) < 0) {
        LOGER::PrintfLog("fcntl F_SETFL fail");
    }
}


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

static int initServer(BASE::ARMS_THREAD_INFO *pTModule)
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
  LOGER::PrintfLog("endup  ");
  return 0;
}


///msg functions/////////////////
static int packageFrame(BASE::ARMS_S_MSG* pMsg,  BASE::MOTORS &mMotors)
{
  int32_t iRet = 0;
  if(pMsg == NULL)
  {
    return -1;
  }

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
    {
      iRet |= motorBit;
    }
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
    LOGER::PrintfLog("%s  bind server ip failed, check network again !", pTModule->mThreadName);
    moduleEndUp(pTModule);
    pTModule->mWorking = false;

    //working bit set 1,stop
    pthread_mutex_lock(pTModule->mCheckWorkingMutex);
    *pTModule->mWorkingBit |= ((0x0001)<<(pTModule->mSerialNumber));
    pthread_mutex_unlock(pTModule->mCheckWorkingMutex);

    return 0;
  }

  //struct timespec  pfTime1;

  socklen_t mun = sizeof(pTModule->mPeerAddr);

  //clock_gettime(CLOCK_MONOTONIC, &pfTime1);

  //uint32_t mArmsState = -1;
  uint16_t mCrc = 0;
  //motors data
  BASE::MOTORS lMotors;
  uint16_t     lArmsStateCode;

  LOGER::PrintfLog("%s running!",pTModule->mThreadName);
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
    lArmsStateCode = (size != sizeof(BASE::ARMS_R_MSG)) ? BASE::ST_SYS_REC_ERROR : pTModule->mRecMsg.mStateCode;
    memset((char*)&lMotors, 0, sizeof(lMotors));

    //hard error
    if(checkHardError(lArmsStateCode))
    {
      LOGER::PrintfLog("hard error.statues code :%d",lArmsStateCode);

      //stop all modules
      pTModule->mState = BASE::M_STATE_STOP;
      //TODU
    }
    //send
    switch (pTModule->mState)
    {
      case BASE::M_STATE_INIT:
      {
        uint16_t mMotorState = checkMotorsState(pTModule->mRecMsg);
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
          //motor power on
          int iRet =  motorCmd(pTModule, lMotors);
          LOGER::PrintfLog("power on motors!");
        }
        break;
      }
      case BASE::M_STATE_CONF:
      {
        //TUDO*******
        //if Manual ctrl move mode than move (xyz), else move (0,0,0)(relative position)
        motorMoveCmd(pTModule, lMotors, BASE::CT_MOTOR_RUN, 0, 0);
        break;
      }
      case BASE::M_STATE_RUN:
      {
        //TUDO*******
        //TUDO PID ctrl move to (x y z)
        pTModule->mNewRecMsg = true;
        float tensiosV = readTensionValue(pTModule,  pTModule->mRecMsg.mIdentifier);

        motorMoveCmd(pTModule, lMotors, BASE::CT_MOTOR_RUN, 0, 0);
        break;
      }
      case BASE::M_STATE_STOP:
      {
        motorMoveCmd(pTModule, lMotors, BASE::CT_MOTOR_POWERDOWN, 0, 0);
        pTModule->mWorking = false;
        //working bit set 1,stop
        pthread_mutex_lock(pTModule->mCheckWorkingMutex);
        *pTModule->mWorkingBit |= ((0x0001)<<(pTModule->mSerialNumber));
        pthread_mutex_unlock(pTModule->mCheckWorkingMutex);
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
