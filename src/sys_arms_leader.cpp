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
static int packageFrame(BASE::ARMS_S_MSG* pMsg,  BASE::MOTORS &mMotors, uint16_t mCrcCode);

static uint16_t checkMotorsState(BASE::ARMS_R_MSG &mRecMsg);

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
static int packageFrame(BASE::ARMS_S_MSG* pMsg,  BASE::MOTORS &mMotors, uint16_t mCrcCode)
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

  pMsg->mCrcCode = mCrcCode;

  return iRet;
}

static int motorMoveCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors, uint8_t mCtrl, uint16_t mCrcCode)
{
  int32_t iRet = 0;

  //ctrl cmd motor
  for(int i=0; i<4; i++)
  {
    mMotors.mMotorsCmd[i].mCmd = mCtrl;
  }
  packageFrame(&pTModule->mSendMsg,  mMotors, mCrcCode);
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
    if((mRecMsg.mMotors[i].mMotorStateCode % 2) == BASE::ST_MOTOR_STOP)
    {
      iRet |= motorBit;
    }
    motorBit << 1;
  }

  return iRet;
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
    return 0;
  }

  //struct timespec  pfTime1;

  socklen_t mun = sizeof(pTModule->mPeerAddr);

  //clock_gettime(CLOCK_MONOTONIC, &pfTime1);

  //uint32_t mArmsState = -1;
  uint16_t mCrc = 0;
  //motors data
  BASE::MOTORS lMotors;
  uint32_t     lArmsStateCode;

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

    //send
    switch (pTModule->mState)
    {
      case BASE::M_STATE_INIT:
      {
        if(lArmsStateCode == BASE::ST_SYS_STATE_OK)
        {
          //check motor is power on
          if(checkMotorsState(pTModule->mRecMsg) == 0) // motor 0000  all start
          {
            //TUDO send 0 let arms wait
            memset((char*)&lMotors, 0, sizeof(lMotors));
            //zero
            motorMoveCmd(pTModule, lMotors, BASE::CT_MOTOR_ZERO, mCrc);
            pTModule->mAckState = BASE::ACK_STATE_INIT_OK;
          }
          else if(checkMotorsState(pTModule->mRecMsg) == 0xF) // motor 1111  all stop
          {
            //motor power on
            int iRet =  motorMoveCmd(pTModule, lMotors, BASE::CT_MOTOR_POWERON, mCrc);
            LOGER::PrintfLog("test log hear");
          }
        }

        break;
      }
      case BASE::M_STATE_CONF:
      {
        //TUDO
        if(lArmsStateCode == BASE::ST_SYS_STATE_OK)
        {
          //TUDO cal******,save all data to interactioner

          //if Manual ctrl move mode than move (xyz), else move (0,0,0)(relative position)
          motorMoveCmd(pTModule, lMotors, BASE::CT_MOTOR_SPEED, mCrc);
        }
        else
        {
          LOGER::PrintfLog("error code:%d\n", lArmsStateCode);
          motorMoveCmd(pTModule, lMotors, BASE::CT_MOTOR_STOP, mCrc);
          pTModule->mState = BASE::M_STATE_STOP;
        }
        break;
      }
      case BASE::M_STATE_RUN:
      {
        //TUDO
        if(lArmsStateCode == BASE::ST_SYS_STATE_OK)
        {
          //TUDO PID ctrl move to (x y z)

          motorMoveCmd(pTModule, lMotors, BASE::CT_MOTOR_SPEED, mCrc);
        }
        else
        {
          LOGER::PrintfLog("error code:%d\n", lArmsStateCode);
          motorMoveCmd(pTModule, lMotors, BASE::CT_MOTOR_STOP, mCrc);
          pTModule->mState = BASE::M_STATE_STOP;
        }
        break;
      }
      case BASE::M_STATE_STOP:
      {
        motorMoveCmd(pTModule, lMotors, BASE::CT_MOTOR_POWERDOWN, mCrc);
        pTModule->mWorking = false;
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
