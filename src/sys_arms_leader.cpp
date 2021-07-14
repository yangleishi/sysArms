/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_leader.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:机械臂控制模块，控制对应的机械臂，接收supr发来的周期性的控制指令，
将控制指令发送给机械臂板子，并接收机械臂板子发来的运行数据。
*
********************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>
#include <math.h>

#include "sys_arms_leader.hpp"
#include "sys_arms_defs.h"
#include "sys_arms_loger.hpp"
#include "sys_arms_daemon.hpp"

namespace LEADER {

BASE::MOTORS mNowMotors = {0};
static float mCmdTension = 0;
static BASE::SYS_TIME  mSysSendTime = {0};

static int  initServer(BASE::ARMS_THREAD_INFO *pTModule);
static void setFdNonblocking(int sockfd);
static void setFdTimeout(int sockfd, const int mSec, const int mUsec);
static void calSysDelayed(BASE::ReadLiftHzData  &mSysDelayed, BASE::SYS_TIME  mStartSysTime, BASE::SYS_TIME  mEndSysTime);
static void setStateCond(BASE::M_STATE_CONDITIONS &tCond, BASE::M_STATE tState, uint16_t tMotorCmd, int16_t  tCycTimes, uint16_t  tWaitAck);

static int moduleEndUp(BASE::ARMS_THREAD_INFO *pTModule);

///msg functions/////////////////
static int packageFrame(BASE::ARMS_S_MSG* pMsg,  BASE::MOTORS &mMotors, uint16_t mIdentifier, uint16_t mMsgType, uint16_t mRandomCode, uint16_t mDataLength);

static uint16_t checkMotorsState(BASE::ARMS_R_MSG &mRecMsg);
static uint16_t copyArmsRunDatas(BASE::ARMS_R_MSG mRecMsg, BASE::ReadRunAllData &mRunDatas);
static int32_t checkHardError(uint16_t mStatusCode);
//pTmodule cmd or data send to client
static int motorMoveAllCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors, uint8_t mCtrl);
static int motorMoveXYZCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors, uint8_t mCtrl, uint8_t mDirection, uint8_t mPosOrVel);

static int motorCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors);

static float readTensionValue(BASE::ARMS_THREAD_INFO *pTModule, int devInt);

//app running
static int32_t initFire(BASE::ARMS_THREAD_INFO *pTModule);
static int32_t confFire(BASE::ARMS_THREAD_INFO *pTModule);
static int32_t confCondFire(BASE::ARMS_THREAD_INFO *pTModule);
static int32_t runFire(BASE::ARMS_THREAD_INFO *pTModule);
////////////////////////////////////////////////////////////////////////////////
///////internal interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
* 功能：此函数设置socket套接字，设置为阻塞模式（不建议，阻塞模式下程序会阻塞，建议设置非阻塞超时）。
* @param sockfd : [in]套接子id，
* @return Descriptions
******************************************************************************/
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

/******************************************************************************
* 功能：此函数设置socket套接字超时时间。
* @param sockfd : [in]套接子id，
* @param mSec : [in]套接字超时妙
* @param mUsec : [in]套接字超时微妙
* @return Descriptions
******************************************************************************/
static void setFdTimeout(int sockfd, const int mSec, const int mUsec)
{
  struct timeval timeout;
  timeout.tv_sec = mSec;//秒
  timeout.tv_usec = mUsec;//微秒
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1)
    LOGER::PrintfLog(BASE::S_APP_LOGER, "setsockopt failed:");
}

/******************************************************************************
* 功能：此函数初始化模块，初始化套接字，绑定ip端口，主要和机械臂底层板对接
* @param pTModule : pTModule是线程信息结构体指针，里边存储的线程运行期间用到的数据和交换通信数据
* @return Descriptions
******************************************************************************/
static int initServer(BASE::ARMS_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;

  if((pTModule->mSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      LOGER::PrintfLog(BASE::S_APP_LOGER, "socket creat Failed");
      return -1;
  }

  //setFdNonblocking(pTModule->mSocket);
  setFdTimeout(pTModule->mSocket, CONF::SERVER_UDP_TIMEOUT_S, CONF::SERVER_UDP_TIMEOUT_US);

  //my
  bzero(&(pTModule->mSerAddr), sizeof(pTModule->mSerAddr));
  pTModule->mSerAddr.sin_family = AF_INET;
  pTModule->mSerAddr.sin_port = htons(pTModule->mMyPort);
  pTModule->mSerAddr.sin_addr.s_addr = inet_addr(pTModule->mMyIpV4Str);

  //peer
  bzero(&(pTModule->mPeerAddr), sizeof(pTModule->mPeerAddr));
  pTModule->mPeerAddr.sin_family = AF_INET;
  pTModule->mPeerAddr.sin_port = htons(pTModule->mPeerPort);
  pTModule->mPeerAddr.sin_addr.s_addr = inet_addr(pTModule->mPeerIpV4Str);

  iRet = bind(pTModule->mSocket, (struct sockaddr*)&(pTModule->mSerAddr), sizeof(pTModule->mSerAddr));
  if(iRet < 0)
    return iRet;

  //powup motors
  setStateCond(pTModule->mCond, BASE::M_STATE_INIT, BASE::CT_MOTOR_STOP, 100, 0);
  BASE::MOTORS mZeroMotors = {0};
  motorMoveAllCmd(pTModule, mZeroMotors, BASE::CT_MOTOR_STOP);
  return 0;
}

/******************************************************************************
* 功能：此函数销毁初始化模块，释放线程指针里的内容
* @param pTModule : pTModule是线程信息结构体指针，里边存储的线程运行期间用到的数据和交换通信数据
* @return Descriptions
******************************************************************************/
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

/******************************************************************************
* 功能：此函数计算系统延时（消息发出时间，到接收到机械臂控制板发来的msg时间），使用supr作为同步时钟
* @param mSysDelayed : mSysDelayed是延时结构体，和上位机延时显示结构体一致，延时将存储在此结构体中。
* @param mStartSysTime : mStartSysTime是消息发送时间。
* @param mEndSysTime : mEndSysTime是消息接收时间。
* @return Descriptions
******************************************************************************/
static void calSysDelayed(BASE::ReadLiftHzData  &mSysDelayed, BASE::SYS_TIME  mStartSysTime, BASE::SYS_TIME  mEndSysTime)
{
  mSysDelayed.mLiftHz = mEndSysTime.mSysTimeS - mStartSysTime.mSysTimeS;
  mSysDelayed.mLiftShake = mEndSysTime.mSysTimeUs - mStartSysTime.mSysTimeUs;
  mSysDelayed.mIsValid = 1;
}

/******************************************************************************
* 功能：组包函数，将motor电机控制结构，组成ARMS_S_MSG类型消息
* @param pMsg : pMsg是发送给机械臂控制板的消息，
* @param mMotors : mMotors是消电机控制数据，组包到pMsg里
* @return Descriptions
******************************************************************************/
static int packageFrame(BASE::ARMS_S_MSG* pMsg,  BASE::MOTORS &mMotors, uint16_t mIdentifier, uint16_t mMsgType, uint16_t mRandomCode, uint16_t mDataLength)
{
  int32_t iRet = 0;
  if(pMsg == NULL)
    return -1;

  struct timespec now;

  pMsg->mIdentifier = mIdentifier;
  pMsg->mMsgType    = mMsgType;
  pMsg->mRandomCode = mRandomCode;
  pMsg->mDataLength = mDataLength;

  memcpy((char*)&(pMsg->mMotors), (char*)&mMotors, sizeof(mMotors));

  clock_gettime(CLOCK_MONOTONIC, &now);
  pMsg->mSysTime.mSysTimeS  = now.tv_sec;
  pMsg->mSysTime.mSysTimeUs = now.tv_nsec/1000;

  LOGER::PrintfLog(BASE::S_APP_LOGER, "send time(us) :%d", pMsg->mSysTime.mSysTimeUs);
  mSysSendTime.mSysTimeS = pMsg->mSysTime.mSysTimeS;
  mSysSendTime.mSysTimeUs = pMsg->mSysTime.mSysTimeUs;

  //TUDO

  pMsg->mCrcCodeH = 0;
  pMsg->mCrcCodeL = 0;

  return iRet;
}

/******************************************************************************
* 功能：机械臂控制函数，发送控制指令
* @param pMsg : pTModule是线程信息结构体指针，里边存储的线程运行期间用到的数据和交换通信数据
* @param mMotors : mMotors是电机控制数据，
* @param mCtrl : mCtrl是电机控制命令，cmd
* @param mDirection : mDirection是电机转动方向
* @param mPosOrVel : mPosOrVel是电机位置控制，还是速度控制
* @return Descriptions
******************************************************************************/
static int motorMoveAllCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors, uint8_t mCtrl)
{
  int32_t iRet = 0;

  //ctrl cmd motor
  for(int i=0; i<4; i++)
  {
    mMotors.mMotorsCmd[i].mCmd = mCtrl;
  }
  iRet = motorCmd(pTModule, mMotors);

  return  iRet;
}


/******************************************************************************
* 功能：机械臂控制函数，发送控制指令。只控制XYZ轴运动，没有控制拉力电机
* @param pMsg : pTModule是线程信息结构体指针，里边存储的线程运行期间用到的数据和交换通信数据
* @param mMotors : mMotors是电机控制数据，
* @param mCtrl : mCtrl是电机控制命令，cmd
* @param mDirection : mDirection是电机转动方向
* @param mPosOrVel : mPosOrVel是电机位置控制，还是速度控制
* @return Descriptions
******************************************************************************/
static int motorMoveXYZCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors, uint8_t mCtrl, uint8_t mDirection, uint8_t mPosOrVel)
{
    int32_t iRet = 0;

    //ctrl cmd motor
    for(int i=0; i<3; i++)
    {
      mMotors.mMotorsCmd[i].mCmd = mCtrl;

      if(mDirection)
        mMotors.mMotorsCmd[i].mCmd |= (0x0100);

      if(mPosOrVel)
        mMotors.mMotorsCmd[i].mCmd |= (0x0200);
    }
    mMotors.mMotorsCmd[3].mCmd = BASE::CT_MOTOR_STOP;

    iRet = motorCmd(pTModule, mMotors);
    return  iRet;
}

/******************************************************************************
* 功能：电机控制，
* @param pTModule : pTModule是线程信息结构体，存储socket，收发电机消息结构体
* @param mMotors : mMotors是电机控制数据，组包到pMsg里
* @return Descriptions
******************************************************************************/
static int motorCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors)
{
  int32_t iRet = 0;
  packageFrame(&pTModule->mSendMsg,  mMotors, pTModule->mMsgId, BASE::M_TYPE_TO_CONTROLER_CMD, pTModule->mRandomCode++, 36);
  iRet = sendto(pTModule->mSocket, &pTModule->mSendMsg, sizeof(BASE::ARMS_S_MSG), 0, (struct sockaddr *)&(pTModule->mPeerAddr), sizeof(pTModule->mPeerAddr));
  return  iRet;
}


//TUDO
/******************************************************************************
* 功能：检查电机运行状态，是否报错
* @param mRecMsg : mRecMsg是机械臂接收消息，包含电机，传感器数据及状态
* @return Descriptions
  返回4个电机检查是否有错误标志位
******************************************************************************/
static uint16_t checkMotorsState(BASE::ARMS_R_MSG &mRecMsg)
{
  uint16_t iRet = 0;
  //check all start
  for(int i=0; i<4; i++)
  {
    if(mRecMsg.mMotors[i].mMotorStateCode != BASE::ST_SYS_OK)
      iRet ++;
  }
  return iRet;
}

/******************************************************************************
* 功能：检查机械臂硬件是否有错
* @param mStatusCode :mStatusCode是机械臂接收消息的状态码，标识机械臂是否出错
* @return Descriptions
  返回机械臂是否报错
******************************************************************************/
static int32_t checkHardError(uint16_t mStatusCode)
{
  int32_t iRet = 0;
  if(mStatusCode != BASE::ST_SYS_OK)
    iRet = 1;
  return iRet;
}

/******************************************************************************
* 功能：读取拉力计值
* @param pTModule : pTModule是线程信息结构体，存储有拉力计结构体指针
* @param devInt : devInt是拉力计index下标
* @return Descriptions
  返回devInt拉力计数据
******************************************************************************/
static float readTensionValue(BASE::ARMS_THREAD_INFO *pTModule, int devInt)
{
  float iRet = -1;
  //如果DEF_SYS_USE_TENSIONLEADER_NUMS=0 则不采用无线模块，拉力计信息在RecMsg里边，否则拉力计信息需要无线模块传输。
  iRet = ((DEF_SYS_USE_TENSIONLEADER_NUMS == 0) ? iRet = pTModule->mRecMsg.mTension : pTModule->mNowTension[devInt].mTensions);
  return  iRet;
}


/******************************************************************************
* 功能：leader处于初始化状态下，循环调用的函数，主要是检查arms状态，上电机械臂电机
* @param pTModule : pTModule是线程信息结构体，存储有拉力计结构体指针
* @return Descriptions
******************************************************************************/
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
    motorMoveAllCmd(pTModule, lMotors, BASE::CT_MOTOR_MOVE_POSITIVE);
    printf("init motor cmd\n");

    if(pTModule->mAckState != BASE::ACK_STATE_INIT_OK)
      pTModule->mAckState = BASE::ACK_STATE_INIT_OK;
  }
  else //some motors is stop
  {
    for (int i=0;i<4;i++)
    {
      //i motor is stop,then send cmd to stop
      lMotors.mMotorsCmd[i].mCmd = BASE::CT_MOTOR_STOP;
    }
    //if motor is stop then power on else motor running (0,0)wait
    motorCmd(pTModule, lMotors);
    LOGER::PrintfLog(BASE::S_APP_LOGER, "send stop to motors!");
  }
  return iRet;
}

/******************************************************************************
* 功能：leader处于配置状态下，执行某个命令期间，此函数会一直被调用，直到命令达到，或者新的命令到来
* @param pTModule : pTModule是线程信息结构体，存储有拉力计结构体指针
* @return Descriptions
******************************************************************************/
static int32_t confCondFire(BASE::ARMS_THREAD_INFO *pTModule)
{
  //接收命令后执行命令中
  switch (pTModule->mCond.mMotorCmd)
  {
    case BASE::ST_HAND_MOVE_RUNNING:
    {
      if(abs(pTModule->mRecMsg.mMotors[0].mPosition-mNowMotors.mMotorsCmd[0].mPosition) > 0.001 &&
         abs(pTModule->mRecMsg.mMotors[1].mPosition-mNowMotors.mMotorsCmd[1].mPosition) > 0.001 &&
         abs(pTModule->mRecMsg.mMotors[2].mPosition-mNowMotors.mMotorsCmd[2].mPosition) > 0.001 &&
         abs(pTModule->mRecMsg.mMotors[3].mPosition-mNowMotors.mMotorsCmd[3].mPosition) > 0.001 )
      {
          motorMoveAllCmd(pTModule, mNowMotors, BASE::CT_MOTOR_MOVE_POSITIVE);
          printf("conf motor cmd postion:%f, now postion:%f\n",  mNowMotors.mMotorsCmd[0].mPosition, pTModule->mRecMsg.mMotors[0].mPosition);
          //send rec running data to uper
          pTModule->mReadLiftSigalNowData.mHandXMoveNow = pTModule->mRecMsg.mMotors[0].mPosition;
          pTModule->mReadLiftSigalNowData.mHandYMoveNow = pTModule->mRecMsg.mMotors[1].mPosition;
          pTModule->mReadLiftSigalNowData.mHandZMoveNow = pTModule->mRecMsg.mMotors[2].mPosition;
          pTModule->mReadLiftSigalNowData.mHandWMoveNow = pTModule->mRecMsg.mMotors[3].mPosition;
      }
      else
      {
          BASE::MOTORS mZeroMotors = {0};
          motorMoveAllCmd(pTModule, mZeroMotors, BASE::CT_MOTOR_STOP);
      }
      break;
    }
    case BASE::ST_ALL_MOVE_RUNNING:
    {
      if(abs(pTModule->mRecMsg.mMotors[0].mPosition-mNowMotors.mMotorsCmd[0].mPosition) > 0.001 &&
         abs(pTModule->mRecMsg.mMotors[1].mPosition-mNowMotors.mMotorsCmd[1].mPosition) > 0.001 &&
         abs(pTModule->mRecMsg.mMotors[2].mPosition-mNowMotors.mMotorsCmd[2].mPosition) > 0.001)
      {
          motorMoveXYZCmd(pTModule, mNowMotors, BASE::CT_MOTOR_MOVE_POSITIVE, 0, 0);
          printf("conf motor all move postion:%f, now postion:%f\n",  mNowMotors.mMotorsCmd[0].mPosition, pTModule->mRecMsg.mMotors[0].mPosition);
          //send rec running data to uper
          pTModule->mReadLiftSigalNowData.mHandXMoveNow = pTModule->mRecMsg.mMotors[0].mPosition;
          pTModule->mReadLiftSigalNowData.mHandYMoveNow = pTModule->mRecMsg.mMotors[1].mPosition;
          pTModule->mReadLiftSigalNowData.mHandZMoveNow = pTModule->mRecMsg.mMotors[2].mPosition;
          pTModule->mReadLiftSigalNowData.mHandWMoveNow = pTModule->mRecMsg.mMotors[3].mPosition;
      }
      else
      {
          BASE::MOTORS mZeroMotors = {0};
          motorMoveAllCmd(pTModule, mZeroMotors, BASE::CT_MOTOR_STOP);
      }
      break;
    }
    case BASE::ST_ALL_PULL_RUNNING:
    {
      ////pull 命令，根据拉立计信息慢慢的增加电机转速。
      float tensiosV = readTensionValue(pTModule,  pTModule->mRecMsg.mIdentifier);
      if(fabs(tensiosV-mCmdTension) > 0.001)
      {
        BASE::MOTORS mZeroMotors = {0};
        memset((char*)&mZeroMotors, 0, sizeof(BASE::MOTORS));
        mZeroMotors.mMotorsCmd[3].mSpeed = 0.01;
        ////TUDO 正转  倒转
        if(tensiosV > mCmdTension)
            motorMoveAllCmd(pTModule, mZeroMotors, BASE::CT_MOTOR_MOVE_POSITIVE);
        else
            motorMoveAllCmd(pTModule, mZeroMotors, BASE::CT_MOTOR_MOVE_INVERSE);
         printf("conf motor all pull move:set tension %f, now tension:%f, state:%d\n",  mCmdTension, tensiosV, pTModule->mIsNowMotorCmd);
      }
      else
      {
          BASE::MOTORS mZeroMotors = {0};
          motorMoveAllCmd(pTModule, mZeroMotors, BASE::CT_MOTOR_STOP);
      }
      break;
    }
    case BASE::ST_LIFT_STOPED:
    {
      BASE::MOTORS mZeroMotors = {0};
      motorMoveAllCmd(pTModule, mZeroMotors, BASE::CT_MOTOR_STOP);
      printf("lift stop cmd....\n");
      break;
    }
    //run mode
    case BASE::ST_ALL_RUN_RUNNING:
    {
      //TUDO PID ctrl move to (x y z)
      BASE::MOTORS lMotors = {0};
      pTModule->mNewRecMsg = true;
      float tensiosV = readTensionValue(pTModule,  pTModule->mRecMsg.mIdentifier);
      pTModule->mReadRunData.runD_PullNow = tensiosV;
      copyArmsRunDatas(pTModule->mRecMsg, pTModule->mReadRunData);
      motorMoveAllCmd(pTModule, lMotors, BASE::CT_MOTOR_MOVE_POSITIVE);
      printf("run motor all running\n");
      break;
    }
    case BASE::ST_RUN_STOPED:
    {
      BASE::MOTORS mZeroMotors = {0};
      motorMoveAllCmd(pTModule, mZeroMotors, BASE::CT_MOTOR_STOP);
      printf("run stop cmd....  send:%d , rec:%d\n", pTModule->mRandomCode, pTModule->mRecMsg.mRandomCode);
      break;
    }
    default:
    {
      printf("conf cond default....\n");
      break;
    }
  }

}

/******************************************************************************
* 功能：leader处于配置状态下，循环调用的函数，配置模式下，上位机可以读取、保存配置，手动控制
       整体控制等
* @param pTModule : pTModule是线程信息结构体，存储有拉力计结构体指针
* @return Descriptions
******************************************************************************/
static int32_t confFire(BASE::ARMS_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;

  //检查是否机械臂报错
  if(checkHardError(pTModule->mRecMsg.mStateCode))
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "***conf state: hard error.statues code :%d", pTModule->mRecMsg.mStateCode);
    //stop all modules
    pTModule->mState = BASE::M_STATE_STOP;
    //TODU
    return -1;
  }

  //检查电机是否报错
  uint16_t mMotorState = checkMotorsState(pTModule->mRecMsg);
  if(mMotorState != 0) // some motor stop. then stop all modules
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "***conf state: have motor stop. motor state:%d", mMotorState);
    //stop all modules
    pTModule->mState = BASE::M_STATE_STOP;
    return -2;
  }

/**************************** TUDU send cmd to motor*************************/
  //接收新的命令
  switch (pTModule->mIsNowMotorCmd)
  {
    //上位机发来的cmd是不是停止命令，如果是的话就停止电机转动
    case BASE::CMD_HAND_MOVE_STOP:
    case BASE::CMD_ALL_MOVE_STOP:
    case BASE::CMD_ALL_PULL_STOP:
    {
      setStateCond(pTModule->mCond, BASE::M_STATE_CONF, BASE::ST_LIFT_STOPED, 0, 0);
      printf("conf motor cmd stop\n");
      LOGER::PrintfLog(BASE::S_APP_LOGER, "***conf state: have motor stop. motor state:%d", pTModule->mIsNowMotorCmd);
      pTModule->mIsNowMotorCmd = BASE::CMD_INTO_COND_RUN;
      break;
    }
    case BASE::CMD_HAND_MOVE_START:
    {
      BASE::MoveLiftSigalData lMovedata = {0};
      pthread_mutex_lock(&pTModule->mMotorMutex);
      memcpy((char*)&lMovedata, (char*)&pTModule->mMoveData, sizeof(BASE::MoveLiftSigalData));
      pthread_mutex_unlock(&pTModule->mMotorMutex);

      //if Manual ctrl move mode than move (xyz), else move (0,0,0)(relative position)
      memset((char*)&mNowMotors, 0, sizeof(mNowMotors));
      mNowMotors.mMotorsCmd[0].mPosition = lMovedata.mHandXMove;
      mNowMotors.mMotorsCmd[1].mPosition = lMovedata.mHandYMove;
      mNowMotors.mMotorsCmd[2].mPosition = lMovedata.mHandZMove;
      mNowMotors.mMotorsCmd[3].mPosition = lMovedata.mHandWMove;
      setStateCond(pTModule->mCond, BASE::M_STATE_CONF, BASE::ST_HAND_MOVE_RUNNING, 0, 0);
      pTModule->mIsNowMotorCmd = BASE::CMD_INTO_COND_RUN;
      break;
    }
    case BASE::CMD_ALL_MOVE_START:
    {
      BASE::MoveLiftAllData lMovedata = {0};
      pthread_mutex_lock(&pTModule->mMotorMutex);
      memcpy((char*)&lMovedata, (char*)&pTModule->mMoveData, sizeof(BASE::MoveLiftAllData));
      pthread_mutex_unlock(&pTModule->mMotorMutex);

      //if Manual ctrl move mode than move (xyz), else move (0,0,0)(relative position)
      memset((char*)&mNowMotors, 0, sizeof(mNowMotors));
      mNowMotors.mMotorsCmd[0].mPosition = lMovedata.mHandXMove;
      mNowMotors.mMotorsCmd[1].mPosition = lMovedata.mHandYMove;
      mNowMotors.mMotorsCmd[2].mPosition = lMovedata.mHandZMove;
      setStateCond(pTModule->mCond, BASE::M_STATE_CONF, BASE::ST_ALL_MOVE_RUNNING, 0, 0);
      pTModule->mIsNowMotorCmd = BASE::CMD_INTO_COND_RUN;
      break;
    }
    case BASE::CMD_ALL_PULL_START:
    {
      //TUDO
      BASE::PullLiftAllData lPulldata = {0};
      pthread_mutex_lock(&pTModule->mMotorMutex);
      memcpy((char*)&lPulldata, (char*)&pTModule->mAllPullData, sizeof(BASE::PullLiftAllData));
      pthread_mutex_unlock(&pTModule->mMotorMutex);

      mCmdTension = lPulldata.mHandPull;

      //if Manual ctrl move mode than move (xyz), else move (0,0,0)(relative position)
      memset((char*)&mNowMotors, 0, sizeof(mNowMotors));
      setStateCond(pTModule->mCond, BASE::M_STATE_CONF, BASE::ST_ALL_PULL_RUNNING, 0, 0);
      pTModule->mIsNowMotorCmd = BASE::CMD_INTO_COND_RUN;
      break;
    }
    case BASE::CMD_INTO_COND_RUN:
    {
      //printf("cond running....\n");
      break;
    }
    default:
    {
      BASE::MOTORS mZeroMotors = {0};
      //LOGER::PrintfLog(BASE::S_APP_LOGER,"wait cmd....  send:%d , rec:%d", pTModule->mRandomCode, pTModule->mRecMsg.mRandomCode);
      motorMoveAllCmd(pTModule, mZeroMotors, BASE::CT_MOTOR_STOP);
      break;
    }
  }

  //陷入条件运行
  if(pTModule->mIsNowMotorCmd == BASE::CMD_INTO_COND_RUN)
    confCondFire(pTModule);

  return iRet;
}

/******************************************************************************
* 功能：机械臂周期运行的数据需要leader拷贝到supr线程中，然后interaction在从supr中发送给上位机显示
* @param mRecMsg : mRecMsg是接收到控制板发来的数据，包含机械臂运行时候的各种传感器数据
* @param mRunDatas : mRunDatas是机械臂运行数据结构体，发送到上位机显示的
* @return Descriptions
******************************************************************************/
static uint16_t copyArmsRunDatas(BASE::ARMS_R_MSG mRecMsg, BASE::ReadRunAllData &mRunDatas)
{
  mRunDatas.mIsValid = 1;
  mRunDatas.runD_Rsiko1 = mRecMsg.mSiko1;
  mRunDatas.runD_Rsiko2 = mRecMsg.mSiko2;
  mRunDatas.runD_Level1 = mRecMsg.mInclinometer1_x;
  mRunDatas.runD_Level2 = mRecMsg.mInclinometer1_y;
  mRunDatas.runR_sysMsg_X = mRecMsg.mMotors[0].mPosition;
  mRunDatas.runR_sysMsg_Y = mRecMsg.mMotors[1].mPosition;
  mRunDatas.runR_sysMsg_Z = mRecMsg.mMotors[2].mPosition;

  mRunDatas.runD_RencoderT = mRecMsg.mEncoderTurns;
  mRunDatas.runD_RencoderXNow = mRecMsg.mMotors[0].mEncoderTurns;
  mRunDatas.runD_RencoderYNow = mRecMsg.mMotors[1].mEncoderTurns;
  mRunDatas.runD_RencoderZNow = mRecMsg.mMotors[2].mEncoderTurns;
}

/******************************************************************************
* 功能：机械臂在运行模式下周期调用的函数，包含系统检测，控制命令发送，运动数据拷贝等
* @param pTModule : pTModule是线程信息结构体，存储有拉力计结构体指针等
* @return Descriptions
******************************************************************************/
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

  /**************************** TUDU send cmd to motor*************************/
  switch (pTModule->mIsNowMotorCmd)
  {
    case BASE::CMD_RUN_STOP:
    {
      //TUDO PID ctrl move to (x y z)
      setStateCond(pTModule->mCond, BASE::M_STATE_RUN, BASE::ST_RUN_STOPED, 0, 0);
      pTModule->mIsNowMotorCmd = BASE::CMD_INTO_COND_RUN;
      break;
    }
    case BASE::CMD_RUN_START:
    {
      //TUDO PID ctrl move to (x y z)
      setStateCond(pTModule->mCond, BASE::M_STATE_RUN, BASE::ST_ALL_RUN_RUNNING, 0, 0);
      pTModule->mIsNowMotorCmd = BASE::CMD_INTO_COND_RUN;
      break;
    }
    case BASE::CMD_INTO_COND_RUN:
    {
      break;
    }
    default:
    {
      BASE::MOTORS mZeroMotors = {0};
      motorMoveAllCmd(pTModule, mZeroMotors, BASE::CT_MOTOR_STOP);
      printf("running wait cmd....\n");
      break;
    }
  }

  //陷入条件运行
  if(pTModule->mIsNowMotorCmd == BASE::CMD_INTO_COND_RUN)
    confCondFire(pTModule);

  return  iRet;
}

/******************************************************************************
* 功能：设置状态条件变量
* @param tCond : tCond是条件状态变量
* @param tState : tState是当前运行状态
* @param tMotorCmd : tMotorCmd是当前运行的命令
* @param tCycTimes : tCycTimes是设置当前命令运行周期数，0不设置
* @param tWaitAck : tWaitAck是设置将要接收的Ack命令，0不设置
* @return Descriptions
******************************************************************************/
static void setStateCond(BASE::M_STATE_CONDITIONS &tCond, BASE::M_STATE tState, uint16_t tMotorCmd, int16_t  tCycTimes, uint16_t  tWaitAck)
{
  tCond.mState = tState;
  tCond.mMotorCmd = tMotorCmd;
  tCond.mCycTimes = tCycTimes;
  tCond.mWaitAck  = tWaitAck;
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
    memset((char*)&pTModule->mRecMsg, 0 ,sizeof(pTModule->mRecMsg));
    int size = recvfrom(pTModule->mSocket , (char*)&(pTModule->mRecMsg), sizeof(BASE::ARMS_R_MSG), 0, (sockaddr*)&(pTModule->mPeerAddr), &mun);

    calSysDelayed(pTModule->mReadLiftHzData, mSysSendTime, pTModule->mRecMsg.mSysTime);
    //printf("system delayed: %ds, %dus, now: \n",pTModule->mSysDelayed.mSysTimeS, pTModule->mSysDelayed.mSysTimeUs);

    switch (pTModule->mState)
    {
      case BASE::M_STATE_INIT:
      {
        //check if have client link
        if(size != sizeof(BASE::ARMS_R_MSG))
        {
          //判断是否还在上电过程中
          if((pTModule->mCond.mMotorCmd == BASE::CT_MOTOR_STOP) && ((--pTModule->mCond.mCycTimes) > 0))
          {
            BASE::MOTORS mZeroMotors = {0};
            motorMoveAllCmd(pTModule, mZeroMotors, BASE::CT_MOTOR_STOP);
            LOGER::PrintfLog(BASE::S_APP_LOGER, "power on doing, wait:%d times, size:%d", pTModule->mCond.mCycTimes, size);
            break;
          }
          else
          {
            LOGER::PrintfLog(BASE::S_APP_LOGER, "init state: %s, no client link", pTModule->mThreadName);
            pTModule->mState = BASE::M_STATE_QUIT;
            break;
          }
        }
        initFire(pTModule);
        break;
      }
      case BASE::M_STATE_CONF:
      {
        //TUDO**** rec error msg then stop app
        //printf("ok rec size:%d, mpid:%d\n", size, pTModule->mRecMsg.mApid);
        //perror("leader rec");
        if(size != sizeof(BASE::ARMS_R_MSG))
        {
          LOGER::PrintfLog(BASE::S_APP_LOGER, "conf state: %s, client overtime or lost link. stop app", pTModule->mThreadName);
          //stop all modules
          pTModule->mState = BASE::M_STATE_QUIT;
          perror("this");
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
        motorMoveAllCmd(pTModule, lMotors, BASE::CT_MOTOR_STOP);
        //printf("motor stoped state...\n");
        break;
      }
      case BASE::M_STATE_QUIT:
      {
        motorMoveAllCmd(pTModule, lMotors, BASE::CT_MOTOR_STOP);
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
