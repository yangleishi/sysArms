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

#include <stdio.h>
#include <stdlib.h>
#include <stdarg.h>

#include "sys_arms_leader.hpp"
#include "sys_arms_defs.h"
#include "sys_arms_loger.hpp"
#include "sys_arms_daemon.hpp"


namespace LEADER {


static int  initServer(BASE::ARMS_THREAD_INFO *pTModule);
static void setFdNonblocking(int sockfd);
static void setFdTimeout(int sockfd, const int mSec, const int mUsec);
static int32_t findMotorNum(int32_t index);

static void calSysDelayed(BASE::ReadLiftHzData  &mSysDelayed, BASE::SYS_TIME  mStartSysTime, BASE::SYS_TIME  mEndSysTime);
static void reformRecMsg(BASE::ARMS_THREAD_INFO *pTModule);
static void calibrationSensors(BASE::ARMS_THREAD_INFO *pTModule);
static int32_t checkRecMsgError(BASE::ARMS_THREAD_INFO *pTModule);
static void setStateCond(BASE::M_STATE_CONDITIONS &tCond, BASE::M_STATE tState, uint16_t tMotorCmd, int16_t  tCycTimes, uint16_t  tWaitAck);

static int moduleEndUp(BASE::ARMS_THREAD_INFO *pTModule);

///msg functions/////////////////
static int packageFrame(BASE::ARMS_S_MSG* pMsg,  BASE::MOTORS &mMotors, uint16_t mIdentifier, uint16_t mMsgType, uint16_t mRandomCode, uint16_t mDataLength);

static uint16_t checkMotorsState(BASE::ARMS_R_MSG &mRecMsg);
static uint16_t copyArmsRunDatas(BASE::ARMS_R_USE_MSG mRecMsg, BASE::ReadRunAllData &mRunDatas);
static int32_t checkHardError(uint16_t mStatusCode);
//pTmodule cmd or data send to client
//static int motorMoveAllCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors, uint8_t mCtrl);
static int motorMovePosXYZCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::VEL_4 mPos);
static int motorMoveVXYZCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::VEL_4 mPos);
static int motorMoveVXYCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::VEL_4 mVel);
static int motorMoveVXYZWCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::VEL_4 mVel);
static int motorMoveVWCmd(BASE::ARMS_THREAD_INFO *pTModule, float mVel);
static int motorMoveVZCmd(BASE::ARMS_THREAD_INFO *pTModule, float mVel);
static int motorMoveICmd(BASE::ARMS_THREAD_INFO *pTModule, float mVel, uint8_t mCtrl, int8_t mMotorIndex);
static int motorAllStopCmd(BASE::ARMS_THREAD_INFO *pTModule);

static int motorCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors);

static BASE::ACC_2 pidGetDa(BASE::POS_2 mNowL, BASE::POS_2 mLastL, float dT);
static BASE::POS_2 getPosBaseAngle(BASE::ANGLE_2 mNowAngle, float ropeL);
static BASE::ANGLE_2 getEndPosBySikoXY(float mSikoX, float mSikoY, float mRope, float mSRope);


static float readTensionValue(BASE::ARMS_THREAD_INFO *pTModule);

//app running
static int32_t initFire(BASE::ARMS_THREAD_INFO *pTModule);
static int32_t confFire(BASE::ARMS_THREAD_INFO *pTModule);
static int32_t confCondFire(BASE::ARMS_THREAD_INFO *pTModule);
static int32_t runFire(BASE::ARMS_THREAD_INFO *pTModule);

//算法随动、拉力算法
static int32_t pullMagic(BASE::ARMS_THREAD_INFO *pTModule);
static int32_t followagic(BASE::ARMS_THREAD_INFO *pTModule);

static int32_t getV(BASE::ARMS_THREAD_INFO *pTModule, float &mVel);

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


static int32_t findMotorNum(int32_t index)
{
  int iRet = 0;
  if(index == 0)iRet = MOTOR_X;
  if(index == 1)iRet = MOTOR_Y;
  if(index == 2)iRet = MOTOR_Z;
  if(index == 3)iRet = MOTOR_W;
  return iRet;
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

  setFdNonblocking(pTModule->mSocket);
  //setFdTimeout(pTModule->mSocket, CONF::SERVER_UDP_TIMEOUT_S, CONF::SERVER_UDP_TIMEOUT_US);

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
  motorAllStopCmd(pTModule);
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
* 功能：标定编码器0值，设置重物的初始重量，调用此函数时必须将重物吊起来，编码器使用弹簧拉正
* @param pTModule : pTModule是结构体中。
* @return Descriptions
******************************************************************************/
static void calibrationSensors(BASE::ARMS_THREAD_INFO *pTModule)
{
  if(pTModule->mMagicControl.mIsEncoderZero == 0)
  {
    pTModule->mMagicControl.mEncoderZero = pTModule->mRecMsg.mEncoderTurns;
    pTModule->mMagicControl.mIsEncoderZero = 1;
    printf("****  %d \n", pTModule->mRecMsg.mEncoderTurns);
  }

  if(pTModule->mMagicControl.mIsTensionZero == 0)
  {
    pTModule->mMagicControl.mM = ((float)pTModule->mRecMsg.mTension)/1000.0;
    pTModule->mMagicControl.mIsTensionZero = 1;

    //计算算法运行初始值
    pTModule->mMagicControl.mJ = 1.5*pow((pTModule->mMagicControl.mL/2), 2.0);
    pTModule->mMagicControl.mK1 = pTModule->mMagicControl.mM* pTModule->mMagicControl.mL + pTModule->mMagicControl.mJ/pTModule->mMagicControl.mL;
    printf("****  %f \n", pTModule->mMagicControl.mM);
  }

  //设置随动算法初始末端位置
  //pTModule->mMagicControl.mRopeEndL.x = pTModule->mRecUseMsg.mMotors[0].mPosition;
  //pTModule->mMagicControl.mRopeEndL.y = pTModule->mRecUseMsg.mMotors[1].mPosition;
  pTModule->mMagicControl.mRopeEndL.x = 0;
  pTModule->mMagicControl.mRopeEndL.y = 0;
  memcpy((char*)&pTModule->mMagicControl.mRopeEndLastL, (char*)&pTModule->mMagicControl.mRopeEndL, sizeof(BASE::POS_2));


  //初始化滤波数组
  for (int i=0; i<20; ++i)
  {
    pTModule->mMagicControl.magic_Xv[i] = 0;
    pTModule->mMagicControl.magic_Yv[i] = 0;
  }
}

/******************************************************************************
* 功能：此函数将采集机械臂板子上数据转换成标准单位，每个周期都会调用
* @param pTModule : pTModule是结构体中。
* @return Descriptions
******************************************************************************/
static void reformRecMsg(BASE::ARMS_THREAD_INFO *pTModule)
{

  //printf("cyc read lift-------------V1:%d V2:%d V3:%d V4:%d  编码器:%d 拉力计:%f\n", pTModule->mRecMsg.mMotors[0].mSpeed, pTModule->mRecMsg.mMotors[1].mSpeed,pTModule->mRecMsg.mMotors[2].mSpeed,pTModule->mRecMsg.mMotors[3].mSpeed,
  //                                                              pTModule->mRecMsg.mEncoderTurns, pTModule->mRecUseMsg.mTension);

  //编码器重整,
  if(pTModule->mRecMsg.mEncoderTurns < DEF_SYS_ENCODER_MAX)
    pTModule->mRecMsg.mEncoderTurns += 360000;
  pTModule->mRecMsg.mEncoderTurns -= pTModule->mMagicControl.mEncoderZero;


  //数据转换成浮点型
  for (int i=0; i<4; ++i)
  {
    pTModule->mRecUseMsg.mMotors[i].mSpeed = ((float)pTModule->mRecMsg.mMotors[findMotorNum(i)].mSpeed)*0.01745;
    //TUDU
    pTModule->mRecUseMsg.mMotors[i].mPosition = ((float)pTModule->mRecMsg.mMotors[findMotorNum(i)].mPosition);
  }

  pTModule->mRecUseMsg.mEncoderTurns = ((float)pTModule->mRecMsg.mEncoderTurns)*0.00001745;
  pTModule->mRecUseMsg.mTension      = ((float)pTModule->mRecMsg.mTension) * 0.0098;

  //m单位
  pTModule->mRecUseMsg.mSiko1       = ((float)pTModule->mRecMsg.mSiko1)*0.01*0.001;
  pTModule->mRecUseMsg.mSiko2       = ((float)pTModule->mRecMsg.mSiko2)*0.01*0.001;

  //printf("%f %d, %f %d\n", pTModule->mRecUseMsg.mSiko1,pTModule->mRecMsg.mSiko1, pTModule->mRecUseMsg.mSiko2,pTModule->mRecMsg.mSiko2);


  //采集三周期数据
  //存储摆动角、拉力值
  for (int i=2; i>=1; --i)
  {
    pTModule->mMagicControl.alfa_reco[i] = pTModule->mMagicControl.alfa_reco[i-1];
    pTModule->mMagicControl.F_reco[i] = pTModule->mMagicControl.F_reco[i-1];
  }

  //编码器滤波,等新的编码器过来后测试
  /*
  if(abs(pTModule->mRecUseMsg.mEncoderTurns - pTModule->mMagicControl.alfa_reco[1]) > 0.005)
    pTModule->mMagicControl.alfa_reco[0] = pTModule->mMagicControl.alfa_reco[1];
  else
    pTModule->mMagicControl.alfa_reco[0] = pTModule->mRecUseMsg.mEncoderTurns;
  */

  pTModule->mMagicControl.alfa_reco[0] = pTModule->mRecUseMsg.mEncoderTurns;
  pTModule->mMagicControl.F_reco[0] = readTensionValue(pTModule);
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

  //LOGER::PrintfLog(BASE::S_APP_LOGER, "send time(us) :%d", pMsg->mSysTime.mSysTimeUs);
  //mSysSendTime.mSysTimeS = pMsg->mSysTime.mSysTimeS;
  //mSysSendTime.mSysTimeUs = pMsg->mSysTime.mSysTimeUs;

  //TUDO

  pMsg->mCrcCode = 0;

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
/*
static int motorMoveAllCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::MOTORS &mMotors, uint8_t mCtrl)
{
  int32_t iRet = 0;
  BASE::MOTORS tMotors;
  //ctrl cmd motor
  for(int i=0; i<4; i++)
  {
    mMotors.mMotorsCmd[i].mCmd = mCtrl;
    tMotors.mMotorsCmd[findMotorNum(i)].mCmd = mMotors.mMotorsCmd[i].mCmd;
  }
  iRet = motorCmd(pTModule, mMotors);

  return  iRet;
}
*/

/******************************************************************************
* 功能：机械臂控制函数，发送控制指令。只控制XYZ轴运动，没有控制拉力电机
* @param pMsg : pTModule是线程信息结构体指针，里边存储的线程运行期间用到的数据和交换通信数据
* @param mMotors : mMotors是电机控制数据，
* @param mCtrl : mCtrl是电机控制命令，cmd
* @param mDirection : mDirection是电机转动方向
* @param mPosOrVel : mPosOrVel是电机位置控制，还是速度控制
* @return Descriptions
******************************************************************************/
static int motorMovePosXYZCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::VEL_4 mPos)
{
    int32_t iRet = 0;
    BASE::MOTORS tMotor = {0};
    //ctrl cmd motor
    for(int i=0; i<3; i++)
    {
      tMotor.mMotorsCmd[findMotorNum(i)].mCmd = BASE::CT_MOTOR_MOVE_POS;
      tMotor.mMotorsCmd[findMotorNum(i)].mPosition = mPos.v_p[i];
    }
    tMotor.mMotorsCmd[MOTOR_W].mCmd = BASE::CT_MOTOR_STOP;

    iRet = motorCmd(pTModule, tMotor);
    return  iRet;
}

static int motorMoveVXYZCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::VEL_4 mVel)
{
    int32_t iRet = 0;
    BASE::MOTORS mMotors = {0};
    //ctrl cmd motor
    for(int i=0; i<3; i++)
    {
      mMotors.mMotorsCmd[findMotorNum(i)].mCmd = BASE::CT_MOTOR_MOVE_POS;
      mMotors.mMotorsCmd[findMotorNum(i)].mSpeed = (int32_t)(mVel.v_p[i] * DEF_SYS_RADIAN_TO_PULSE);
    }
    mMotors.mMotorsCmd[MOTOR_W].mCmd = BASE::CT_MOTOR_STOP;
    iRet = motorCmd(pTModule, mMotors);
    return  iRet;
}
static int motorMoveVXYCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::VEL_4 mVel)
{
    int32_t iRet = 0;
    BASE::MOTORS mMotors = {0};
    //ctrl cmd motor
    mMotors.mMotorsCmd[MOTOR_X].mCmd = BASE::CT_MOTOR_MOVE_POSITIVE;
    mMotors.mMotorsCmd[MOTOR_Y].mCmd = BASE::CT_MOTOR_MOVE_POSITIVE;
    //输入速度是m/s,需要换算成 脉冲/s
    mMotors.mMotorsCmd[MOTOR_X].mSpeed = (int32_t)(mVel.v_p[0] * DEF_SYS_RADIAN_TO_PULSE);
    mMotors.mMotorsCmd[MOTOR_Y].mSpeed = (int32_t)(mVel.v_p[1] * DEF_SYS_RADIAN_TO_PULSE);

    mMotors.mMotorsCmd[MOTOR_Z].mCmd = BASE::CT_MOTOR_STOP;
    mMotors.mMotorsCmd[MOTOR_W].mCmd = BASE::CT_MOTOR_STOP;
    iRet = motorCmd(pTModule, mMotors);
    return  iRet;
}

static int motorMoveVWCmd(BASE::ARMS_THREAD_INFO *pTModule, float mVel)
{
    return  motorMoveICmd(pTModule, mVel, BASE::CT_MOTOR_MOVE_POSITIVE, MOTOR_W);
}

static int motorMoveVZCmd(BASE::ARMS_THREAD_INFO *pTModule, float mVel)
{
    return  motorMoveICmd(pTModule, mVel, BASE::CT_MOTOR_MOVE_POSITIVE, MOTOR_Z);
}

//只控制Z W电机
static int motorMoveICmd(BASE::ARMS_THREAD_INFO *pTModule, float mVel, uint8_t mCtrl, int8_t mMotorIndex)
{
    int32_t iRet = 0;
    BASE::MOTORS mMotors = {0};
    //ctrl cmd motor. speed 单位是 脉冲/s
    for (int i=0; i<4; ++i)
      mMotors.mMotorsCmd[i].mCmd = BASE::CT_MOTOR_STOP;

    mMotors.mMotorsCmd[mMotorIndex].mCmd = mCtrl;
    mMotors.mMotorsCmd[mMotorIndex].mSpeed = (int32_t)(mVel * DEF_SYS_RADIAN_TO_PULSE);
    //printf("v:%d %d %d %d\n",mMotors.mMotorsCmd[0].mSpeed,mMotors.mMotorsCmd[1].mSpeed,mMotors.mMotorsCmd[2].mSpeed,mMotors.mMotorsCmd[3].mSpeed);
    iRet = motorCmd(pTModule, mMotors);
    return  iRet;
}

static int motorAllStopCmd(BASE::ARMS_THREAD_INFO *pTModule)
{
    int32_t iRet = 0;
    BASE::MOTORS mMotors = {0};
    //ctrl cmd motor. speed 单位是 脉冲/s
    for (int i=0; i<4; ++i)
    {
      mMotors.mMotorsCmd[i].mCmd = BASE::CT_MOTOR_STOP;
    }

    //printf("v:%d %d %d %d\n",mMotors.mMotorsCmd[0].mSpeed,mMotors.mMotorsCmd[1].mSpeed,mMotors.mMotorsCmd[2].mSpeed,mMotors.mMotorsCmd[3].mSpeed);
    iRet = motorCmd(pTModule, mMotors);
    return  iRet;
}

static int motorMoveVXYZWCmd(BASE::ARMS_THREAD_INFO *pTModule, BASE::VEL_4 mVel)
{
    int32_t iRet = 0;
    BASE::MOTORS mMotors = {0};
    //ctrl cmd motor. speed 单位是 脉冲/s
    for (int i=0; i<4; ++i)
    {
      mMotors.mMotorsCmd[i].mCmd = BASE::CT_MOTOR_MOVE_POSITIVE;
      mMotors.mMotorsCmd[findMotorNum(i)].mSpeed = (int32_t)(mVel.v_p[i]*DEF_SYS_RADIAN_TO_PULSE);
    }

    //printf("v:%d %d %d %d\n",mMotors.mMotorsCmd[0].mSpeed,mMotors.mMotorsCmd[1].mSpeed,mMotors.mMotorsCmd[2].mSpeed,mMotors.mMotorsCmd[3].mSpeed);
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
static float readTensionValue(BASE::ARMS_THREAD_INFO *pTModule)
{
  float iRet = -1;
  //如果DEF_SYS_USE_TENSIONLEADER_NUMS=0 则不采用无线模块，拉力计信息在RecMsg里边，否则拉力计信息需要无线模块传输。
  iRet = ((DEF_SYS_USE_TENSIONLEADER_NUMS == 0) ? iRet = pTModule->mRecUseMsg.mTension : pTModule->mNowTension->mTensions);
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

  uint16_t mMotorState = checkMotorsState(pTModule->mRecMsg);
  LOGER::PrintfLog(BASE::S_APP_LOGER, "motor state :%d", mMotorState);
  //check motor is power on
  if(mMotorState == 0) // motor 0000  all start,then change state
  {
    if(pTModule->mAckState != BASE::ACK_STATE_INIT_OK)
      pTModule->mAckState = BASE::ACK_STATE_INIT_OK;
    printf("init motor cmd\n");
  }
  else //some motors is stop
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "send stop to motors!");
  }
  //编码器找零值
  calibrationSensors(pTModule);
  //发送电机停止，
  motorAllStopCmd(pTModule);
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
      //位置控制,暂时没有，机械臂板子只有速度控制
      //速度控制
      //printf("%f %f %f %f %f %f %f %f\n", pTModule->mIntCmd.mCmdV.v_p[0], pTModule->mIntCmd.mCmdV.v_p[1], pTModule->mIntCmd.mCmdV.v_p[2], pTModule->mIntCmd.mCmdV.v_p[3],
      //                                    pTModule->mRecUseMsg.mMotors[0].mSpeed, pTModule->mRecUseMsg.mMotors[1].mSpeed, pTModule->mRecUseMsg.mMotors[2].mSpeed, pTModule->mRecUseMsg.mMotors[3].mSpeed);
      motorMoveVXYZWCmd(pTModule, pTModule->mIntCmd.mCmdV);
      break;
    }
    case BASE::ST_ALL_MOVE_RUNNING:
    {
      if(fabs(pTModule->mRecUseMsg.mMotors[0].mPosition-pTModule->mIntCmd.mCmdPosXYZ.v_p[0]) > 0.001 &&
         fabs(pTModule->mRecUseMsg.mMotors[1].mPosition-pTModule->mIntCmd.mCmdPosXYZ.v_p[1]) > 0.001 &&
         fabs(pTModule->mRecUseMsg.mMotors[2].mPosition-pTModule->mIntCmd.mCmdPosXYZ.v_p[2]) > 0.001)
      {
          motorMovePosXYZCmd(pTModule, pTModule->mIntCmd.mCmdPosXYZ);
          printf("conf motor all move postion:%f, now postion:%f\n",  pTModule->mIntCmd.mCmdPosXYZ.v_p[0], pTModule->mRecUseMsg.mMotors[0].mPosition);
          //send rec running data to uper
      }
      else
      {
          motorAllStopCmd(pTModule);
          setStateCond(pTModule->mCond, BASE::M_STATE_CONF, BASE::ST_LIFT_STOPED, 0, 0);
      }
      break;
    }
    case BASE::ST_ALL_PULL_RUNNING:
    {
      //pull 命令，根据拉立计信息慢慢的增加电机转速。
      float  tensiosKg = readTensionValue(pTModule)/9.8;
      float  cmdKg  =  pTModule->mIntCmd.mCmdTension*pTModule->mMagicControl.mM;
      if(fabs(tensiosKg-cmdKg) > 0.05)
      {
        //TUDO 正转  倒转
        if(tensiosKg > cmdKg)
            motorMoveVWCmd(pTModule, 0.01);
        else
            motorMoveVWCmd(pTModule, -0.01);
         printf("conf motor all pull move:set tension %f, now tension:%f, state:%d\n",  pTModule->mIntCmd.mCmdTension, tensiosKg, pTModule->mIsNowMotorCmd);
      }
      else
      {
          motorAllStopCmd(pTModule);
          setStateCond(pTModule->mCond, BASE::M_STATE_CONF, BASE::ST_LIFT_STOPED, 0, 0);
      }
      break;
    }
    case BASE::ST_LIFT_STOPED:
    {
      motorAllStopCmd(pTModule);
      printf("lift stop cmd....\n");
      break;
    }
    //run mode
    case BASE::ST_ALL_RUN_RUNNING:
    {
      static int ii = 0;
      ii++;
      float tCmdV = 0.0;

      //TUDO PID ctrl move to (x y z)
      followagic(pTModule);

      //平面XY随动控制
      motorMoveVXYCmd(pTModule, pTModule->mMagicControl.mCmdV);

      printf("ixv:%f iyv:%f oxv:%f oyv:%f 兹山尺x:%f 兹山尺y:%f\n", pTModule->mMagicControl.mCmdV.v_p[0], pTModule->mMagicControl.mCmdV.v_p[1], pTModule->mRecUseMsg.mMotors[0].mSpeed, pTModule->mRecUseMsg.mMotors[1].mSpeed,
                                                                  pTModule->mRecUseMsg.mSiko1,pTModule->mRecUseMsg.mSiko2);

      //拉力控制算法
      /*
      //缓冲200个周期在启动控制算法
      if(ii > 200)
      {
          pullMagic(pTModule);//函数计算出收放收缩加速度。
          //这个函数必须在pullMagic或者followMagic函数之后调用
          getV(pTModule, tCmdV);
          ii = 600;
      }
      //算法降速
      if(tCmdV > 30)
          tCmdV = 30;
      if(tCmdV < -30)
          tCmdV = -30;

      LOGER::PrintfLog(BASE::S_ARMS_DATA, "%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
                       ii, pTModule->mRecUseMsg.mSiko1, pTModule->mRecUseMsg.mSiko2, pTModule->mRecUseMsg.mInclinometer1_x, pTModule->mRecUseMsg.mInclinometer1_y,
                       pTModule->mRecUseMsg.mEncoderTurns, pTModule->mRecUseMsg.mTension, pTModule->mRecUseMsg.mTension,
                       pTModule->mRecUseMsg.mMotors[0].mSpeed, pTModule->mRecUseMsg.mMotors[1].mSpeed, pTModule->mRecUseMsg.mMotors[2].mSpeed, pTModule->mRecUseMsg.mMotors[3].mSpeed,
                       pTModule->mRecUseMsg.mMotors[0].mSpeed, tCmdV, pTModule->mRecUseMsg.mMotors[0].mSpeed, pTModule->mRecUseMsg.mMotors[0].mSpeed
                       );

      motorMoveVZCmd(pTModule, tCmdV);
      */

      LOGER::PrintfLog(BASE::S_ARMS_DATA, "%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f",
                       ii, pTModule->mRecUseMsg.mSiko1, pTModule->mRecUseMsg.mSiko2, pTModule->mRecUseMsg.mInclinometer1_x, pTModule->mRecUseMsg.mInclinometer1_y,
                       pTModule->mRecUseMsg.mEncoderTurns, pTModule->mRecUseMsg.mTension, pTModule->mRecUseMsg.mTension,
                       pTModule->mRecUseMsg.mMotors[0].mSpeed, pTModule->mRecUseMsg.mMotors[1].mSpeed, pTModule->mRecUseMsg.mMotors[2].mSpeed, pTModule->mRecUseMsg.mMotors[3].mSpeed,
                       pTModule->mRecUseMsg.mMotors[0].mSpeed, tCmdV, pTModule->mRecUseMsg.mMotors[0].mSpeed, pTModule->mRecUseMsg.mMotors[0].mSpeed
                       );
      break;
    }
    case BASE::ST_RUN_STOPED:
    {
      motorAllStopCmd(pTModule);
      printf("run stop cmd....  send:%d , rec:%d\n", pTModule->mRandomCode, pTModule->mRecMsg.mRandomCode);
      //run模式停止的话自动退回conf模式

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

  //TUDO 暂时不用检查电机错误，消息状态玛出错包含电机出错
  //检查电机是否报错
  /*
  uint16_t mMotorState = checkMotorsState(pTModule->mRecMsg);
  if(mMotorState != 0) // some motor stop. then stop all modules
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "***conf state: have motor stop. motor state:%d", mMotorState);
    //stop all modules
    pTModule->mState = BASE::M_STATE_STOP;
    return -2;
  }
  */

/**************************** TUDU send cmd to motor*************************/
  //接收新的命令
  switch (pTModule->mIsNowMotorCmd)
  {
    //上位机发来的cmd是不是停止命令，如果是的话就停止电机转动
    case BASE::CMD_ALL_MOVE_STOP:
    case BASE::CMD_ALL_PULL_STOP:
    case BASE::CMD_HAND_MOVE_STOP:
    {
      setStateCond(pTModule->mCond, BASE::M_STATE_CONF, BASE::ST_LIFT_STOPED, 0, 0);
      printf("conf motor cmd stop\n");
      LOGER::PrintfLog(BASE::S_APP_LOGER, "***conf state: have motor stop. motor state:%d", pTModule->mIsNowMotorCmd);
      pTModule->mIsNowMotorCmd = BASE::CMD_INTO_COND_RUN;
      break;
    }
    case BASE::CMD_HAND_MOVE_START:
    {
      BASE::LiftCmdData lMovedata = {0};
      pthread_mutex_lock(&pTModule->mMotorMutex);
      memcpy((char*)&lMovedata, (char*)&pTModule->mMoveData, sizeof(BASE::LiftCmdData));
      pthread_mutex_unlock(&pTModule->mMotorMutex);

      //if Manual ctrl move mode than move (xyz), else move (0,0,0)(relative position)
      memset((char*)&pTModule->mIntCmd.mCmdV, 0, sizeof(pTModule->mIntCmd.mCmdV));
      //速度控制
      if(lMovedata.isVevOrPos == 0)
      {
        memcpy((char*)pTModule->mIntCmd.mCmdV.v_p, (char*)lMovedata.v_p, sizeof(float)*4);
      }
      printf("-----------------%f \n", pTModule->mIntCmd.mCmdV.v_p[0]);
      setStateCond(pTModule->mCond, BASE::M_STATE_CONF, BASE::ST_HAND_MOVE_RUNNING, 0, 0);
      pTModule->mIsNowMotorCmd = BASE::CMD_INTO_COND_RUN;
      break;
    }
    case BASE::CMD_ALL_MOVE_START:
    {
      BASE::LiftCmdData lMovedata = {0};
      pthread_mutex_lock(&pTModule->mMotorMutex);
      memcpy((char*)&lMovedata, (char*)&pTModule->mMoveData, sizeof(BASE::LiftCmdData));
      pthread_mutex_unlock(&pTModule->mMotorMutex);

      //if Manual ctrl move mode than move (xyz), else move (0,0,0)(relative position)
      memset((char*)&pTModule->mIntCmd.mCmdPosXYZ, 0, sizeof(pTModule->mIntCmd.mCmdPosXYZ));
      if(lMovedata.isVevOrPos == 1)
      {
        memcpy((char*)pTModule->mIntCmd.mCmdPosXYZ.v_p, (char*)lMovedata.v_p, sizeof(float)*4);
      }
      setStateCond(pTModule->mCond, BASE::M_STATE_CONF, BASE::ST_ALL_MOVE_RUNNING, 0, 0);
      pTModule->mIsNowMotorCmd = BASE::CMD_INTO_COND_RUN;
      break;
    }
    case BASE::CMD_ALL_PULL_START:
    {
      //TUDO
      BASE::LiftCmdData lPulldata = {0};
      pthread_mutex_lock(&pTModule->mMotorMutex);
      memcpy((char*)&lPulldata, (char*)&pTModule->mMoveData, sizeof(BASE::LiftCmdData));
      pthread_mutex_unlock(&pTModule->mMotorMutex);

      pTModule->mIntCmd.mCmdTension = lPulldata.v_p[3];


      //if Manual ctrl move mode than move (xyz), else move (0,0,0)(relative position)
      //memset((char*)&mNowMotors, 0, sizeof(mNowMotors));
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
      LOGER::PrintfLog(BASE::S_APP_LOGER,"wait cmd....  send:%d , rec:%d", pTModule->mRandomCode, pTModule->mRecMsg.mRandomCode);
      motorAllStopCmd(pTModule);

      /*
      static int mm = 0;
      mm++;

      static float dv =0;

      if(dv > 230)
          dv = 231;
      else
      {
          dv = mm * 1.5;
      }

      dv = 50;
      motorMoveVZCmd(pTModule, dv);
      printf("%d %f %f\n",mm, dv, pTModule->mRecUseMsg.mMotors[2].mSpeed);
      */
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
static uint16_t copyArmsRunDatas(BASE::ARMS_R_USE_MSG mRecUseMsg, BASE::ReadRunAllData &mRunDatas)
{
  mRunDatas.mIsValid = 1;
  mRunDatas.runD_Rsiko1 = mRecUseMsg.mSiko1;
  mRunDatas.runD_Rsiko2 = mRecUseMsg.mSiko2;
  mRunDatas.runD_Level1 = mRecUseMsg.mInclinometer1_x;
  mRunDatas.runD_Level2 = mRecUseMsg.mInclinometer1_y;
  mRunDatas.runR_sysMsg_X = mRecUseMsg.mMotors[0].mPosition;
  mRunDatas.runR_sysMsg_Y = mRecUseMsg.mMotors[1].mPosition;
  mRunDatas.runR_sysMsg_Z = mRecUseMsg.mMotors[2].mPosition;

  mRunDatas.runD_RencoderT = mRecUseMsg.mEncoderTurns;
  //mRunDatas.runD_RencoderXNow = mRecMsg.mMotors[0].mEncoderTurns;
  //mRunDatas.runD_RencoderYNow = mRecMsg.mMotors[1].mEncoderTurns;
  //mRunDatas.runD_RencoderZNow = mRecMsg.mMotors[2].mEncoderTurns;
}

/******************************************************************************
* 功能：根据摆动角度，绳索长度得到位置
* @param mNowAngle : mNowAngle是绳索X Y方向角度，单位弧度
* @param ropeL : ropeL是绳索的长度单位m
* @return Descriptions
******************************************************************************/
static BASE::POS_2 getPosBaseAngle(BASE::ANGLE_2 mNowAngle, float ropeL)
{
    BASE::POS_2 iRet = {0,0};
    iRet.x = ropeL*sin(mNowAngle.x);
    iRet.y = ropeL*sin(mNowAngle.y);
    return iRet;
}

static BASE::ANGLE_2 getEndPosBySikoXY(float mSikoX, float mSikoY, float mRope, float mSRope)
{
    BASE::POS_2 iRet = {0,0};
    iRet.x = mSikoX * (mRope/mSRope);
    iRet.y = mSikoY * (mRope/mSRope);
    return iRet;
}

/******************************************************************************
* 功能：随动控制PID算法
* @param angleX : angleX是绳索X方向角度，单位弧度
* @param angleY : angleY是绳索Y方向角度，单位弧度
* @param ropeL : ropeL是绳索的长度单位m
* @param dT : dT是控制周期，单位s
* @return Descriptions
******************************************************************************/
static BASE::ACC_2 pidGetDa(BASE::POS_2 mNowL, BASE::POS_2 mLastL, float dT)
{
    BASE::ACC_2 iRet;

    iRet.x = CONF::PID_P_FOLLOWUP * mNowL.x + CONF::PID_D_FOLLOWUP*(mNowL.x-mLastL.x)/dT;
    iRet.y = CONF::PID_P_FOLLOWUP * mNowL.y + CONF::PID_D_FOLLOWUP*(mNowL.y-mLastL.y)/dT;

    return iRet;
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

  //TUDO 暂时不用检查电机错误，消息状态玛出错包含电机出错
  /*
  //check motor if running
  uint16_t mMotorState = checkMotorsState(pTModule->mRecMsg);
  if(mMotorState != 0) // some motor stop. then stop all modules
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "***run state: have motor stop. motor state:%d", mMotorState);
    //stop all modules
    pTModule->mState = BASE::M_STATE_STOP;
    return -2;
  }
  */

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
      motorAllStopCmd(pTModule);
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

/******************************************************************************
* 功能：拉力平衡算法
* @param pTModule : pTModule是线程结构体
* @return Descriptions
******************************************************************************/
static int32_t checkRecMsgError(BASE::ARMS_THREAD_INFO *pTModule)
{
  static int32_t iRet = 0;
  //检查拉力是否出错
  float tensiosV = readTensionValue(pTModule);
  //float mTempL =  (float)pTModule->mRecMsg.mTension;
  if(tensiosV < 0 || (tensiosV/(pTModule->mMagicControl.mM*10)) < 0.5)
  {
    iRet = -1;
    printf("chaochu:%f  %f\n", tensiosV, (tensiosV/(pTModule->mMagicControl.mM*1000)));
  }

  return iRet;
}

/******************************************************************************
* 功能：拉力平衡算法
* @param pTModule : pTModule是线程结构体
* @return Descriptions
******************************************************************************/
static int32_t pullMagic(BASE::ARMS_THREAD_INFO *pTModule)
{
  BASE::MagicControlData *pControl = &(pTModule->mMagicControl);
  //计算拉力 摆动角度导数
  pControl->d_f_measure = (pControl->F_reco[0] - pControl->F_reco[1])/pControl->dt;
  pControl->d_alfi_measure = (pControl->alfa_reco[0] - pControl->alfa_reco[1])/pControl->dt;
  pControl->ddalfi_measure = (pControl->alfa_reco[0] - 2*pControl->alfa_reco[1] + pControl->alfa_reco[2])/(pControl->dt*pControl->dt);

  //printf("************* ddalfi_measure:%f alfa_reco:%f %f %f\n",
  //                       pControl->ddalfi_measure, pControl->alfa_reco[0], pControl->alfa_reco[1], pControl->alfa_reco[2]);

  float alfa_m = pControl->alfa_reco[0];
  if(fabs(alfa_m) < 1.5/60)
  {
    alfa_m = 0;
  }
  //对外力进行估算
  pControl->f_estimate = -pControl->mK1 * pControl->ddalfi_measure - pControl->mM*pControl->last_dd_Lz - pControl->mK*pControl->mL*alfa_m;




  pControl->dd_Lz = (-pControl->f_estimate + 0.2*pControl->d_f_measure)/pControl->mM  +
                    0.2*((2*pControl->mCo*pControl->mWn*pControl->d_alfi_measure+pow(pControl->mWn,2)*alfa_m)*pControl->mK1 - pControl->mK*pControl->mL*alfa_m)/pControl->mM;


  //printf("mK1:%f mM:%f last_dd_Lz:%f mK:%f  mL:%f\n",
  //        pControl->mK1, pControl->mM, pControl->last_dd_Lz, pControl->mK, pControl->mL);

  //pControl->dd_Lz = (-pControl->f_estimate + (pControl->F_reco[0]-pControl->mM*pControl->mG)*2 + 0.4*pControl->d_f_measure)/pControl->mM;

  //加速度限制超过1
  if(pControl->dd_Lz > 5)
    pControl->dd_Lz = 5;
  if(pControl->dd_Lz < -5)
    pControl->dd_Lz = -5;
  //pControl->dd_Lz = (pControl->dd_Lz > 2 ? 2 : (pControl->dd_Lz < -2 ? -2 : pControl->dd_Lz));

  //得到电机的加速度
  pControl->last_dd_Lz = pControl->dd_Lz;
  //电机的加速度
  pControl->d_w = pControl->dd_Lz/pControl->mR * pControl->mNdecrease;

  printf("dd_Lz:%f f_estimate:%f d_f_measure:%f d_alfi_measure:%f  alfa_m:%f alfa_now:%f  d_w:%f  f_re:%f %f %f\n",
          pControl->dd_Lz, pControl->f_estimate, pControl->d_f_measure, pControl->d_alfi_measure, alfa_m, pControl->alfa_reco[0],pControl->d_w, pControl->F_reco[0], pControl->F_reco[1],pControl->F_reco[2]);
}
static int32_t followagic(BASE::ARMS_THREAD_INFO *pTModule)
{
  //需要根据磁栅尺计算当前XY偏角
  pTModule->mMagicControl.mRopeEndL = getEndPosBySikoXY(pTModule->mRecUseMsg.mSiko1, pTModule->mRecUseMsg.mSiko2, 1.2, 0.6);
  BASE::ACC_2 mAcc = pidGetDa(pTModule->mMagicControl.mRopeEndL, pTModule->mMagicControl.mRopeEndLastL, 0.01);
  pTModule->mMagicControl.mRopeEndLastL = pTModule->mMagicControl.mRopeEndL;

  BASE::ACC_2 mAngelAcc;
  mAngelAcc.x = mAcc.x/0.025 * 30;
  mAngelAcc.y = mAcc.y/0.025 * 30;

  pTModule->mMagicControl.mCmdV.v_p[0] = pTModule->mRecUseMsg.mMotors[0].mSpeed + mAngelAcc.x*0.01;
  pTModule->mMagicControl.mCmdV.v_p[1] = pTModule->mRecUseMsg.mMotors[1].mSpeed + mAngelAcc.y*0.01;

  if(pTModule->mMagicControl.mCmdV.v_p[0] > 50)
      pTModule->mMagicControl.mCmdV.v_p[0] = 50;
  if(pTModule->mMagicControl.mCmdV.v_p[0] < -50)
      pTModule->mMagicControl.mCmdV.v_p[0] = -50;

  if(pTModule->mMagicControl.mCmdV.v_p[1] > 50)
      pTModule->mMagicControl.mCmdV.v_p[1] = 50;
  if(pTModule->mMagicControl.mCmdV.v_p[1] < -50)
      pTModule->mMagicControl.mCmdV.v_p[1] = -50;

  //速度做均质滤波
  for (int i=1; i<5;i++)
  {
    pTModule->mMagicControl.mCmdV.v_p[0] += pTModule->mMagicControl.magic_Xv[i];
    pTModule->mMagicControl.mCmdV.v_p[1] += pTModule->mMagicControl.magic_Yv[i];
  }
  pTModule->mMagicControl.mCmdV.v_p[0] /= 5;
  pTModule->mMagicControl.mCmdV.v_p[1] /= 5;

  //存储之前速度
  for (int i=9; i>=0; --i)
  {
    pTModule->mMagicControl.magic_Xv[i] = pTModule->mMagicControl.magic_Xv[i-1];
    pTModule->mMagicControl.magic_Yv[i] = pTModule->mMagicControl.magic_Yv[i-1];
  }
  pTModule->mMagicControl.magic_Xv[0] = pTModule->mMagicControl.mCmdV.v_p[0];
  pTModule->mMagicControl.magic_Yv[0] = pTModule->mMagicControl.mCmdV.v_p[1];


  pTModule->mMagicControl.mCmdV.v_p[0] = 0;
  return 0;
}

static int32_t getV(BASE::ARMS_THREAD_INFO *pTModule, float &mVel)
{
  int32_t iRet = 0;
  //计算收放绳子电机要执行的速度，rad/s
  float mNowV = pTModule->mRecUseMsg.mMotors[2].mSpeed;
  //单位为 rad/s
  mVel = pTModule->mMagicControl.d_w*pTModule->mMagicControl.dt + mNowV;

  //速度做均质滤波
  for (int i=1; i<3;i++)
  {
    mVel += pTModule->mMagicControl.magic_v[i];
  }
  mVel /= 3;

  //存储之前速度
  for (int i=9; i>=0; --i)
  {
    pTModule->mMagicControl.magic_v[i] = pTModule->mMagicControl.magic_v[i-1];
  }
  pTModule->mMagicControl.magic_v[0] = mVel;

  //检查机械臂是否有错
  if(checkRecMsgError(pTModule) < 0)
  {
    printf("************* mVel:%f , error\n", mVel);
    //mVel.v[0] = 0;
  }
  //printf("************* mVel:%f > 31, error:%d d_w:%f mNowv: %f, d_w:%f, dt:%f \n", mVel.v[0], checkRecMsgError(pTModule),pTModule->mMagicControl.d_w,  mNowV,
  //                        pTModule->mMagicControl.d_w, pTModule->mMagicControl.dt);
  return iRet;
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

    reformRecMsg(pTModule);

    //calSysDelayed(pTModule->mReadLiftHzData, mSysSendTime, pTModule->mRecMsg.mSysTime);
    static float last1 = 0,last2 = 0;
    /*
    printf("%.3f %.3f %d %d V: (%f %f)\n", pTModule->mRecUseMsg.mSiko1, pTModule->mRecUseMsg.mSiko2, pTModule->mRecMsg.mSiko1, pTModule->mRecMsg.mSiko2,
                                           (pTModule->mRecUseMsg.mSiko1-last1)*100, (pTModule->mRecUseMsg.mSiko2-last2)*100);
    last1 = pTModule->mRecUseMsg.mSiko1;      last2 = pTModule->mRecUseMsg.mSiko2;
    */

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
            motorAllStopCmd(pTModule);
            LOGER::PrintfLog(BASE::S_APP_LOGER, "power on doing, wait:%d times, size:%d  %d", pTModule->mCond.mCycTimes, size, sizeof(pTModule->mRecMsg));
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
          LOGER::PrintfLog(BASE::S_APP_LOGER, "conf state: %s, size:%d client overtime or lost link. stop app", pTModule->mThreadName, size);
          //stop all modules
          pTModule->mState = BASE::M_STATE_QUIT;
          perror("this");
          break;
        }
        /*
        LOGER::PrintfLog(BASE::S_APP_LOGER, "时间:%d %d,随即码:%d,接近开关:%d,倾角仪:%d %d,磁删尺:%d %d,编码器:%d,拉计:%d,v1:%f,v2:%f,v3:%f,v4:%f",
                                             pTModule->mRecMsg.mSysTime.mSysTimeS, pTModule->mRecMsg.mSysTime.mSysTimeUs, pTModule->mRecMsg.mRandomCode,
                                             pTModule->mRecMsg.mSwitchTiggers,pTModule->mRecMsg.mInclinometer1_x, pTModule->mRecMsg.mInclinometer1_y,
                                             pTModule->mRecMsg.mSiko1, pTModule->mRecMsg.mSiko2,pTModule->mRecMsg.mEncoderTurns,pTModule->mRecMsg.mTension,
                                             pTModule->mRecMsg.mMotors[0].mSpeed, pTModule->mRecMsg.mMotors[2].mSpeed,pTModule->mRecMsg.mMotors[2].mSpeed,pTModule->mRecMsg.mMotors[3].mSpeed);
        */
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
        motorAllStopCmd(pTModule);
        //printf("motor stoped state...\n");
        break;
      }
      case BASE::M_STATE_QUIT:
      {
        motorAllStopCmd(pTModule);
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