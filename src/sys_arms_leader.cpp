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

static float readTensionValue(BASE::ARMS_THREAD_INFO *pTModule);

//app running
static int32_t initFire(BASE::ARMS_THREAD_INFO *pTModule);
static int32_t confFire(BASE::ARMS_THREAD_INFO *pTModule);
static int32_t confCondFire(BASE::ARMS_THREAD_INFO *pTModule);
static int32_t runFire(BASE::ARMS_THREAD_INFO *pTModule);

//算法随动、拉力算法
static int32_t pullMagic(BASE::ARMS_THREAD_INFO *pTModule);
static int32_t followagic(BASE::ARMS_THREAD_INFO *pTModule);
static float   deadZone(float mIndead, float mthr);

//
static int32_t runningPhase(BASE::ARMS_THREAD_INFO *pTModule);
static int32_t calibPhase(BASE::ARMS_THREAD_INFO *pTModule);

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

  //运行数zhi
  pTModule->mRCnt = 0;

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
  //计算算法运行初始值
  pTModule->mMagicControl.mJ = 1.5*pow((pTModule->mMagicControl.mL/2), 2.0);
  pTModule->mMagicControl.mK1 = pTModule->mConfParam->mConfSaveWeight * pTModule->mMagicControl.mL + pTModule->mMagicControl.mJ/pTModule->mMagicControl.mL;
  printf("****mM %f , mJ:%f, mK1:%f, mL:%f\n", pTModule->mConfParam->mConfSaveWeight, pTModule->mMagicControl.mJ,pTModule->mMagicControl.mK1,pTModule->mMagicControl.mL);

  pTModule->mMagicControl.mRopeEndLastL = {0,0};

  //初始化滤波数组
  memset((char*)&pTModule->magic_XYv, 0, sizeof(BASE::VEL_2)*(XYV_AVG_SIZE+1));
}

/******************************************************************************
* 功能：此函数将采集机械臂板子上数据转换成标准单位，每个周期都会调用
* @param pTModule : pTModule是结构体中。
* @return Descriptions
******************************************************************************/
static void reformRecMsg(BASE::ARMS_THREAD_INFO *pTModule)
{
  //编码器重整,
  if(pTModule->mRecMsg.mEncoderTurns > DEF_SYS_ENCODER_MAX)
    pTModule->mRecMsg.mEncoderTurns -= 360000;
  pTModule->mRecMsg.mEncoderTurns -= pTModule->mConfParam->mConfSaveEncoderT*1000.0;

  /*
  if(strcmp(pTModule->mThreadName, "MN_SERVER2") == 0)
  printf("**********%s encoder:%d\n",pTModule->mThreadName,pTModule->mRecMsg.mEncoderTurns);
  */
  //printf("tnesion:%d\n",pTModule->mRecMsg.mTension);
  //拉力计除以100，变成g
  pTModule->mRecMsg.mTension /= 100;

  //拉力计zuo jiaozhun
  pTModule->mRecMsg.mTension += pTModule->mConfParam->mConfTension;

  //pTModule->mRecMsg.mTension = 40000;
  //printf("拉力计:%d\n",pTModule->mRecMsg.mTension);
  int32_t mTempTension = pTModule->mRecMsg.mTension;

  //拉力计滤波
  for (int i=0; i<AVG_SIZE; i++)
  {
    pTModule->mTensionAvg[i] = pTModule->mTensionAvg[i+1];
    mTempTension += pTModule->mTensionAvg[i];
    //printf("mTempTens**********:%d\n",mTempTension);
  }
  pTModule->mTensionAvg[AVG_SIZE] = pTModule->mRecMsg.mTension;
  mTempTension /= (AVG_SIZE+1.0);

  //printf("mTempTens:%d\n",mTempTension);
  //数据转换成浮点型
  for (int i=0; i<4; ++i)
  {
    //
    pTModule->mRecUseMsg.mMotors[i].mSpeed = ((float)pTModule->mRecMsg.mMotors[findMotorNum(i)].mSpeed)*0.01745*pTModule->motorDirection[i];
    //TUDU
    pTModule->mRecUseMsg.mMotors[i].mPosition = ((float)((pTModule->mRecMsg.mMotors[findMotorNum(i)].mPosition)/1000));
    pTModule->mRecUseMsg.mMotors[i].mPosition /= 1000.0;
  }

  //X Y pos
  pTModule->mRecUseMsg.mMotors[0].mPosition = pTModule->mPoxR*pTModule->mRecUseMsg.mMotors[0].mPosition - pTModule->mPoxTx;
  pTModule->mRecUseMsg.mMotors[1].mPosition = pTModule->mPoxR*pTModule->mRecUseMsg.mMotors[1].mPosition - pTModule->mPoxTy;

  pTModule->mRecUseMsg.mMotors[2].mPosition = (-7.6*(pTModule->mRecUseMsg.mMotors[2].mPosition) - pTModule->mPoxTz);



  //printf("************pos:%d %d %d %d\n",pTModule->mRecMsg.mMotors[0].mPosition/10000,pTModule->mRecMsg.mMotors[1].mPosition/10000,
  //                                       pTModule->mRecMsg.mMotors[2].mPosition/10000,pTModule->mRecMsg.mMotors[3].mPosition/10000);

  //限位开关
  pTModule->mRecUseMsg.mSwitchStateCode = pTModule->mRecMsg.mSwitchStateCode;
  pTModule->mRecUseMsg.mSwitchTiggers = pTModule->mRecMsg.mSwitchTiggers;

  //编码器转换成国际单位
  float tEncoderTurns = ((float)pTModule->mRecMsg.mEncoderTurns)*0.00001745;
  pTModule->mRecUseMsg.mEncoderTurns = tEncoderTurns;
  pTModule->mRecUseMsg.mTension      = ((float)mTempTension) * 0.0098;

  //encoder avg
  for (int i=0; i<AVG_SIZE; i++)
  {
    pTModule->magic_Encoder[i] = pTModule->magic_Encoder[i+1];
    pTModule->mRecUseMsg.mEncoderTurns += pTModule->magic_Encoder[i];
  }
  pTModule->magic_Encoder[AVG_SIZE] = tEncoderTurns;
  pTModule->mRecUseMsg.mEncoderTurns /= (AVG_SIZE+1.0);


  /**********************************水平仪器转换************************************/
  pTModule->mRecUseMsg.mInclinometer1_x = pTModule->mRecMsg.mInclinometer1_y*0.001;     //单位为角度
  pTModule->mRecUseMsg.mInclinometer1_y = pTModule->mRecMsg.mInclinometer1_x*0.001;     //单位为角度
  //水平仪器转换
  pTModule->mRecUseMsg.mInclinometer1_dertX = (pTModule->mRecUseMsg.mInclinometer1_x-pTModule->mConfParam->mInclinometerX)*0.0175/11.3;     //单位为rad
  pTModule->mRecUseMsg.mInclinometer1_dertY = (pTModule->mRecUseMsg.mInclinometer1_y-pTModule->mConfParam->mInclinometerY)*0.0175/11.3;     //单位为rad
  //dert Inclinometer
  for (int i=0; i<LEVEL_AVG_SIZE; i++)
  {
    pTModule->mRecUseMsg.mInclinometer1_dertX += pTModule->mDertInclim[i].x;
    pTModule->mRecUseMsg.mInclinometer1_dertY += pTModule->mDertInclim[i].y;
    pTModule->mDertInclim[i] = pTModule->mDertInclim[i+1];
  }
  pTModule->mDertInclim[LEVEL_AVG_SIZE].x = (pTModule->mRecUseMsg.mInclinometer1_x-pTModule->mConfParam->mInclinometerX)*0.0175/11.3;
  pTModule->mDertInclim[LEVEL_AVG_SIZE].y = (pTModule->mRecUseMsg.mInclinometer1_y-pTModule->mConfParam->mInclinometerY)*0.0175/11.3;
  pTModule->mRecUseMsg.mInclinometer1_dertX /= (LEVEL_AVG_SIZE+1.0);
  pTModule->mRecUseMsg.mInclinometer1_dertY /= (LEVEL_AVG_SIZE+1.0);
  pTModule->mRecUseMsg.mLevelSiko1 = (pTModule->levelSikoXY_L*tan(pTModule->mRecUseMsg.mInclinometer1_dertX));
  pTModule->mRecUseMsg.mLevelSiko2 = (pTModule->levelSikoXY_L*tan(pTModule->mRecUseMsg.mInclinometer1_dertY));

/*
  if(strcmp(pTModule->mThreadName,"MN_SERVER3") == 0)
        printf("recMsg:levelXY(%f %f) dertXY(%f %f) conf(%f %f)  mLevelSiko(%f %f)\n",
               pTModule->mRecUseMsg.mInclinometer1_x,
               pTModule->mRecUseMsg.mInclinometer1_y,
              pTModule->mDertInclim[LEVEL_AVG_SIZE].x,
              pTModule->mDertInclim[LEVEL_AVG_SIZE].y,
               pTModule->mConfParam->mInclinometerX,
               pTModule->mConfParam->mInclinometerY,
              pTModule->mRecUseMsg.mLevelSiko1,
              pTModule->mRecUseMsg.mLevelSiko2);
*/
 /**********************************水平仪器转换 end************************************/

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
  //pTModule->mMagicControl.alfa_reco[0] = deadZone(pTModule->mRecUseMsg.mEncoderTurns, 0.05);
  pTModule->mMagicControl.alfa_reco[0] = pTModule->mRecUseMsg.mEncoderTurns;
  //pTModule->mMagicControl.alfa_reco[0] = 0;

  pTModule->mMagicControl.F_reco[0] = readTensionValue(pTModule);


  //ci shan chi zhuanhuan juedui
  pTModule->mRecMsg.mSiko1 *= pTModule->motorDirection[0];
  pTModule->mRecMsg.mSiko2 *= pTModule->motorDirection[1];

  //m单位
  pTModule->mRecUseMsg.mSiko1 = ((float)pTModule->mRecMsg.mSiko1)*0.01*0.001 - pTModule->mConfParam->mConfSaveSikoX*0.001;
  pTModule->mRecUseMsg.mSiko2 = ((float)pTModule->mRecMsg.mSiko2)*0.01*0.001 - pTModule->mConfParam->mConfSaveSikoY*0.001;

  //慈善尺滤波
  for (int i=0; i<AVG_SIZE; i++)
  {
    pTModule->mSikoAvg[i] = pTModule->mSikoAvg[i+1];
    pTModule->mRecUseMsg.mSiko1 += pTModule->mSikoAvg[i].x;
    pTModule->mRecUseMsg.mSiko2 += pTModule->mSikoAvg[i].y;
  }
  pTModule->mSikoAvg[AVG_SIZE].x = ((float)pTModule->mRecMsg.mSiko1)*0.01*0.001 - pTModule->mConfParam->mConfSaveSikoX*0.001;
  pTModule->mSikoAvg[AVG_SIZE].y = ((float)pTModule->mRecMsg.mSiko2)*0.01*0.001 - pTModule->mConfParam->mConfSaveSikoY*0.001;
  pTModule->mRecUseMsg.mSiko1 /= (AVG_SIZE+1.0);
  pTModule->mRecUseMsg.mSiko2 /= (AVG_SIZE+1.0);

  //**********Inclim compensate Y
  //pTModule->mRecUseMsg.mSiko1   += (pTModule->mRecUseMsg.mLevelSiko1*pTModule->levelChangeSikoXYDirection[0]);
  pTModule->mRecUseMsg.mSiko2   += pTModule->mRecUseMsg.mLevelSiko2*pTModule->levelChangeSikoXYDirection[1];
  //if siko > 9 t
  if(fabs(pTModule->mRecUseMsg.mSiko2) < 9)
    pTModule->mRecUseMsg.mSiko2   += 0.2*(1.0-cos(pTModule->mMagicControl.alfa_reco[0]));

 // printf("%f %d, %f %d\n", pTModule->mRecUseMsg.mSiko1,pTModule->mRecMsg.mSiko1, pTModule->mRecUseMsg.mSiko2,pTModule->mRecMsg.mSiko2);



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

  //TUDO

  pMsg->mCrcCode = 0;

  return iRet;
}

#if 0
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

    printf("VXYZ:%f %f %f %f\n",mVel.v_p[0],mVel.v_p[1],
                                mVel.v_p[2],mVel.v_p[3]);

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
#endif
static int motorMoveVZCmd(BASE::ARMS_THREAD_INFO *pTModule, float mVel)
{
    return  motorMoveICmd(pTModule, mVel*pTModule->motorDirection[2], BASE::CT_MOTOR_MOVE_POSITIVE, MOTOR_Z);
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
      mMotors.mMotorsCmd[findMotorNum(i)].mSpeed = ((int32_t)(mVel.v_p[i]*DEF_SYS_RADIAN_TO_PULSE))*(pTModule->motorDirection[i]);
    }

    //printf("V:%f %f %f %f v:%d %d %d %d\n",mVel.v_p[0], mVel.v_p[1],mVel.v_p[2],mVel.v_p[3],
    //                                      mMotors.mMotorsCmd[0].mSpeed,mMotors.mMotorsCmd[1].mSpeed,mMotors.mMotorsCmd[2].mSpeed,mMotors.mMotorsCmd[3].mSpeed);
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
* 功能：读取拉力计值，N 单位
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
  if(pTModule->mCond.mMotorCmd != BASE::ST_ALL_RUN_RUNNING)
    pTModule->mMagicRCnt = 0;

  //接收命令后执行命令中
  switch (pTModule->mCond.mMotorCmd)
  {
    case BASE::ST_ALL_MOVE_RUNNING:
    {
      //位置控制,暂时没有，机械臂板子只有速度控制
      //速度控制
      if((pTModule->mRCnt%10) == 0)
      {
          printf("%s %f %f %f %f %f %f %f %f\n", pTModule->mThreadName,
                                              pTModule->mIntCmd.mCmdV.v_p[0], pTModule->mIntCmd.mCmdV.v_p[1], pTModule->mIntCmd.mCmdV.v_p[2], pTModule->mIntCmd.mCmdV.v_p[3],
                                              pTModule->mRecUseMsg.mMotors[0].mSpeed, pTModule->mRecUseMsg.mMotors[1].mSpeed, pTModule->mRecUseMsg.mMotors[2].mSpeed, pTModule->mRecUseMsg.mMotors[3].mSpeed);
      }

      motorMoveVXYZWCmd(pTModule, pTModule->mIntCmd.mCmdV);
      break;
    }
    case BASE::ST_ALL_PULL_RUNNING:
    {
      //pull 命令，根据拉立计信息慢慢的增加电机转速。
      float  tensiosKg = readTensionValue(pTModule)/9.8;
      float  cmdKg  =  pTModule->mIntCmd.mCmdTension; //*pTModule->mMagicControl.mM;
      if(fabs(tensiosKg-cmdKg) > 0.05)
      {
        //TUDO 正转  倒转
        if(tensiosKg > cmdKg)
            motorMoveVZCmd(pTModule, -2);
        else
            motorMoveVZCmd(pTModule, 2);
         //printf("conf motor all pull move:set tension %f, now tension:%f, state:%d\n",  pTModule->mIntCmd.mCmdTension, tensiosKg, pTModule->mIsNowMotorCmd);
      }
      else
      {
          motorAllStopCmd(pTModule);
          setStateCond(pTModule->mCond, BASE::M_STATE_CONF, BASE::ST_LIFT_STOPED, 0, 0);
      }
      break;
    }
    case BASE::ST_ALL_CALBRATE:
    {
      calibPhase(pTModule);
      break;
    }
    case BASE::ST_LIFT_STOPED:
    {
      motorAllStopCmd(pTModule);
      //printf("lift stop cmd....\n");
      break;
    }
    //run mode
    case BASE::ST_ALL_RUN_RUNNING:
    {
      runningPhase(pTModule);
      break;
    }
    case BASE::ST_RUN_STOPED:
    {
      motorAllStopCmd(pTModule);
      if((pTModule->mRCnt%100) == 0)
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
* 功能：拉力弹簧、自标定算法phase
* @param pTModule : pTModule是线程信息结构体，存储有拉力计结构体指针
* @return Descriptions
******************************************************************************/
static int32_t calibPhase(BASE::ARMS_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;
  //pull 命令，根据拉立计信息慢慢的增加电机转速。
  float  tensiosKg = readTensionValue(pTModule)/9.8;
  float  cmdKg  =  pTModule->mIntCmd.mCalibKg;
  float  encoderD = pTModule->mRecUseMsg.mEncoderTurns*57.29578;
  printf("%s calibrate %f %f %f\n",pTModule->mThreadName,tensiosKg,cmdKg,encoderD);
  //p d自标定算法
  float kp = 5.0;
  float kD = 10.0;
  BASE::VEL_4 mCmdV = {0};

  //编码器和卷扬机不能同时标定
  if(fabs((tensiosKg-cmdKg))>0.06)
  {
    mCmdV.v_p[2] = kp*(cmdKg - tensiosKg);
  }
  else {
    if(fabs(encoderD)>0.5)
      mCmdV.v_p[3] = -kp*encoderD;
  }

  if(mCmdV.v_p[2]>50)
      mCmdV.v_p[2] = 50;
  if(mCmdV.v_p[2]<-50)
      mCmdV.v_p[2] = -50;

  if(mCmdV.v_p[3]>50)
      mCmdV.v_p[3] = 50;
  if(mCmdV.v_p[3]<-50)
      mCmdV.v_p[3] = -50;

  motorMoveVXYZWCmd(pTModule, mCmdV);
  return iRet;
}

/******************************************************************************
* 功能：算法运行phase
* @param pTModule : pTModule是线程信息结构体，存储有拉力计结构体指针
* @return Descriptions
******************************************************************************/
static int32_t runningPhase(BASE::ARMS_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;
  if(*(pTModule->mIsRun) != 0)
  {
    pTModule->mMagicRCnt++;
    memset((char*)&pTModule->mMagicControl.mCmdV, 0, sizeof(BASE::VEL_4));
    if(pTModule->mMagicRCnt<10)
    {
      //参数初始化
      calibrationSensors(pTModule);
    }
    //缓冲200个周期在启动控制算法
    if(pTModule->mMagicRCnt > 200)
    {
      //根据上位机选择，运行那个算法
      if(*(pTModule->mIsRun) == 1)
      {
          followagic(pTModule);
          pullMagic(pTModule);//函数计算出收放收缩加速度。
      }else if (*(pTModule->mIsRun) == 2) {
          followagic(pTModule);
      }else if (*(pTModule->mIsRun) == 3) {
          pullMagic(pTModule);//函数计算出收放收缩加速度。
      }

      pTModule->mMagicRCnt = 600;
    }else
    {
      memset((char*)&pTModule->mMagicControl.mCmdV, 0, sizeof(BASE::VEL_4));

      memset((char*)&pTModule->mMagicControl.mRopeEndLastL, 0, sizeof(BASE::POS_2));
      memset((char*)pTModule->magic_v, 0, sizeof(float)*(AVG_SIZE+1));
      memset((char*)pTModule->magic_Encoder, 0, sizeof(float)*(AVG_SIZE+1));
      memset((char*)pTModule->magic_d_alfi, 0, sizeof(float)*(AVG_SIZE+1));
      memset((char*)pTModule->magic_f_estime, 0, sizeof(float)*(AVG_SIZE+1));

      memset((char*)&pTModule->magic_XYv, 0, sizeof(BASE::VEL_2)*(XYV_AVG_SIZE+1));
      pTModule->mMagicControl.last_dd_Lz = 0;
    }

    if((pTModule->mRCnt%8) == 0)
      LOGER::PrintfLog(BASE::S_APP_LOGER,"name:%s,ixv:%f iyv:%f izV:%f oxv:%f oyv:%f ozV:%f 兹山尺x:%f 兹山尺y:%f",
                       pTModule->mThreadName, pTModule->mMagicControl.mCmdV.v_p[0]*pTModule->motorDirection[0],
                       pTModule->mMagicControl.mCmdV.v_p[1]*pTModule->motorDirection[1],pTModule->mMagicControl.mCmdV.v_p[2]*pTModule->motorDirection[2],
                       pTModule->mRecUseMsg.mMotors[0].mSpeed,pTModule->mRecUseMsg.mMotors[1].mSpeed,pTModule->mRecUseMsg.mMotors[2].mSpeed,
                       pTModule->mRecUseMsg.mSiko1,pTModule->mRecUseMsg.mSiko2);
    motorMoveVXYZWCmd(pTModule, pTModule->mMagicControl.mCmdV);
  }else
  {
    motorAllStopCmd(pTModule);
    pTModule->mMagicRCnt = 0;
  }

  return  iRet;
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
    {
      setStateCond(pTModule->mCond, BASE::M_STATE_CONF, BASE::ST_LIFT_STOPED, 0, 0);
      printf("conf motor cmd stop\n");
      LOGER::PrintfLog(BASE::S_APP_LOGER, "***conf state: have motor stop. motor state:%d", pTModule->mIsNowMotorCmd);
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
      memset((char*)&pTModule->mIntCmd.mCmdV, 0, sizeof(pTModule->mIntCmd.mCmdV));
      if(lMovedata.isVevOrPos == 1)
      {
        memcpy((char*)pTModule->mIntCmd.mCmdV.v_p, (char*)lMovedata.v_p, sizeof(float)*4);
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

      pTModule->mIntCmd.mCmdTension = lPulldata.v_p[2];

      //if Manual ctrl move mode than move (xyz), else move (0,0,0)(relative position)
      setStateCond(pTModule->mCond, BASE::M_STATE_CONF, BASE::ST_ALL_PULL_RUNNING, 0, 0);
      pTModule->mIsNowMotorCmd = BASE::CMD_INTO_COND_RUN;
      break;
    }
    case BASE::CMD_CALIBRATE_ARM:
    {
      //TUDO
      pthread_mutex_lock(&pTModule->mMotorMutex);
      //memcpy((char*)&Calidata, (char*)&pTModule->mCaliData, sizeof(BASE::CalibrateData));
      pTModule->mIntCmd.mCalibKg = pTModule->mCaliData.mCalKg;
      pthread_mutex_unlock(&pTModule->mMotorMutex);

      //if Manual ctrl move mode than move (xyz), else move (0,0,0)(relative position)
      setStateCond(pTModule->mCond, BASE::M_STATE_CONF, BASE::ST_ALL_CALBRATE, 0, 0);
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
      //LOGER::PrintfLog(BASE::S_APP_LOGER,"wait cmd....  send:%d , rec:%d", pTModule->mRandomCode, pTModule->mRecMsg.mRandomCode);
      motorAllStopCmd(pTModule);
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
  int32_t iRet = 0;
  //检查拉力是否出错
  float tensiosV = readTensionValue(pTModule);
  //float mTempL =  (float)pTModule->mRecMsg.mTension;
  if(tensiosV < 0 || (tensiosV/(pTModule->mConfParam->mConfSaveWeight*10)) < 0.5)
  {
    iRet = -1;
    LOGER::PrintfLog(BASE::S_APP_LOGER,"chaochu:%f  %f\n", tensiosV, (tensiosV/(pTModule->mConfParam->mConfSaveWeight*10)));
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
  float mM = pTModule->mConfParam->mConfSaveWeight;
  float mWn = pTModule->mConfParam->mWn;
  float mCo = pTModule->mConfParam->mCo;
  //计算拉力 摆动角度导数
  float d_f_measure = (pControl->F_reco[0] - pControl->F_reco[1])/pTModule->sysDt;
  //printf("f_0:%f, f_1:%f\n",pControl->F_reco[0],pControl->F_reco[1]);

  //printf("***********alfa value:%f\n",);

  float d_alfi_measure_t = (pControl->alfa_reco[0] - pControl->alfa_reco[1])/pTModule->sysDt;
  float d_alfi_measure = d_alfi_measure_t;
  //d alfi_measure  avg
  //拉力计滤波
  for (int i=0; i<AVG_SIZE; i++)
  {
    pTModule->magic_d_alfi[i] = pTModule->magic_d_alfi[i+1];
    d_alfi_measure += pTModule->magic_d_alfi[i];
  }
  pTModule->magic_d_alfi[AVG_SIZE] = d_alfi_measure_t;
  d_alfi_measure /= (AVG_SIZE+1.0);


  float ddalfi_measure = (pControl->alfa_reco[0] - 2*pControl->alfa_reco[1] + pControl->alfa_reco[2])/(pTModule->sysDt*pTModule->sysDt);
  //float ddalfi_measure = (d_alfi_measure - pTModule->magic_d_alfi[AVG_SIZE-1])/(pTModule->sysDt);

  //printf("************* ddalfi_measure:%f alfa_reco:%f %f %f\n",
  //                       pControl->ddalfi_measure, pControl->alfa_reco[0], pControl->alfa_reco[1], pControl->alfa_reco[2]);

  float alfa_m = deadZone(pControl->alfa_reco[0], 0.0005);
  //float alfa_m = pControl->alfa_reco[0];
  //if(fabs(alfa_m) < 1.5/60)
  //{
  //  alfa_m = 0;
  //}
  //对外力进行估算
  float f_estimate_t = -pControl->mK1 * ddalfi_measure - mM*pControl->last_dd_Lz - pControl->mK*pControl->mL*alfa_m;
  float f_estimate = f_estimate_t;
  //d alfi_measure  avg
  //拉力计滤波
  for (int i=0; i<AVG_SIZE; i++)
  {
    pTModule->magic_f_estime[i] = pTModule->magic_f_estime[i+1];
    f_estimate += pTModule->magic_f_estime[i];
  }
  pTModule->magic_f_estime[AVG_SIZE] = f_estimate_t;
  f_estimate /= (AVG_SIZE+1.0);


  float dd_Lz = 0;
  /**********************************/
  // server1 not use d_f_measure
  if(strcmp(pTModule->mThreadName, "MN_SERVER1") == 0)
  {
    dd_Lz = (-f_estimate + 0.2*d_f_measure)/mM  +
             0.2*((2*mCo*mWn*d_alfi_measure+pow(mWn,2)*alfa_m)*pControl->mK1 - pControl->mK*pControl->mL*alfa_m)/mM;
  }else
  {
    dd_Lz = (-f_estimate + 0.0*d_f_measure)/mM  +
               0.2*((2*mCo*mWn*d_alfi_measure+pow(mWn,2)*alfa_m)*pControl->mK1 - pControl->mK*pControl->mL*alfa_m)/mM;
  }



  //float dd_Lz = (-f_estimate + 0.0*d_f_measure)/mM  +
  //              0.2*((2*mCo*mWn*d_alfi_measure+pow(mWn,2)*alfa_m)*pControl->mK1 - pControl->mK*pControl->mL*alfa_m)/mM;

  //printf("mK1:%f mM:%f last_dd_Lz:%f mK:%f  mL:%f\n",
  //        pControl->mK1, pControl->mM, pControl->last_dd_Lz, pControl->mK, pControl->mL);

  //pControl->dd_Lz = (-pControl->f_estimate + (pControl->F_reco[0]-pControl->mM*pControl->mG)*2 + 0.4*pControl->d_f_measure)/pControl->mM;
  dd_Lz = -dd_Lz;

  //加速度限制超过1
  if(dd_Lz > 3)
    dd_Lz = 3;
  if(dd_Lz < -3)
    dd_Lz = -3;
  //pControl->dd_Lz = (pControl->dd_Lz > 2 ? 2 : (pControl->dd_Lz < -2 ? -2 : pControl->dd_Lz));

  //得到电机的加速度
  pControl->last_dd_Lz = dd_Lz;
  //电机的加速度
  float d_w = dd_Lz/pControl->mR * pControl->mNdecrease;


  //printf("**********encoder:%f,%d\n",pTModule->mMagicControl.alfa_reco[0],pTModule->mRecMsg.mEncoderTurns);

  if((pTModule->mRCnt%10) == 0)
  printf("*****%s,dd_Lz:%f f_estimate:%f d_f_measure:%f d_alfi_measure:%f  alfa_m:%f alfa_now:%f  d_w:%f  f_re:%f %f %f\n",pTModule->mThreadName,
          dd_Lz, f_estimate, d_f_measure, d_alfi_measure, alfa_m, pControl->alfa_reco[0],d_w, pControl->F_reco[0], pControl->F_reco[1],pControl->F_reco[2]);

  //根据加速度，计算要控制电机的速度
  //计算收放绳子电机要执行的速度，rad/s
  //单位为 rad/s
  //float mjubuZSpeed = pTModule->mRecUseMsg.mMotors[2].mSpeed;
  float mVel = d_w*pTModule->sysDt + pTModule->mRecUseMsg.mMotors[2].mSpeed;
  float mTempVel = mVel;

  //printf("**********d_w:%f, sysDt:%f, cmd:%f, mMotorsSpeed:%f\n",d_w, pTModule->sysDt, mVel,pTModule->mRecUseMsg.mMotors[2].mSpeed);

  /*
  //存储之前速度
  for (int i=0; i<AVG_SIZE; ++i)
  {
    pTModule->magic_v[i] = pTModule->magic_v[i+1];
    mTempVel += pTModule->magic_v[i];
  }
  pTModule->magic_v[AVG_SIZE] = mVel;
  mTempVel /= (AVG_SIZE+1.0);
  */
  //限制速度
  if(mTempVel > 80)
      mTempVel = 80;
  if(mTempVel < -80)
      mTempVel = -80;

  //检查机械臂是否有错
  if(checkRecMsgError(pTModule) < 0)
  {
    //printf("************* mVel:%f , error laliji\n", mTempVel);
    //mVel.v[0] = 0;
  }

  pTModule->mMagicControl.mCmdV.v_p[2] = mTempVel;
}

static float deadZone(float mIndead, float mthr)
{
  float iRet = mIndead;
  if(fabs(iRet) <= fabs(mthr))
  {
    iRet = 0;
  }
  else if(iRet > fabs(mthr))
  {
    iRet -= fabs(mthr);
  }
  else if(iRet < -fabs(mthr))
  {
    iRet += fabs(mthr);
  }
  return iRet;
}

static int32_t followagic(BASE::ARMS_THREAD_INFO *pTModule)
{
  //需要根据磁栅尺计算当前XY偏角
  BASE::POS_2  mRopeEndL;
  float kp = pTModule->mConfParam->mFollowKp;
  float kd = pTModule->mConfParam->mFollowKd;
  float sikoK = pTModule->mMagicControl.mSikoK;

  //mRopeEndL.x = pTModule->mRecUseMsg.mSiko1 * (3.0/0.5);
  //mRopeEndL.y = pTModule->mRecUseMsg.mSiko2 * (3.0/0.5);
  //x y轴设置死区间
  //mRopeEndL.x =  deadZone(pTModule->mRecUseMsg.mSiko1*(sikoK), 0.0025);
  //mRopeEndL.y =  deadZone(pTModule->mRecUseMsg.mSiko2*(sikoK), 0.0025);

  //x y轴设置死区间.
  //502修改算法死区
  mRopeEndL.x =  deadZone(pTModule->mRecUseMsg.mSiko1*(sikoK), 0.0015);
  mRopeEndL.y =  deadZone(pTModule->mRecUseMsg.mSiko2*(sikoK), 0.001);
  //mRopeEndL.y =  deadZone((pTModule->mRecUseMsg.mSiko2+0.2*(1.0-cos(pTModule->mMagicControl.alfa_reco[0])))*(sikoK), 0.0015);
  //printf("++++++siko_y:%f, alf:%f %f",pTModule->mRecUseMsg.mSiko2,pTModule->mMagicControl.alfa_reco[0],  10.0*(1.0-cos(pTModule->mMagicControl.alfa_reco[0])) );

  BASE::ACC_2 mAcc;
  //mAcc = pidGetDa(pTModule->mMagicControl.mRopeEndL, pTModule->mMagicControl.mRopeEndLastL, 0.01);
  mAcc.x = kp * mRopeEndL.x + kd*(mRopeEndL.x-pTModule->mMagicControl.mRopeEndLastL.x)/pTModule->sysDt;
  mAcc.y = kp * mRopeEndL.y + kd*(mRopeEndL.y-pTModule->mMagicControl.mRopeEndLastL.y)/pTModule->sysDt;

  //ceshi
  /*
  if(fabs((pTModule->mRecUseMsg.mSiko2+0.1*(1.0-cos(pTModule->mMagicControl.alfa_reco[0])))*(sikoK)) < 0.002)
     mAcc.y = 0;
  if(fabs(pTModule->mRecUseMsg.mSiko1*(sikoK)) < 0.002)
     mAcc.x = 0;
  */

  pTModule->mMagicControl.mRopeEndLastL = mRopeEndL;

  BASE::ACC_2 mAngelAcc;
  mAngelAcc.x = mAcc.x/0.025 * 30;
  mAngelAcc.y = mAcc.y/0.025 * 30;

  float Xv = pTModule->mRecUseMsg.mMotors[0].mSpeed + mAngelAcc.x*0.01;
  float Yv = pTModule->mRecUseMsg.mMotors[1].mSpeed + mAngelAcc.y*0.01;

  pTModule->mMagicControl.mCmdV.v_p[0] = Xv;
  pTModule->mMagicControl.mCmdV.v_p[1] = Yv;


  //存储之前速度
  for (int i=0; i<XYV_AVG_SIZE; ++i)
  {
    pTModule->magic_XYv[i] = pTModule->magic_XYv[i+1];
    pTModule->mMagicControl.mCmdV.v_p[0] += pTModule->magic_XYv[i].x;
    pTModule->mMagicControl.mCmdV.v_p[1] += pTModule->magic_XYv[i].y;
  }

  pTModule->magic_XYv[XYV_AVG_SIZE].x = Xv;
  pTModule->magic_XYv[XYV_AVG_SIZE].y = Yv;
  pTModule->mMagicControl.mCmdV.v_p[0] /= (XYV_AVG_SIZE+1.0);
  pTModule->mMagicControl.mCmdV.v_p[1] /= (XYV_AVG_SIZE+1.0);


  //限速
  if(pTModule->mMagicControl.mCmdV.v_p[0] > 50)
      pTModule->mMagicControl.mCmdV.v_p[0] = 50;
  if(pTModule->mMagicControl.mCmdV.v_p[0] < -50)
      pTModule->mMagicControl.mCmdV.v_p[0] = -50;

  if(pTModule->mMagicControl.mCmdV.v_p[1] > 50)
      pTModule->mMagicControl.mCmdV.v_p[1] = 50;
  if(pTModule->mMagicControl.mCmdV.v_p[1] < -50)
      pTModule->mMagicControl.mCmdV.v_p[1] = -50;

  //pTModule->mMagicControl.mCmdV.v_p[0] = 0;
  return 0;
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
  socklen_t mun = sizeof(pTModule->mPeerAddr);

  if(initServer(pTModule) != 0)
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "leader bind server ip failed, check network again !");
    moduleEndUp(pTModule);
    pTModule->mWorking = false;
    return 0;
  }

  int size = 0;
  char printfError[100];

  LOGER::PrintfLog(BASE::S_APP_LOGER, "%s leader running!", pTModule->mThreadName);
  //running state
  while(pTModule->mWorking)
  {
    //lock,wait signal
    pthread_mutex_lock(&pTModule->mArmsMsgMutex);
    pthread_cond_wait(&pTModule->mArmsMsgReady, &pTModule->mArmsMsgMutex);
    pthread_mutex_unlock(&pTModule->mArmsMsgMutex);

    pTModule->mRCnt++;
    //struct timespec now;
    //clock_gettime(CLOCK_MONOTONIC, &now);
    //LOGER::PrintfLog(BASE::S_APP_LOGER, "lock time(s) :%d,lock time(us):%d", now.tv_sec, now.tv_nsec/1000);

    //rec msg
    memset((char*)&pTModule->mRecMsg, 0 ,sizeof(pTModule->mRecMsg));
    size = recvfrom(pTModule->mSocket , (char*)&(pTModule->mRecMsg), sizeof(BASE::ARMS_R_MSG), 0, (sockaddr*)&(pTModule->mPeerAddr), &mun);

    if(size != sizeof(BASE::ARMS_R_MSG))
    {
      sprintf(printfError,"%s no client link, or size < rec msg, size:%d\n",pTModule->mThreadName,size);
      //LOGER::PrintfLog(BASE::S_APP_LOGER, "state: %s, no client link, or size < rec msg", pTModule->mThreadName);
      //只有在conf 或者 run 模式下超时才转换到stop模式
      if((pTModule->mState == BASE::M_STATE_CONF) || (pTModule->mState == BASE::M_STATE_RUN))
        pTModule->mState = BASE::M_STATE_STOP;
    }else
    {
        reformRecMsg(pTModule);
        //printf("**********%f  %f\n",pTModule->mConfParam->mFollowKp,pTModule->mConfParam->mFollowKd);
    }

    //计算延迟

    //struct timespec now;
    //clock_gettime(CLOCK_MONOTONIC, &now);
    //LOGER::PrintfLog(BASE::S_APP_LOGER, "send time(us) :%d,rec time(us):%d", pTModule->mSendMsg.mSysTime.mSysTimeUs,now.tv_nsec/1000);




    //calSysDelayed(pTModule->mReadLiftHzData, mSysSendTime, pTModule->mRecMsg.mSysTime);
    //static float last1 = 0,last2 = 0;
    /*
    printf("%.3f %.3f %d %d V: (%f %f)\n", pTModule->mRecUseMsg.mSiko1, pTModule->mRecUseMsg.mSiko2, pTModule->mRecMsg.mSiko1, pTModule->mRecMsg.mSiko2,
                                           (pTModule->mRecUseMsg.mSiko1-last1)*100, (pTModule->mRecUseMsg.mSiko2-last2)*100);
    last1 = pTModule->mRecUseMsg.mSiko1;      last2 = pTModule->mRecUseMsg.mSiko2;
    */

    switch (pTModule->mState)
    {
      case BASE::M_STATE_INIT:
      {
        initFire(pTModule);
        break;
      }
      case BASE::M_STATE_CONF:
      {
        //接近开关限位
        if((pTModule->mRCnt%8) == 0)
            LOGER::PrintfLog(BASE::S_APP_LOGER, "模块：%s,随即码:%d,接近开关:%d,倾角仪:%d %d,磁删尺:%d %d,编码器:%d,拉计:%d,速度(%.2f,%.2f,%.2f,%.2f),位置(%.1f,%.1f,%.1f,%.1f)",
                                             pTModule->mThreadName,  pTModule->mRecMsg.mRandomCode,
                                             pTModule->mRecMsg.mSwitchStateCode,pTModule->mRecMsg.mInclinometer1_x, pTModule->mRecMsg.mInclinometer1_y,
                                             pTModule->mRecMsg.mSiko1, pTModule->mRecMsg.mSiko2,pTModule->mRecMsg.mEncoderTurns,pTModule->mRecMsg.mTension,
                                             pTModule->mRecUseMsg.mMotors[0].mSpeed, pTModule->mRecUseMsg.mMotors[1].mSpeed,pTModule->mRecUseMsg.mMotors[2].mSpeed,pTModule->mRecUseMsg.mMotors[3].mSpeed,
                                             pTModule->mRecUseMsg.mMotors[0].mPosition, pTModule->mRecUseMsg.mMotors[1].mPosition,pTModule->mRecUseMsg.mMotors[2].mPosition,pTModule->mRecUseMsg.mMotors[3].mPosition);

        confFire(pTModule);
        break;
      }
      case BASE::M_STATE_RUN:
      {
        runFire(pTModule);
        break;
      }
      case BASE::M_STATE_STOP:
      {
        motorAllStopCmd(pTModule);
        if((pTModule->mRCnt%100) == 0)
          printf(printfError);
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
