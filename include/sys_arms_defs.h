/******************************************************************************
**
* Copyright (c)2021 SHI YANGLEI
* All Rights Reserved
*
*
******************************************************************************/
#ifndef SYS_ARMS_DEFS_H
#define SYS_ARMS_DEFS_H

#include <arpa/inet.h>
#include <pthread.h>

#include "sys_arms_conf.hpp"

#define USEC_PER_SEC		1000000
#define NSEC_PER_SEC		1000000000

namespace BASE {

//////////////////////////////////// System internal structure  /////////////////////////////////////

////////////////////////////////////
//DEFINE THREAD running state
typedef enum
{
  M_STATE_INIT = 0,
  M_STATE_RUN,
  M_STATE_STOP,
} M_STATE;

////////////////////////////////////
//  Each thread runs parameters
typedef struct
{
  bool         mWorking;
  uint32_t     mSocket;
  sockaddr_in  mSerAddr, mPeerAddr;
  uint32_t     mSerPort;
  char         mIpV4Str[16];
  M_STATE         mState;
  pthread_mutex_t mArmsMsgMutex;
  pthread_cond_t  mArmsMsgReady;
  timespec        startTime;
} ARMS_THREAD_INFO;


////////////////////////////////////  UDP communication protocol   /////////////////////////////////////

///////////////////////////////////
//Motor control data
typedef struct
{
  float   mPosition;
  float   mSpeed;
  float   mAcceleration;
  //TODU
} MOTOR_DATA;


///////////////////////////////////
// UDP, send control data structure
typedef struct
{
  //0x88
  uint8_t   mIdentifier;
  // control
  uint8_t   mCtrl;
  uint16_t  mSysState;
  uint16_t  mDataLength;
  //motor data
  MOTOR_DATA mServo1;
  MOTOR_DATA mServo2;
  MOTOR_DATA mStepping1;
  MOTOR_DATA mStepping2;

  //mCrcCode++
  uint16_t mCrcCode;
} ARMS_MSG;


/////////////////////////////////TODU//////////////////////////////////////////////
//TODU
typedef struct
{
  ARMS_MSG mArmsMsgs[DEF_SYS_ARMS_NUMS];
  M_STATE  mArmsState[DEF_SYS_ARMS_NUMS];
} ARMS_MSGS;



}  //namespace

#endif // SYS_ARMS_DEFS_H
