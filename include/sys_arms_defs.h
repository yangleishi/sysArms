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

#define STR_IPV4_LENGTH     16

#define	PRINT_QUEUE_MAX_ITEMS		1024
#define	PRINT_STRING_MAX_LENGTH		128

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


////////////////////////////////////  UDP communication protocol   /////////////////////////////////////

///for udp msg ctrl part/////////////
//Define the ctrl
const uint8_t   CT_SYS_POWERON = 0x00;
const uint8_t   CT_SYS_FIRE    = 0x01;
const uint8_t   CT_SYS_UNFIRE  = 0x02;
const uint8_t   CT_SYS_RESET   = 0x03;
///end of udp msg ctrl part


///for udp msg state part/////////////
//Define the state
const uint16_t   ST_SYS_POWERON_OK        = 0x0000;
const uint16_t   ST_SYS_POWERON_FAILED    = 0x0001;
const uint16_t   ST_SYS_FIRE_OK           = 0x0002;
const uint16_t   ST_SYS_STOP_OK           = 0x0003;

const uint16_t   ST_SYS_KNOCK             = 0xFFFF;
const uint16_t   ST_SYS_PRKNOCK           = 0xFFFE;

const uint16_t   ST_SYS_REC_ERROR         = 0xFFF0;

///end of udp msg state part

////////////////////////////////////printf queue /////////////////////////////////////
typedef struct
{
  char  mString[PRINT_STRING_MAX_LENGTH];
} PRINT_STR;

typedef struct
{
  PRINT_STR mPrintPond[PRINT_QUEUE_MAX_ITEMS];	// 循环队列的缓冲区
  uint32_t mFront;		// 循环队列的头指针
  uint32_t mRear;		// 循环队列的尾指针
} STR_QUEUE;

///////////////////////////////////
//Motor control data
typedef struct
{
  float   mPosition;
  float   mSpeed;
  //TODU
} MOTOR_DATA;

//Motor control datas
typedef struct
{
  //four motors data
  MOTOR_DATA mServo1;
  MOTOR_DATA mServo2;
  MOTOR_DATA mStepping1;
  MOTOR_DATA mStepping2;
} MOTORS;

///////////////////////////////////
//Time
typedef struct
{
  uint32_t   mSysTimeS;
  uint32_t   mSysTimeUs;
} SYS_TIME;


///////////////////////////////////
// UDP, send control data structure
typedef struct
{
  //frame start 0x88
  uint8_t   mIdentifier;

  // control cmd
  uint8_t   mCtrl;

  // system state
  uint16_t  mSysState;

  // frame time
  SYS_TIME  mSysTime;

  // data length,now fixed length
  uint16_t  mDataLength;

  //motors data
  MOTORS mMotors;

  //mCrcCode++
  uint16_t mCrcCode;
} ARMS_S_MSG;


///////////////////////////////////
// UDP, rec control data structure
typedef struct
{
  float    mEncoder;
  float    mSiko[2];
  float    mInclinometerXY[2];
  uint16_t mSwitch;
} SENSOR;

typedef struct
{
  //frame start 0x88
  uint8_t   mIdentifier;

  // control cmd
  uint8_t   mCtrl;

  // system state
  uint16_t  mSysState;

  // frame time
  SYS_TIME  mSysTime;

  // data length,now fixed length
  uint16_t  mDataLength;

  //motors data
  MOTORS mMotors;

  SENSOR mSensors;
  //mCrcCode++
  uint16_t mCrcCode;
} ARMS_R_MSG;

//tensions. 6 float data
typedef struct
{
  //frame start 0x88
  uint8_t   mIdentifier;

  // control cmd
  uint8_t   mCtrl;

  // system state
  uint16_t  mSysState;

  // frame time
  SYS_TIME  mSysTime;

  // data length,now fixed length
  uint16_t  mDataLength;

  float    mTensions[6];
  //mCrcCode++
  uint16_t mCrcCode;
} ARMS_TENSIONS_MSG;

/////////////////////////////////TODU//////////////////////////////////////////////
//TODU
typedef struct
{
  ARMS_R_MSG mArmsMsgs[DEF_SYS_ARMS_NUMS];
  M_STATE  mArmsState[DEF_SYS_ARMS_NUMS];
} ARMS_MSGS;

////////////////////////////////////
//  Each thread runs parameters
typedef struct
{
  bool             mWorking;
  char         mThreadName[15];

  M_STATE          mState;
  pthread_cond_t   mPrintQueueReady;

  STR_QUEUE* mLogQueue;

} LOG_THREAD_INFO;

typedef struct
{
  bool         mWorking;
  char         mThreadName[15];

  uint32_t     mSocket;
  sockaddr_in  mSerAddr, mPeerAddr;
  uint32_t     mSerPort;
  char         mIpV4Str[STR_IPV4_LENGTH];
  M_STATE         mState;
  pthread_mutex_t mArmsMsgMutex;
  pthread_cond_t  mArmsMsgReady;

  ARMS_R_MSG      mRecMsg;
  ARMS_S_MSG      mSendMsg;

  //log queue pri
  STR_QUEUE* mLogQueue;
} ARMS_THREAD_INFO, INTERACTION_THREAD_INFO;


}  //namespace

#endif // SYS_ARMS_DEFS_H
