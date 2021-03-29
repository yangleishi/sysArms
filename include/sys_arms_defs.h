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

#define	PRINT_QUEUE_MAX_ITEMS		5120
#define	PRINT_STRING_MAX_LENGTH		128

namespace BASE {

//////////////////////////////////// System internal structure  /////////////////////////////////////

////////////////////////////////////
//DEFINE THREAD running state
typedef enum
{
  M_STATE_INIT = 0,
  M_STATE_CONF,
  M_STATE_RUN,
  M_STATE_STOP,
} M_STATE;

//DEFINE THREAD ack state
typedef enum
{
  ACK_STATE_NULL = 100,
  ACK_STATE_INIT_OK,
} ACK_STATE;

//DEFINE THREAD recfrom state
typedef enum
{
  F_STATE_UNLINK = 0,
  F_STATE_LINK,
  F_STATE_OVERTIME,
} REC_UDP_STATE;

//for log part/////////////
//Define
typedef enum
{
  S_APP_LOGER = 0,
  S_ARMS_DATA,
} LOG_SAVA_W;

////////////////////////////////////11 arms  UDP communication protocol   /////////////////////////////////////

///for udp msg ctrl part/////////////

//Define the motor ctrl
const uint8_t   CT_MOTOR_POWERDOWN = 0x00;
const uint8_t   CT_MOTOR_POWERON    = 0x01;
const uint8_t   CT_MOTOR_STOP  = 0x02;
const uint8_t   CT_MOTOR_ZERO   = 0x03;
const uint8_t   CT_MOTOR_RUN   = 0x04;

//Define the motor rec state
const uint8_t   ST_MOTOR_START   = 0x00;
const uint8_t   ST_MOTOR_STOP    = 0x01;

///end of udp msg ctrl part


///for udp msg state part/////////////
//Define the rec code state
const uint16_t   ST_SYS_STATE_OK        = 0x0000;

const uint16_t   ST_SYS_REC_ERROR         = 0xFFF0;

///end of udp msg state part

////////////////////////////////////man interaction  UDP communication protocol   /////////////////////////////////////

///for udp msg ctrl part/////////////
//Define the ctrl
const uint8_t   CT_MAN_MACHINE_POWERON = 0x00;
const uint8_t   CT_MAN_MACHINE_FIRE    = 0x01;
const uint8_t   CT_MAN_MACHINE_UNFIRE  = 0x02;
const uint8_t   CT_MAN_MACHINE_RESET   = 0x03;
///end of udp msg ctrl part


///for udp msg state part/////////////
//Define the state
const uint16_t   ST_MAN_MACHINE_POWERON_OK        = 0x0000;
const uint16_t   ST_MAN_MACHINE_POWERON_FAILED    = 0x0001;
const uint16_t   ST_MAN_MACHINE_FIRE_OK           = 0x0002;
const uint16_t   ST_MAN_MACHINE_STOP_OK           = 0x0003;

const uint16_t   ST_MAN_MACHINE_KNOCK             = 0xFFFF;
const uint16_t   ST_MAN_MACHINE_PRKNOCK           = 0xFFFE;

const uint16_t   ST_MAN_MACHINE_REC_ERROR         = 0xFFF0;
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
//Motor control datas
typedef struct
{
  //ctrl motors data
  uint16_t   mCmd;
  float      mPosition;
  float      mSpeed;
} MOTOR_CTRL;


//Motor control datas
typedef struct
{
  //four motors data
  MOTOR_CTRL mMotorsCmd[4];
} MOTORS;


///////////////////////////////////
//Time
typedef struct
{
  uint32_t   mSysTimeS;
  uint32_t   mSysTimeUs;
} SYS_TIME;


///////////////////////////////////

typedef struct
{
  //frame start 0x1ACF
  uint16_t   mFrameStart;

  //frame unique dev 0xFF
  uint8_t   mIdentifier;

  //APID 0x02
  uint8_t    mApid;

  //dev type 0x01
  uint8_t    mType;

  // frame time
  SYS_TIME  mSysTime;

} MSG_S_HEARDER;

// UDP, send control data structure. 11 arms
typedef struct: public MSG_S_HEARDER
{
  //ctrl motors data
  MOTORS    mMotors;

  //mCrcCode++
  uint16_t mCrcCode;
} ARMS_S_MSG;

// UDP, send control data structure tensions
typedef struct: public MSG_S_HEARDER
{
  //ctrl tensions start/stop data. 0 1
  uint8_t    mCmd;

  //mCrcCode++
  uint16_t mCrcCode;
} TENSIONS_S_MSG;

///////////////////////////////////
//Motor rec datas
typedef struct
{
  //rec motors datas
  uint8_t    mMotorStateCode;
  float      mPosition;
  float      mSpeed;
  float      mAcceleration;
} MOTOR_REC_DATAS;

// UDP, rec data structure. 11 arms
typedef struct
{
  //frame start 0x1ACF
  uint16_t   mFrameStart;

  //frame unique dev 0xFF
  uint8_t   mIdentifier;

  //APID 0x01
  uint8_t    mApid;

  //dev type 0x01
  uint8_t    mType;

  //states
  uint16_t   mStateCode;

  // frame time
  SYS_TIME  mSysTime;

//****************************  rec Datas  ***********************************//
  // Switch datas
  uint16_t  mSwitchStateCode;
  uint16_t  mSwitchTiggers;

  //inclinometers  datas 0-7,7-15
  uint16_t  mInclinometersStateCode;
  float     mInclinometer1_x;
  float     mInclinometer1_y;
  float     mInclinometer2_x;
  float     mInclinometer2_y;

  //inclinometers  datas 0-7,7-15
  uint16_t  mSikosStateCode;
  float     mSiko1;
  float     mSiko2;

  //encoder  data
  uint8_t   mEncoderStateCode;
  uint64_t  mEncoders;


  //motors datas
  MOTOR_REC_DATAS mMotors[4];

  //mCrcCode++
  uint16_t mCrcCode;
} ARMS_R_MSG;

//tensions. 6 float data
typedef struct
{
  //frame start 0x1ACF
  uint16_t  mFrameStart;

  //frame unique dev 0xFF
  uint8_t   mIdentifier;

  //APID 0x03
  uint8_t   mApid;

  //dev type 0x01
  uint8_t   mType;

  //states
  uint8_t   mStateCode;

  // frame time
  SYS_TIME  mSysTime;

  //****************************  rec tensions  ***********************************//
  // tensions datas
  uint8_t   mTensionsStateCode;
  float     mTensions;
  //mCrcCode++
  uint16_t  mCrcCode;
} TENSIONS_R_MSG;


//tensions. float data.leader use the value
typedef struct
{
  float   mTensions;
  bool    iNewTensions;
} TENSIONS_NEW_MSG;

///////////////////////////////////
// interaction data structure
#define INTERACTION_DATA_LENGTH 1024
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

  //fixed length data
  char mDatas[INTERACTION_DATA_LENGTH];

  //mCrcCode++
  uint16_t mCrcCode;
} ARMS_INTERACTION_MSG;

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
  bool         mWorking;
  char         mThreadName[15];

  uint32_t     mSocket;
  sockaddr_in  mSerAddr, mPeerAddr;
  uint32_t     mSerPort;
  char         mIpV4Str[STR_IPV4_LENGTH];
  M_STATE      mState;

  //log queue pri
  STR_QUEUE* mLogQueue;
  pthread_mutex_t *mPrintQueueMutex;

  //cpu mask
  int         mCpuAffinity;
} THREAD_INFO_HEADER;

//loger thread info
typedef struct
{
  bool         mWorking;
  char         mThreadName[15];

  M_STATE      mState;

  pthread_mutex_t *mPrintQueueMutex;
  pthread_cond_t   mPrintQueueReady;
  STR_QUEUE* mLogQueue;

  STR_QUEUE* mArmsDataQueue;
  //cpu mask
  uint8_t         mCpuAffinity;
} LOG_THREAD_INFO;


//11 arms thread info
typedef struct: public THREAD_INFO_HEADER
{
  ACK_STATE            mAckState;

  bool                 mNewRecMsg;
  ARMS_R_MSG           mRecMsg;
  ARMS_S_MSG           mSendMsg;
  pthread_mutex_t mArmsMsgMutex;
  pthread_cond_t  mArmsMsgReady;
  int             mSerialNumber;  //0 1 2.... not dev int

  TENSIONS_NEW_MSG   *mNowTensionMsg;
} ARMS_THREAD_INFO;

//tensions thread info
typedef struct: public THREAD_INFO_HEADER
{
  TENSIONS_NEW_MSG   *mNowTensionMsg;
} TENSIONS_THREAD_INFO;


//interaction thread info
typedef struct: public THREAD_INFO_HEADER
{
  bool      mIsStataChange;
  M_STATE   mNewState;
}INTERACTION_THREAD_INFO;

}  //namespace

#endif // SYS_ARMS_DEFS_H
