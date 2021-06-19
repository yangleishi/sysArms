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

////////////////////////////////////man  UDP communication protocol   /////////////////////////////////////

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


///for interaction msg state part/////////////
const uint16_t   IN_BASE          = 0x0000;  //first msg upper to interactioner
//Define the ctrl upper =>> interactioner
const uint16_t   IN_KNOCK_DOOR          = IN_BASE + 0;  //first msg upper to interactioner
const uint16_t   IN_CONF_CONFIGURE      = IN_BASE + 1;
const uint16_t   IN_CONF_READ_PARAMS    = IN_BASE + 2;

const uint16_t   IN_CONF_MOVE           = IN_BASE + 3;  // 0-15 modules ,16-31 X Y Z W motors move



//Define the states upper <<= interactioner
const uint16_t   IN_KNOCK_DOOR_OK          = IN_BASE + 100;  //first msg interactioner to upper
const uint16_t   IN_KNOCK_DOOR_FILED       = IN_BASE + 101;  //first msg interactioner to upper

const uint16_t   IN_CONF_CONFIGURE_OK      = IN_BASE + 102;


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
  uint64_t   mEncoder;
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
} TENSIONS_NOW_DATA;

///////////////////////////////////
/******************************************interaction data structure***************************************/
//////////////////////interaction控制指令//////////////
const uint16_t  CMD_BASE = 0;
const uint16_t  CMD_LINK = CMD_BASE + 1;
const uint16_t  CMD_UNLINK = CMD_BASE + 2;
const uint16_t  CMD_SAVE_CONF = CMD_BASE + 3;
const uint16_t  CMD_READ_CONF = CMD_BASE + 4;
const uint16_t  CMD_HAND_MOVE_START = CMD_BASE + 5;
const uint16_t  CMD_HAND_MOVE_STOP = CMD_BASE + 6;
const uint16_t  CMD_ALL_MOVE_START = CMD_BASE + 7;
const uint16_t  CMD_ALL_MOVE_STOP = CMD_BASE + 8;
const uint16_t  CMD_ALL_PULL_START = CMD_BASE + 9;
const uint16_t  CMD_ALL_PULL_STOP = CMD_BASE + 10;

const uint16_t  CMD_RUN_START = CMD_BASE + 11;
const uint16_t  CMD_RUN_STOP = CMD_BASE + 12;
const uint16_t  CMD_RUN_STOP_E = CMD_BASE + 13;

//cycly read datas
const uint16_t  CMD_CYC_READ_LIFT_DATAS = CMD_BASE + 14;
const uint16_t  CMD_CYC_READ_RUNNING_DATAS = CMD_BASE + 15;
const uint16_t  CMD_CYC_READ_SHOWDE_DATAS = CMD_BASE + 16;

//leader run state
const uint16_t  ST_HAND_MOVE_RUNNING = CMD_BASE + 17;
const uint16_t  ST_ALL_MOVE_RUNNING = CMD_BASE + 18;
const uint16_t  ST_ALL_PULL_RUNNING = CMD_BASE + 19;
const uint16_t  ST_LIFT_STOPED = CMD_BASE + 20;

const uint16_t  ST_ALL_RUN_RUNNING = CMD_BASE + 21;
const uint16_t  ST_RUN_STOPED = CMD_BASE + 22;

//////////////////////interaction ACK指令//////////////
const uint16_t  CMD_ACK_BASE = 0;
const uint16_t  CMD_ACK_LINK_OK = CMD_ACK_BASE + 1;
const uint16_t  CMD_ACK_LINK_FAILD = CMD_ACK_BASE + 2;
const uint16_t  CMD_ACK_UNLINK_OK = CMD_ACK_BASE + 3;

const uint16_t  CMD_ACK_SAVECONF_OK = CMD_ACK_BASE + 4;
const uint16_t  CMD_ACK_SAVECONF_FAILD = CMD_ACK_BASE + 5;
const uint16_t  CMD_ACK_READCONF_OK = CMD_ACK_BASE + 6;

const uint16_t  CMD_ACK_READ_LIFT_DATAS = CMD_BASE + 7;
const uint16_t  CMD_ACK_READ_RUNNING_DATAS = CMD_BASE + 8;
const uint16_t  CMD_ACK_READ_SHOWDE_DATAS = CMD_BASE + 9;

//////////////////////CONF 模式下，interaction发送给Supr同步命令类型。便于supr分类上位机发送的按钮CMD//////////////
const uint16_t  CMD_TYPE_BASE = 0;


//下发配置数据发送的最大数据char
#define MSG_DOWN_DATA_MAX  400
//上传数据发送的最大数据char
#define MSG_UP_DATA_MAX  1000
/////////////////////////////////////////////////rec interactions/////////////////////////////////////
typedef struct
{
    //*******************16位CtrlWord******************************************/
    uint16_t   CmdIdentify;
    //*******************数据段**********************/
    //下发的数据
    char Datas[MSG_DOWN_DATA_MAX];
    //*******************序列码、随机码、保留位**********************/
    uint16_t CRC;
}MArmsDownData;

///////////////////////上传数据////////////////////////
typedef struct
{
    //*******************16位stateWord******************************************/
    uint16_t   StatusWord;
    //*******************16位stateCode******************************************/
    uint16_t   StatusCode;
    //*******************数据段**********************/
    //下发的数据
    char Datas[MSG_UP_DATA_MAX];
    //*******************序列码、随机码、保留位**********************/
    uint16_t CRC;
}MArmsUpData;

//配置界面中单元的重量、编码器真值设置
typedef struct{
  float mConfSaveWeight;
  float mConfSaveEncoderX;
  float mConfSaveEncoderY;
  float mConfSaveEncoderZ;
  float mConfSaveEncoderP;
  float mConfSaveEncoderT;
  int   mIsValid;
} SaveConfData;

//配置界面中单元的重量、编码器真值设置
typedef struct{
  float mConfSaveWeight;
  float mConfSaveEncoderX;
  float mConfSaveEncoderY;
  float mConfSaveEncoderZ;
  float mConfSaveEncoderP;
  float mConfSaveEncoderT;

  float mConfReadPull;
  float mConfReadEncoderX;
  float mConfReadEncoderY;
  float mConfReadEncoderZ;
  float mConfReadEncoderP;
  float mConfReadEncoderT;
  float mConfReadSikoX;
  float mConfReadSikoY;
  float mConfReadLevelX;
  float mConfReadLevelY;
  int   mIsValid;
} ReadConfData;

//起吊界面中，上位机发送的手动控制数据命令
typedef struct{
  int   mMudoleNum;
  float mHandXMove;
  float mHandYMove;
  float mHandZMove;
  float mHandWMove;
  float mMoveRelatAbs;
} MoveLiftSigalData;

//起重界面中整体控制移动,可控制多个单元X Y Z方向移动
typedef struct{
  int     mMudoleNum;
  float   mHandXMove;
  float   mHandYMove;
  float   mHandZMove;
  float   mHandMoveSpeed;
  bool    mIsValid;
} MoveLiftAllData;

//起吊界面中，控制器接收arms运行数据
typedef struct{
  int     mMudoleNum;
  float   mHandXMoveNow;
  float   mHandYMoveNow;
  float   mHandZMoveNow;
  float   mHandWMoveNow;
} ReadLiftSigalNowData;

//起重界面中整体pull移动,可控制多个单元上拉
typedef struct{
  int     mMudoleNum;
  float   mHandPull;
  int    mIsValid;
} PullLiftAllData;

//interaction cmd datas to supr
typedef struct{
  int            mIsNewRec;
  int            mIsNewSend;
  MArmsDownData  mRecMsg;
  MArmsUpData    mSendMsg;

  pthread_mutex_t mInteractionRecMutex;
  pthread_mutex_t mInteractionSendMutex;
} InteractionDataToSupr;

//interaction. supr to interaction
typedef struct{
  BASE::ReadLiftSigalNowData* mReadLiftNowDatas;
  pthread_mutex_t*            mReadLiftNowDatasMutex;
} SuprDataToInteraction;

/////////////////////////////////sys //////////////////////////////////////////////
//TODU
typedef struct
{
  ARMS_R_MSG mArmsMsgs[DEF_SYS_USE_ARMS_NUMS];
  M_STATE  mArmsState[DEF_SYS_USE_ARMS_NUMS];
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
  ACK_STATE       mAckState;

  bool            mNewRecMsg;
  ARMS_R_MSG      mRecMsg;
  ARMS_S_MSG      mSendMsg;
  pthread_mutex_t mArmsMsgMutex;
  pthread_cond_t  mArmsMsgReady;
  int             mSerialNumber;  //0 1 2.... not dev int

  //rec motor cmd
  int                mIsNowMotorCmd;
  MoveLiftSigalData  mMoveData;
  MoveLiftAllData    mAllMoveData;
  PullLiftAllData    mAllPullData;

  pthread_mutex_t   mMotorMutex;

  //send motor data
  ReadLiftSigalNowData  mNowData;

  TENSIONS_NOW_DATA   *mNowTension;
} ARMS_THREAD_INFO;

//tensions thread info
typedef struct: public THREAD_INFO_HEADER
{
  TENSIONS_NOW_DATA   *mNowTension;
} TENSIONS_THREAD_INFO;


//interaction thread info
typedef struct: public THREAD_INFO_HEADER
{
  bool      mIsStataChange;
  M_STATE   mNewState;

  MArmsDownData  mRecMsg;
  MArmsUpData    mSendMsg;

  InteractionDataToSupr *mInterToSuprDatas;
  SuprDataToInteraction  mSuprDatasToInterasction;

}INTERACTION_THREAD_INFO;

}  //namespace

#endif // SYS_ARMS_DEFS_H
