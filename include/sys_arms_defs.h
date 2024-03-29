﻿/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_daemon.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:创建线程模块（后期修改为多进程），修改线程优先级，线程绑定核模块
*
********************************************************************************/
#ifndef SYS_ARMS_DEFS_H
#define SYS_ARMS_DEFS_H

#include <arpa/inet.h>
#include <pthread.h>

#include "sys_arms_conf.hpp"

#define USEC_PER_SEC		1000000
#define NSEC_PER_SEC		1000000000

#define STR_IPV4_LENGTH     16

#define	PRINT_QUEUE_MAX_ITEMS		5120
#define	PRINT_STRING_MAX_LENGTH		256

//电机速度 ，脉冲/s,1rpm 等于 10000/60
#define MOTOR_V_TO_S            (166)

#define MOTOR_X          (2)
#define MOTOR_Y          (3)
#define MOTOR_Z          (0)
#define MOTOR_W          (1)

//慈善尺滤波尺寸
#define AVG_SIZE    (9)

//shui ping yi滤波尺寸
#define LEVEL_AVG_SIZE    (399)

#define XYV_AVG_SIZE    (1)

namespace BASE {

//////////////////////////////////// System internal structure  /////////////////////////////////////

////////////////////////////////////
//定义线程模块运行的状态
typedef enum
{
  M_STATE_INIT = 0,
  M_STATE_CONF,
  M_STATE_RUN,
  M_STATE_STOP,
  M_STATE_QUIT,
} M_STATE;

//定义线程模块在运行状态下执行各种条件结构体
typedef struct
{
  M_STATE   mState;         //模块处于的状态
  uint16_t   mMotorCmd;      //模块当前执行命令
  int16_t  mCycTimes;       //模块命令要循环周期数
  uint16_t  mWaitAck;       //模块要等待的ACK
} M_STATE_CONDITIONS;

//定义线程模块应答的状态
typedef enum
{
  ACK_STATE_NULL = 100,
  ACK_STATE_INIT_OK,
} ACK_STATE;

//for log part/////////////
//日志模块需要记录系统log或机械臂运行数据
typedef enum
{
  S_APP_LOGER = 0,
  S_ARMS_DATA,
} LOG_SAVA_W;


/**************************控制算法使用结构体**************************/
//二维位置向量,位置，速度，加速度，角度
typedef struct
{
  float x;
  float y;
} POS_2,ANGLE_2,ACC_2,VEL_2;

//四维位置向量速度，加速度，角度
typedef struct
{
  float v_p[4];
} VEL_4, POS_4;//单位为 rad/s

////////////////////////////////////11 组机械臂协议中的定义   /////////////////////////////////////
//协议中系统标识符号定义,上位机ID 控制版ID/////////////
/**********************************************
拉力计模块1-21为:0x02—0x16
数据中继模块1-21为: 0x17—0x2B
电机控制板1-21为: 0x42—0x56
上位机1-11为:0x57—0x61
数据站1-11为:0x62—0xCE
**********************************************/
const uint16_t   ID_CONTROL_PANEL_FIR = 0x42;
const uint16_t   ID_LEADER_FIR = 0x57;


//协议中消息类型////////////////////////////////
/**********************************************
数据消息:0x01(本地发给上位机)数据定义见附 3
设备控制消息:0x03(上位机发给本地,设置设备的状态表)消息格式与状态表见附 3
设备参数设置消息:0x04(上位机发给本地)
设备参数读取消息:0x05(上位机发给本地)
消息重发请求:0x07(双向)
消息正确应答:0x08(双向)
消息错误应答:0x09(双向)
**********************************************/
const uint16_t   M_TYPE_TO_LEADER_DATA = 0x01;
const uint16_t   M_TYPE_TO_CONTROLER_CMD = 0x03;
const uint16_t   M_TYPE_TO_CONTROLER_WCONF = 0x04;
const uint16_t   M_TYPE_TO_CONTROLER_RCONF = 0x05;
const uint16_t   M_TYPE_TO_ALL_REPEAT= 0x07;
const uint16_t   M_TYPE_TO_ALL_ANSWER_OK= 0x08;
const uint16_t   M_TYPE_TO_ALL_ANSWER_FAILED= 0x09;

//定义电机控制命令//////////////////////////////
/**********************************************
0x00 急停
0x01 正时针运动
0x02 逆时针运动
0x03 正常停止(带减速过程)
0x04 清除错误
0x05 梯形变速
0x06 立即变速
0x07 曲线改变位置
0x08 梯形改变位置
0x09 立即改变位置
**********************************************/
const uint8_t   CT_MOTOR_STOPE = 0x00;
const uint8_t   CT_MOTOR_MOVE_POSITIVE = 0x01;
const uint8_t   CT_MOTOR_MOVE_INVERSE  = 0x02;
const uint8_t   CT_MOTOR_STOP = 0x03;
const uint8_t   CT_MOTOR_CLEAR_ERROR = 0x04;

const uint8_t   CT_MOTOR_MOVE_V_TRAPEZIUM  = 0x05;
const uint8_t   CT_MOTOR_MOVE_V_IMMEDIATELY = 0x06;
const uint8_t   CT_MOTOR_MOVE_P_CURVE = 0x07;
const uint8_t   CT_MOTOR_MOVE_P_TRAPEZIUM  = 0x08;
const uint8_t   CT_MOTOR_MOVE_P_IMMEDIATELY = 0x09;

const uint8_t   CT_MOTOR_MOVE_POS = 0x10;

const uint8_t   CT_MOTOR_SIGNOUT = 0x09;

const uint8_t   CT_MOTOR_ZERO   = 0x13;


//定义控制板反馈状态
/*******************************************
0x00 正常
0x01 系统错误
0x02 内部设备模块错误
0x03 通讯模块无法通讯
0x04 电流异常
0x05 电池异常
0x06 传感器异常
0x07 电机异常
0x08 拉力计传感器异常
*******************************************/
const uint16_t   ST_SYS_OK   = 0x00;
const uint16_t   ST_SYS_ERROR    = 0x01;
const uint16_t   ST_SYS_MODULES_ERROR   = 0x02;
const uint16_t   ST_SYS_UN_COMMUNICATE    = 0x03;
const uint16_t   ST_SYS_ELECTRIC_ABNORMAL   = 0x04;
const uint16_t   ST_SYS_BATTERY_ABNORMAL    = 0x05;
const uint16_t   ST_SYS_SENSOR_ABNORMAL    = 0x06;
const uint16_t   ST_SYS_MOTOR_ABNORMAL   = 0x07;
const uint16_t   ST_SYS_TENSION_ABNORMAL    = 0x08;

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
//定义log打印保存字符串
typedef struct
{
  char  mString[PRINT_STRING_MAX_LENGTH];
} PRINT_STR;

//定义log打印保存循环队列
typedef struct
{
  PRINT_STR mPrintPond[PRINT_QUEUE_MAX_ITEMS];	// 循环队列的缓冲区
  uint32_t mFront;		// 循环队列的头指针
  uint32_t mRear;		// 循环队列的尾指针
} STR_QUEUE;

///////////////////////////////////
//定义电机控制结构体
typedef struct
{
  //ctrl motors data
  uint32_t    mCmd;
  int32_t     mPosition;
  //单位是rpm *1000 取整
  int32_t     mSpeed;   //单位为 脉冲/s
} MOTOR_CTRL;


//定义机械臂4组电机控制
typedef struct
{
  //four motors data
  MOTOR_CTRL mMotorsCmd[4];
} MOTORS;

///////////////////////////////////
//定义时间戳
typedef struct
{
  uint32_t   mSysTimeS;
  uint32_t   mSysTimeUs;
} SYS_TIME;


///////////////////////////////////
//定义机械臂控制板发送消息
typedef struct
{
  uint16_t   mIdentifier;
  //msg type
  uint16_t   mMsgType;
  //随机码，上位机，自加一
  uint16_t   mRandomCode;
  //states
  uint16_t   mStateCode;
  //时间戳，妙、微妙
  SYS_TIME   mSysTime;
  //数据长度
  uint32_t   mDataLength;
  //ctrl motors data
  MOTORS     mMotors;
  //MOTOR_CTRL mMotorsCmd[4];
  //crc 校验码
  uint16_t   mCrcCode;
} ARMS_S_MSG;

//定义无线拉力计控制板发送消息
typedef struct
{
  uint16_t   mIdentifier;
  //msg type
  uint16_t   mMsgType;
  //随机码，上位机，自加一
  uint16_t   mRandomCode;
  //states
  uint16_t   mStateCode;
  //时间戳，妙、微妙
  SYS_TIME   mSysTime;
  //数据长度
  uint16_t   mDataLength;
  //ctrl tensions start/stop data. 0 1
  uint8_t    mCmd;

  //mCrcCode++
  uint16_t mCrcCode;
} TENSIONS_S_MSG;

///////////////////////////////////
//定义单个电机接收数据结构体
typedef struct
{
  //rec motors datas
  uint8_t    mMotorStateCode;
  int32_t    mPosition;
  int32_t    mSpeed;           //单位为:度/s
  int32_t    mAcceleration;
  uint32_t   mEncoderTurns;
  uint16_t   mEncoderPulses;
} MOTOR_REC_DATAS;

//定义单个电机接收数据结构体
typedef struct
{
  //rec motors datas
  uint8_t    mMotorStateCode;
  float    mPosition;
  float    mSpeed;           //单位为:度/s
  float    mAcceleration;
  float   mEncoderTurns;
  float   mEncoderPulses;
} MOTOR_REC_USE_DATAS;

//定义机械臂控制板接收消息
typedef struct
{
  //frame unique dev 0-10
  uint16_t   mIdentifier;
  //msg type
  uint16_t   mMsgType;
  //随机码，上位机，自加一
  uint16_t   mRandomCode;
  //states
  uint16_t   mStateCode;
  //时间戳，妙、微妙
  SYS_TIME   mSysTime;
  //数据长度
  uint16_t   mDataLength;
//****************************  rec Datas  ***********************************//
  // Switch datas
  uint8_t    mSwitchStateCode;
  uint16_t   mSwitchTiggers;

  //inclinometers  datas 0-7,7-15
  uint8_t    mInclinometersStateCode;
  uint32_t   mInclinometer1_x;     //单位为角度
  uint32_t   mInclinometer1_y;     //单位为角度
  uint32_t   mInclinometer2_x;
  uint32_t   mInclinometer2_y;

  //inclinometers  datas 0-7,7-15
  uint8_t    mSikosStateCode;
  int32_t   mSiko1;            //单位为毫米
  int32_t   mSiko2;

  //encoder  data
  uint8_t    mEncoderStateCode;
  int32_t    mEncoderTurns;     //单位为角度*1000。
  uint16_t   mEncoderPulses;

  //四个电机数据
  MOTOR_REC_DATAS mMotors[4];

  //拉力计
  uint8_t    mTensionCode;
  int32_t    mTension;         //单位为g

  //crc 校验码
  uint8_t    mCrcCodeH;
  uint8_t    mCrcCodeL;
} ARMS_R_MSG;


//定义机械臂控制板接收消息
typedef struct
{
  //frame unique dev 0-10
  uint16_t   mIdentifier;
  //msg type
  uint16_t   mMsgType;
  //随机码，上位机，自加一
  uint16_t   mRandomCode;
  //states
  uint16_t   mStateCode;
  //时间戳，妙、微妙
  SYS_TIME   mSysTime;
  //数据长度
  uint16_t   mDataLength;
//****************************  rec Datas  ***********************************//
  // Switch datas
  uint8_t    mSwitchStateCode;
  uint16_t   mSwitchTiggers;

  float   mInclinometer1_x;     //单位为角度
  float   mInclinometer1_y;     //单位为角度

  float   mInclinometer1_dertX;     //单位为角度
  float   mInclinometer1_dertY;     //单位为角度

  float   mSiko1;            //单位为米
  float   mSiko2;

  float   mLevelSiko1;            //单位毫米
  float   mLevelSiko2;

  float   mEncoderTurns;     //单位为角度*1000。
  //四个电机数据,已经转换成float
  MOTOR_REC_USE_DATAS mMotors[4];

  float   mCmdSpeed[4];      //算法输入速度单位为:弧度/s
  float   mCmdPos[4];      //算法输入速度单位为:弧度/s
  int32_t mOverLap;

  float    mTension;         //单位为g
} ARMS_R_USE_MSG;

//定义无线拉力计控制板接收消息
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


//定义无线拉力计传输给leader数据
typedef struct
{
  float   mTensions;
  bool    iNewTensions;
} TENSIONS_NOW_DATA;

///////////////////////////////////
/******************************************人机界面***************************************/
//////////////////////interaction控制指令//////////////
const uint16_t  CMD_BASE = 0;
const uint16_t  CMD_LINK = CMD_BASE + 1;
const uint16_t  CMD_UNLINK = CMD_BASE + 2;
const uint16_t  CMD_SAVE_CONF = CMD_BASE + 3;
const uint16_t  CMD_READ_CONF = CMD_BASE + 4;

const uint16_t  CMD_ALL_MOVE_START = CMD_BASE + 7;
const uint16_t  CMD_ALL_MOVE_STOP = CMD_BASE + 8;
const uint16_t  CMD_ALL_PULL_START = CMD_BASE + 9;
const uint16_t  CMD_ALL_PULL_STOP = CMD_BASE + 10;

const uint16_t  CMD_RUN_START = CMD_BASE + 11;
const uint16_t  CMD_RUN_STOP = CMD_BASE + 12;
const uint16_t  CMD_RUN_STOP_E = CMD_BASE + 13;
const uint16_t  CMD_QUIT = CMD_BASE + 14;
const uint16_t  CMD_READ_PLAYBACK = CMD_BASE + 15;
const uint16_t  CMD_READ_CYC = CMD_BASE + 16;
const uint16_t  CMD_CLEAR_MOTOR_ERROR = CMD_BASE + 17;
const uint16_t  CMD_CALIBRATE_ARM = CMD_BASE + 18;
const uint16_t  CMD_CHANGE_ConfSaveSiko = CMD_BASE + 19;

//接收到上位机控制按钮后，运行期间leader陷入条件运行状态
const uint16_t  CMD_INTO_COND_RUN = CMD_BASE + 20;

//上位机循环读取显示数据命令
const uint16_t  CMD_CYC_READ_SYS_DELAYED = CMD_BASE + 25;
const uint16_t  CMD_CYC_READ_LIFT_DATAS = CMD_BASE + 26;
const uint16_t  CMD_CYC_READ_RUNNING_DATAS = CMD_BASE + 27;
const uint16_t  CMD_CYC_READ_SHOWDE_DATAS = CMD_BASE + 28;

//上位机发送控制，使得app运行状态
const uint16_t  ST_HAND_MOVE_RUNNING = CMD_BASE + 50;
const uint16_t  ST_ALL_MOVE_RUNNING = CMD_BASE + 51;
const uint16_t  ST_ALL_PULL_RUNNING = CMD_BASE + 52;
const uint16_t  ST_LIFT_STOPED = CMD_BASE + 53;

const uint16_t  ST_ALL_RUN_RUNNING = CMD_BASE + 54;
const uint16_t  ST_RUN_STOPED = CMD_BASE + 55;
const uint16_t  ST_ALL_CALBRATE = CMD_BASE + 56;

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
const uint16_t  CMD_ACK_READ_DELAYED_DATAS = CMD_BASE + 10;

const uint16_t  CMD_ACK_READ_PLAYBACK = CMD_BASE + 11;
const uint16_t  CMD_ACK_READ_CYC = CMD_BASE + 12;

//////////////////////CONF 模式下，interaction发送给Supr同步命令类型。便于supr分类上位机发送的按钮CMD//////////////
const uint16_t  CMD_TYPE_BASE = 0;


//下发配置数据发送的最大数据char
#define MSG_DOWN_DATA_MAX  400
//上传数据发送的最大数据char
#define MSG_UP_DATA_MAX  3000

#define MAGIC_AVG_SIZE (20)
/////////////////////////////////////////////////rec interactions/////////////////////////////////////
//上位机发送给app的消息
typedef struct
{
  //16位CtrlWord
  uint16_t   CmdIdentify;
  //数据段
  char Datas[MSG_DOWN_DATA_MAX];
  //序列码、随机码、保留位
  uint16_t CRC;
}MArmsDownData;

///////////////////////上传数据////////////////////////
//app发送给上位机显示的消息
typedef struct
{
    //16位stateWord
    uint16_t   StatusWord;
    //16位stateCode
    uint16_t   StatusCode;
    uint32_t   mRandomCode;
    //数据段
    char Datas[MSG_UP_DATA_MAX];
    //序列码、随机码、保留位
    uint16_t CRC;
}MArmsUpData;

//配置界面中单元的重量、编码器真值设置
typedef struct{
  float mConfSaveWeight;
  float mConfSaveSikoX;
  float mConfSaveSikoY;
  float mConfSaveEncoderT;

  //随动算法用到的pd值
  float mFollowKp;
  float mFollowKd;

  //拉力算法用到的
  float mWn;
  float mCo;

  //laliji
  float mConfTension;

  //start level x y
  float mInclinometerX;
  float mInclinometerY;

  int   mIsValid;
} ConfData;


//起重界面中单元的频率,时间抖动读取
typedef struct{
  int   mLiftHz;
  int   mLiftShake;
  int   mIsValid;
} ReadLiftHzData;

//起吊界面中，控制器接收arms运行数据
typedef struct{
  int   mMudoleNum;
  float mHandXMoveNow;
  float mHandYMoveNow;
  float mHandZMoveNow;
  float mHandWMoveNow;
} ReadLiftSigalNowData;


//起吊界面上位机控制数据
typedef struct{
  int    mMudoleNum;  //单元号
  int    isVevOrPos;  //速度或者位置控制
  float  v_p[4];      //四个电机控制数据
  float  posCmdVel;   //位置控制的设定速度
  int   mIsValid;     //是否有效
} LiftCmdData;


//起吊界面上位机控制数据
typedef struct{
  int    mMudoleNum;  //单元号
  float  mCalKg;   //
  int   mIsValid;     //是否有效
} CalibrateData;

//运行界面中细节信息显示,
typedef struct{
  int     mMudoleNum;
//运行界面中信息显示,
  float   runR_sysMsg_X;
  float   runR_sysMsg_Y;
  float   runR_sysMsg_Z;
  float   runR_sysMsg_AngleX;
  float   runR_sysMsg_AngleY;
  float   runR_sysMsg_Pull;
  float   runR_sysMsg_PullE;
//运行界面中细节调试显示,
  float   runD_Rsiko1;      //磁栅尺1
  float   runD_Rsiko2;      //磁栅尺2
  float   runD_Level1;      //水平仪1
  float   runD_Level2;      //水平仪2
  float   runD_RencoderT;   //翻转编码器
  float   runD_PullNow;     //拉力计当前值
  float   runD_PullSet;     //拉力计设定值
  float   runD_VXNow;     //X电机当前值
  float   runD_VXSet;     //电机设定值
  float   runD_VYNow;     //电机当前值
  float   runD_VYSet;     //电机设定值
  float   runD_VZNow;     //电机当前值
  float   runD_VZSet;     //电机设定值
  float   runD_VWNow;     //电机当前值
  float   runD_VWSet;     //电机设定值
  int     mIsValid;
} ReadRunAllData;


//interaction cmd datas to supr
//人机交互线程发送给supr线程的data
typedef struct{
  int            mIsNewRec;
  int            mIsNewSend;
  MArmsDownData  mRecMsg;
  //MArmsUpData    mSendMsg;

  pthread_mutex_t mInteractionRecMutex;
  pthread_mutex_t mInteractionSendMutex;
} InteractionDataToSupr;

//interaction. supr to interaction
//supr传输给人机交互线程的数据，机械臂运行数据
typedef struct{
  BASE::ReadLiftHzData        mReadLiftHzDatas[DEF_SYS_MAX_ARMS_NUMS];
  BASE::ARMS_R_USE_MSG        mRecMsgsDatas[DEF_SYS_MAX_ARMS_NUMS];
  pthread_mutex_t             mArmsNowDatasMutex;
} SuprDataToInteraction;

////////////////////////////////控制算法数据///////////////////////////////////////////////////
typedef struct
{
  /******************初始参数***********************/
  //悬掉重物质量，旋转臂长、转动惯量、弹簧弹性系数、绳索卷筒半径、减速机的减速比、三周期传感器角速度、三周期传感器拉力、重力加速度、阻尼系数、角速度、K1系数
  float  mL, mJ, mK, mR, mNdecrease, alfa_reco[3],F_reco[3],mK1;
  float mSikoK;
  /*********************拉力平衡算法*********************/
  //绳索释放加速度,电机释放角速度,上一个周期释放绳索加速度
  float  last_dd_Lz;

  //执行的速度值
  BASE::VEL_4 mCmdV;

  /*******************随动算法数据*******************/
  //绳索末端偏离水平位置的距离,上一周期L
  BASE::POS_2  mRopeEndLastL;
}MagicControlData;


////////////////////////////////上位机控制命令数据///////////////////////////////////////////////////
typedef struct
{
  //执行的速度值
  BASE::VEL_4 mCmdV, mCmdPosXYZ, mCmdPosXYZW;
  float   mCmdTension;
  float   mCalibKg;
}IntCmdData;

/////////////////////////////////整个系统中线程定义 //////////////////////////////////////////////
//线程结构体头，线程运行期间必须的参数 Each thread runs parameters
typedef struct
{
  bool         mWorking;
  char         mThreadName[15];

  uint32_t     mSocket;
  sockaddr_in  mSerAddr, mPeerAddr;
  uint32_t     mMyPort;
  char         mMyIpV4Str[STR_IPV4_LENGTH];
  uint32_t     mPeerPort;
  char         mPeerIpV4Str[STR_IPV4_LENGTH];
  M_STATE      mState;

  //log queue pri
  STR_QUEUE*   mLogQueue;
  pthread_mutex_t *mPrintQueueMutex;

  //cpu mask
  int          mCpuAffinity;

  //标识当前leader的ID
  uint16_t     mMsgId;

  //运行周期
  uint32_t     mRCnt;

  //running状态，运行周期计数
  uint32_t     mMagicRCnt;

} THREAD_INFO_HEADER;

//日志线程结构体信息loger thread info
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


//11个机械臂线程信息 11 arms thread info
typedef struct: public THREAD_INFO_HEADER
{
  ACK_STATE       mAckState;

  bool            mNewRecMsg;
  ARMS_R_MSG      mRecMsg;
  ARMS_R_USE_MSG  mRecUseMsg;

  ARMS_S_MSG      mSendMsg;
  pthread_mutex_t mArmsMsgMutex;
  pthread_cond_t  mArmsMsgReady;

  //motor fangxiang
  int motorDirection[4];

  //level change siko direction
  int levelChangeSikoXYDirection[2];
  float levelSikoXY_L;

  //rec motor cmd
  int                mIsNowMotorCmd;
  LiftCmdData      mMoveData;
  CalibrateData    mCaliData;

  //电机传输命令锁
  pthread_mutex_t   mMotorMutex;


  //无线数据传输模块，如果不使用无线模块，此值在MArmsUpData中
  TENSIONS_NOW_DATA   *mNowTension;

  //线程在每个状态下运行条件
  BASE::M_STATE_CONDITIONS mCond;

  //标定参数，初始编码器真值，重物参数，内存在supr
  BASE::ConfData *mConfParam;

////////////控制算法需要的数据////////////////////////////////
  BASE::MagicControlData mMagicControl;

  //系统控制周期
  float sysDt;

  //Inclinometer avg
  POS_2 mDertInclim[LEVEL_AVG_SIZE+1];

  //慈善尺滤波保存数组
  POS_2 mSikoAvg[AVG_SIZE+1];
  int32_t mTensionAvg[AVG_SIZE+1];
  float magic_v[AVG_SIZE+1];
  VEL_2 magic_XYv[XYV_AVG_SIZE+1];
  float magic_Encoder[AVG_SIZE+1];

  float magic_d_alfi[AVG_SIZE+1];
  float magic_f_estime[AVG_SIZE+1];

////////////上位机发送的控制命令数据////////////////////////////////
  BASE::IntCmdData  mIntCmd;
  int *mIsRun;
  //随即码
  uint16_t   mRandomCode;

  //
  float      mPoxR,mPoxTx,mPoxTy,mPoxTz;

} ARMS_THREAD_INFO;

//拉力计线程信息 tensions thread info
typedef struct: public THREAD_INFO_HEADER
{
  TENSIONS_NOW_DATA   *mNowTension;
} TENSIONS_THREAD_INFO;


//人机交互线程信息 interaction thread info
typedef struct: public THREAD_INFO_HEADER
{
  bool      mIsStataChange;
  M_STATE   mNewState;

  MArmsDownData  mRecMsg;
  MArmsUpData    mSendMsg;

  InteractionDataToSupr  *mInterToSuprDatas;
  SuprDataToInteraction  *mSuprDatasToInterasction;
  //标定参数 所有模块的，初始编码器真值，重物参数，内存在supr
  BASE::ConfData *mConfParam;
  int * mIsRun;

}INTERACTION_THREAD_INFO;


}//namespace

#endif // SYS_ARMS_DEFS_H
