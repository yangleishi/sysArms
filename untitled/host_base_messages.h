/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name : host_base_messages.h
* Module :
* Create on: 2021/5/10
* Author: 师洋磊
* Email: 546783926@qq.com
* Description: 消息相关的结构体头文件
*
********************************************************************************/

#ifndef HOST_BASE_MESSAGES_H
#define HOST_BASE_MESSAGES_H
#include <stdlib.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include <QMetaType>
#include <QString>
#include <QRect>

///////////////////////////////////////
namespace BASE {

#define MSG_DATA_SIZE  32
#define MSG_DRAW_DATA_SIZE  1000

//系统中机械臂最大数据数
#define SYS_ARMS_MAX_SIZE  8

//下发配置数据发送的最大数据char
#define MSG_DOWN_DATA_MAX  400

//上传数据发送的最大数据char
#define MSG_UP_DATA_MAX  3000

//显示界面最多能显示的图形
#define SHOW_MAX_MODULS  5

#define SHOW_DETAILS_MUD_MUNS 9


/////////////////////////////////////线程相关define////////////////////////////////////////////////////
/////定义线程运行状态（0 1 2 3）///////////////
//线程刚启动后到初始化配置之前处于该状态
const int STATE_THREAD_CONFIGURE = 0;
//线程刚初始化配置成功后到正常工作之前处于该状态，如初始化配置失败则状态0
const int STATE_THREAD_INITOK= 1;
//线程正常工作期间处于该状态，比如vicon数据正在传输
const int STATE_THREAD_RUNNING= 2;
//
const int STATE_THREAD_BEFFCONFING= 3;

//线程停止工作时候处于该状态
const int STATE_THREAD_STOP= 4;



/////定义线程的id号//////////////////////////
const int THREAD_ID_WIND = 0;
const int THREAD_ID_DRAWCURVE = THREAD_ID_WIND + 1;
const int THREAD_ID_LINKER = THREAD_ID_WIND + 2;
const int THREAD_ID_CURVEWIND = THREAD_ID_WIND + 3;
const int THREAD_ID_CONFBEFF = THREAD_ID_WIND + 4;

const int THREAD_MAX_ID = THREAD_ID_WIND + 4;

const int WIDGET_ID_CURVE = THREAD_ID_WIND + 50;
//////////////线程运行状态///////////////////
typedef enum {
  _MODULE_INIT = 0,
  _MODULE_INIT_OK,
  _MODULE_CONF,
  _MODULE_CONF_OK,
  _MODULE_RUNNING,
  _MODULE_STOP,
  _MODULE_EXIT,
} _MODULE_RUN_STATE;
typedef struct _MODULEINFOS {
  int mThreadId;
  _MODULE_RUN_STATE mCurRState;
} MODULEINFOS;

//////////////////////消息通知//////////////
const int MSG_NOTICE = 100;
const int MSG_NOTICE_INIT_ALL = MSG_NOTICE + 1;
const int MSG_NOTICE_INIT_ALL_ACK = MSG_NOTICE + 2;

const int MSG_NOTICE_CONF_ALL = MSG_NOTICE + 3;
const int MSG_NOTICE_CONF_ALL_ACK = MSG_NOTICE + 4;

const int MSG_NOTICE_RUN_ALL = MSG_NOTICE + 5;
const int MSG_NOTICE_RUN_ALL_ACK = MSG_NOTICE + 6;

const int MSG_NOTICE_STOP_ALL = MSG_NOTICE + 7;
const int MSG_NOTICE_STOP_ALL_ACK = MSG_NOTICE + 8;

const int MSG_NOTICE_EMS_ALL = MSG_NOTICE + 9;
const int MSG_NOTICE_EMS_ALL_ACK = MSG_NOTICE + 10;


const int MSG_NOTICE_LINK = MSG_NOTICE + 11;
const int MSG_NOTICE_UNLINK = MSG_NOTICE + 12;

/******************配置界面************************************/
const int MSG_NOTICE_SCONF = MSG_NOTICE + 13;

const int MSG_NOTICE_RCONF = MSG_NOTICE + 15;
const int MSG_NOTICE_CALIBRATE = MSG_NOTICE + 16;

/******************起重界面************************************/
const int MSG_NOTICE_HZ_SHAKE = MSG_NOTICE + 17;
const int MSG_ACK_HZ_SHAKE = MSG_NOTICE + 18;

const int MSG_NOTICE_HAND_MOVE = MSG_NOTICE + 19;
const int MSG_ACK_HAND_MOVE = MSG_NOTICE + 20;
const int MSG_NOTICE_HAND_MOVE_STOP = MSG_NOTICE + 21;

const int MSG_NOTICE_ALL_MOVE = MSG_NOTICE + 22;
const int MSG_NOTICE_ALL_MOVE_STOP = MSG_NOTICE + 23;

const int MSG_NOTICE_ALL_PULL = MSG_NOTICE + 24;
const int MSG_NOTICE_ALL_PULL_STOP = MSG_NOTICE + 25;

/******************运行界面************************************/
const int MSG_NOTICE_RUN_START = MSG_NOTICE + 26;
const int MSG_NOTICE_RUN_STOP = MSG_NOTICE + 27;
const int MSG_NOTICE_RUN_STOPE = MSG_NOTICE + 28;
const int MSG_NOTICE_RUN_PLAY = MSG_NOTICE + 29;
const int MSG_NOTICE_RUN_PLAY_STOP = MSG_NOTICE + 30;
//所有细节通知
const int MSG_ACK_RUN_SHOW = MSG_NOTICE + 39;

/******************细节显示界面************************************/
const int MSG_NOTICE_DRAW_CURVE = MSG_NOTICE + 40;
const int MSG_NOTICE_CURVEWID_CLOSE = MSG_NOTICE + 41;

const int MSG_NOTICE_CURVEWID_SHOW = MSG_NOTICE + 42;
const int MSG_NOTICE_CURVE_DATA = MSG_NOTICE + 43;

/*************************ack msg******************************/
const int MSG_NOTICE_ACK_LINK = MSG_NOTICE + 50;
const int MSG_NOTICE_ACK_SCONF = MSG_NOTICE + 51;
const int MSG_NOTICE_ACK_RCONF = MSG_NOTICE + 52;


const int MSG_NOTICE_ACK_UNLINK = MSG_NOTICE + 53;
const int MSG_NOTICE_ACK_CYC = MSG_NOTICE + 54;

const int MSG_NOTICE_KILL_ALL = MSG_NOTICE + 99;


const int MSG_NOTICE_SHOW_FREQUENCY= 300;

//定义消息通知的值////////////////////////////
const int MSG_VALUE = 500;

const int MSG_VALUE_INIT_OK = MSG_VALUE + 1;
const int MSG_VALUE_INIT_FAIL = MSG_VALUE + 2;
const int MSG_VALUE_CONF_OK = MSG_VALUE + 3;
const int MSG_VALUE_CONF_FAIL = MSG_VALUE + 4;
const int MSG_VALUE_RUN_OK = MSG_VALUE + 5;
const int MSG_VALUE_RUN_FAIL = MSG_VALUE + 6;
const int MSG_VALUE_STOP_OK = MSG_VALUE + 7;
const int MSG_VALUE_STOP_FAIL = MSG_VALUE + 8;
const int MSG_VALUE_EMS_OK = MSG_VALUE + 9;
const int MSG_VALUE_EMS_FAIL = MSG_VALUE + 10;


//////////////////////控制指令//////////////
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
const uint16_t  CMD_QIUT = CMD_BASE + 14;
const uint16_t  CMD_READ_PLAYBACK = CMD_BASE + 15;
const uint16_t  CMD_READ_CYC = CMD_BASE + 16;
const uint16_t  CMD_CLEAR_MOTOR_ERROR = CMD_BASE + 17;
const uint16_t  CMD_CALIBRATE_ARM = CMD_BASE + 18;


//循环的读取arms数据
const uint16_t  CMD_CYC_READ_SYS_DELAYED = CMD_BASE + 25;
const uint16_t  CMD_CYC_READ_LIFT_DATAS = CMD_BASE + 26;
//const uint16_t  CMD_CYC_READ_RUNNING_DATAS = CMD_BASE + 27;
const uint16_t  CMD_CYC_READ_SHOWDE_DATAS = CMD_BASE + 28;


///////////////////////interaction ACK指令///////////////////////////////
const uint16_t  CMD_ACK_BASE = 0;
const uint16_t  CMD_ACK_LINK_OK = CMD_ACK_BASE + 1;
const uint16_t  CMD_ACK_LINK_FAILD = CMD_ACK_BASE + 2;
const uint16_t  CMD_ACK_UNLINK_OK = CMD_ACK_BASE + 3;

const uint16_t  CMD_ACK_SAVECONF_OK = CMD_ACK_BASE + 4;
const uint16_t  CMD_ACK_SAVECONF_FAILD = CMD_ACK_BASE + 5;
const uint16_t  CMD_ACK_READCONF_OK = CMD_ACK_BASE + 6;

//循环的ACK arms数据
const uint16_t  CMD_ACK_READ_LIFT_DATAS = CMD_BASE + 7;
const uint16_t  CMD_ACK_READ_RUNNING_DATAS = CMD_BASE + 8;
const uint16_t  CMD_ACK_READ_SHOWDE_DATAS = CMD_BASE + 9;
const uint16_t  CMD_ACK_READ_DELAYED_DATAS = CMD_BASE + 10;
const uint16_t  CMD_ACK_READ_PLAYBACK = CMD_BASE + 11;
const uint16_t  CMD_ACK_READ_CYC = CMD_BASE + 12;


//显示绘图消息
const int  CMD_DRAW_BASE = 0;
const int  CMD_DRAW_START = CMD_DRAW_BASE + 1;  //点击显示绘图
const int  CMD_DRAW_ADD = CMD_DRAW_BASE + 2;    //点击多选框，增加一个绘图
const int  CMD_DRAW_CLEAR = CMD_DRAW_BASE + 3;  //清除多选框,清除绘图
const int  CMD_DRAW_CLEAR_ALL = CMD_DRAW_BASE + 4;  //清除全部绘图，
const int  CMD_DRAW_FIRE = CMD_DRAW_BASE + 5;  //清除全部绘图


const int  CMD_DRAW_QUIT = CMD_DRAW_BASE + 10;  //清除全部绘图，


typedef struct
{
  uint32_t   mSysTimeS;
  uint32_t   mSysTimeUs;
} SYS_TIME;

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

  float   mInclinometer1_x;     //单位为角度
  float   mInclinometer1_y;     //单位为角度

  float   mSiko1;            //单位为毫米
  float   mSiko2;
  float   mEncoderTurns;     //单位为角度*1000。
  //四个电机数据,已经转换成float
  MOTOR_REC_USE_DATAS mMotors[4];

  float   mCmdSpeed[4];      //算法输入速度单位为:弧度/s
  float   mCmdPos[4];      //算法输入速度单位为:弧度/s
  int32_t mOverLap;

  float    mTension;         //单位为g
} ARMS_R_USE_MSG;

//多播数据结构
typedef struct
{
  //frame unique dev
  uint32_t   mIdentifier;
  //随机码
  uint32_t   mRandomCode;

  //启停命令
  uint32_t   mCmd;

  //系统状态
  uint32_t   mStatue;

  //数据有效标识,标识传输的数据是否有效
  int32_t    mMark[8];
  //8套机械臂拉力计
  int32_t    mTension[8];         //单位为g

  //8 pos
  int32_t    mPosX[8];
  int32_t    mPosY[8];
  int32_t    mPosZ[8];

  //需要哪些数据随后加

} ARMS_MULTICAST_UDP;


/////////////////////////////////////////////////////下发配置数据////////////////////////////////////////
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

  /////////////////////////////////////////////////////上传数据////////////////////////////////////////
typedef struct
{
    //*******************16位stateWord******************************************/
    uint16_t   StatusWord;
    //*******************16位stateCode******************************************/
    uint16_t   StatusCode;
    //*******************数据段**********************/
    uint32_t   mRandomCode;
    //下发的数据
    char Datas[MSG_UP_DATA_MAX];
    //*******************序列码、随机码、保留位**********************/
    uint16_t CRC;
}MArmsUpData;

//配置界面中单元的重量、编码器真值设置
typedef struct{
  float mConfSaveWeight;
  float mConfSaveSikoX;
  float mConfSaveSikoY;
  float mConfSaveEncoderT;

  float mFollowKp;
  float mFollowKd;

  float mWn;
  float mCo;

  float mConfTension;

  int   mIsValid;
} ConfData;


//起重界面中单元的频率,时间抖动读取
typedef struct{
  int   mDelayedS;
  int   mDelayedUs;
  int   mIsValid;
} ReadLiftHzData;


//起吊界面上位机控制数据
typedef struct{
  int     mMudoleNum;  //单元号
  int     isVelOrPos;  //速度或者位置控制
  float   v_p[4];      //四个电机数据
  float   posCmdVel;   //位置控制设定速度
  int     mIsValid;    //是否有效
} LiftCmdData;

//起吊界面上位机控制数据
typedef struct{
  int     mMudoleNum;  //单元号
  float   mCalKg;   //位置控制设定速度
  int     mIsValid;    //是否有效
} CalibrateData;

typedef enum {
  SIKO_X = 0,
  SIKO_Y,
  INCLINOMETER,
  TENSION,
  VEL_X,
  VEL_Y,
  VEL_Z,
  POS_W,
  ANGLE_ENCODER,
  MAX_END,
} DRAW_SENSOR_MARK;

//运行界面中细节信息显示,UDP数据包发送给显示程序
typedef struct{
  ARMS_R_USE_MSG mRunData;
  //多选框中要显示模块传感器标志
  int  drawMark[SHOW_DETAILS_MUD_MUNS];
  //通知
  int m_Notice;
  int m_Value;
} DrawData;


////////////////////////////////线程之间通信结构体////////////////////////
//发送通知的消息
typedef  struct
{
  //发送者、接受者
  int m_Sender;
  int m_Reciver;

  //通知
  int m_Notice;
  int m_Value;
  int m_Value1[6];

  //要发送数据的指针或数组
  char *m_MsgPData;
}SigalMessages;

#pragma pack()


//autohanging
typedef struct
{
  //rec motors datas
  uint32_t   mIdentifier;

  int32_t   mMsgId;

  uint32_t  mRandCode;

  int32_t   mStatuesCode;

  uint32_t  mTimeS;

  uint32_t  mTimeUs;

  uint32_t  mDataSize;

  char      mData[400];

  uint16_t  mCrc;

} AUTO_HANGING_MSG;


/********************************标签名字***************************/
const QString labelConfStrings[9]={"选择单元","拉力设置(kg)","X向编码器","Y向编码器","Z向编码器","拉力平衡编码器","板转编码器","磁栅尺","水平仪"};
const QRect   LabelConfRect[9] = {QRect(10, 10, 80, 20),QRect(120, 10, 100, 20),QRect(330, 10, 80, 20),
                                  QRect(530, 10, 80, 20),QRect(730, 10, 80, 20),QRect(900, 10, 120, 20),
                                  QRect(1120, 10, 80, 20),QRect(1340, 10, 80, 20),QRect(1540, 10, 80, 20)};

const QString ModulesStrings[8]={"单元1","单元2","单元3","单元4","单元5","单元6","单元7","单元8"};
const QRect   checkConfRect[8] = {QRect(10, 70, 91, 19),QRect(10, 120, 91, 19),QRect(10, 170, 91, 19),QRect(10, 220, 91, 19),
                                  QRect(10, 270, 91, 19),QRect(10, 320, 91, 19),QRect(10, 370, 91, 19),QRect(10, 420, 91, 19)};

const QRect LineConfReadSikoXRect[8] = {QRect(80, 70, 61, 21),QRect(80, 120, 61, 21),QRect(80, 170, 61, 21),QRect(80, 220, 61, 21),
                                          QRect(80, 270, 61, 21),QRect(80, 320, 61, 21),QRect(80, 370, 61, 21),QRect(80, 420, 61, 21)};
const QRect LineConfReadSikoYRect[8] = {QRect(150, 70, 61, 21),QRect(150, 120, 61, 21),QRect(150, 170, 61, 21),QRect(150, 220, 61, 21),
                                           QRect(150, 270, 61, 21),QRect(150, 320, 61, 21),QRect(150, 370, 61, 21),QRect(150, 420, 61, 21)};




const QRect   LiftRect[8] = {QRect(10, 70, 71, 19),QRect(10, 120, 71, 19),QRect(10, 170, 71, 19),QRect(10, 220, 71, 19),
                              QRect(10, 270, 71, 19),QRect(10, 320, 71, 19),QRect(10, 370, 71, 19),QRect(10, 420, 71, 19)};



const QRect LineLiftReadHandXNowRect[8] = {QRect(250, 70, 71, 21),QRect(250, 120, 71, 21),QRect(250, 170, 71, 21),QRect(250, 220, 71, 21),
                                            QRect(250, 270, 71, 21),QRect(250, 320, 71, 21),QRect(250, 370, 71, 21),QRect(250, 420, 71, 21)};

const QRect LineLiftReadHandYNowRect[8] = {QRect(330, 70, 71, 21),QRect(330, 120, 71, 21),QRect(330, 170, 71, 21),QRect(330, 220, 71, 21),
                                            QRect(330, 270, 71, 21),QRect(330, 320, 71, 21),QRect(330, 370, 71, 21),QRect(330, 420, 71, 21)};

const QRect LineLiftReadHandZNowRect[8] = {QRect(420, 70, 71, 21),QRect(420, 120, 71, 21),QRect(420, 170, 71, 21),QRect(420, 220, 71, 21),
                                            QRect(420, 270, 71, 21),QRect(420, 320, 71, 21),QRect(420, 370, 71, 21),QRect(420, 420, 71, 21)};

const QRect LineLiftReadHandWNowRect[8] = {QRect(510, 70, 71, 21),QRect(510, 120, 71, 21),QRect(510, 170, 71, 21),QRect(510, 220, 71, 21),
                                            QRect(510, 270, 71, 21),QRect(510, 320, 71, 21),QRect(510, 370, 71, 21),QRect(510, 420, 71, 21)};

const QRect LineConfReadPullRect[8] = {QRect(600, 70, 71, 21),QRect(600, 120, 71, 21),QRect(600, 170, 71, 21),QRect(600, 220, 71, 21),
                                        QRect(600, 270, 71, 21),QRect(600, 320, 71, 21),QRect(600, 370, 71, 21),QRect(600, 420, 71, 21)};

const QRect LineConfReadEncoderTRect[8] = {QRect(690, 70, 71, 21),QRect(690, 120, 71, 21),QRect(690, 170, 71, 21),QRect(690, 220, 71, 21),
                                            QRect(690, 270, 71, 21),QRect(690, 320, 71, 21),QRect(690, 370, 71, 21),QRect(690, 420, 71, 21)};

const QRect LineConfReadLevelXRect[8] = {QRect(790, 70, 71, 21),QRect(790, 120, 71, 21),QRect(790, 170, 71, 21),QRect(790, 220, 71, 21),
                                         QRect(790, 270, 71, 21),QRect(790, 320, 71, 21),QRect(790, 370, 71, 21),QRect(790, 420, 71, 21)};
const QRect LineConfReadLevelYRect[8] = {QRect(870, 70, 71, 21),QRect(870, 120, 71, 21),QRect(870, 170, 71, 21),QRect(870, 220, 71, 21),
                                         QRect(870, 270, 71, 21),QRect(870, 320, 71, 21),QRect(870, 370, 71, 21),QRect(870, 420, 71, 21)};

const QRect LineRunPosXRect[8] = {QRect(990, 70, 71, 21),QRect(990, 120, 71, 21),QRect(990, 170, 71, 21),QRect(990, 220, 71, 21),
                                   QRect(990, 270, 71, 21),QRect(990, 320, 71, 21),QRect(990, 370, 71, 21),QRect(990, 420, 71, 21)};
const QRect LineRunPosYRect[8] = {QRect(1070, 70, 71, 21),QRect(1070, 120, 71, 21),QRect(1070, 170, 71, 21),QRect(1070, 220, 71, 21),
                                   QRect(1070, 270, 71, 21),QRect(1070,320, 71, 21),QRect(1070, 370, 71, 21),QRect(1070, 420, 71, 21)};
const QRect LineRunPosZRect[8] = {QRect(1150, 70, 71, 21),QRect(1150, 120, 71, 21),QRect(1150, 170, 71, 21),QRect(1150, 220, 71, 21),
                                   QRect(1150, 270, 71, 21),QRect(1150, 320, 71, 21),QRect(1150, 370, 71, 21),QRect(1150, 420, 71, 21)};

const QRect LineRunOverLap[8] = {QRect(1230, 70, 71, 21),QRect(1230, 120, 71, 21),QRect(1230, 170, 71, 21),QRect(1230, 220, 71, 21),
                                   QRect(1230, 270, 71, 21),QRect(1230, 320, 71, 21),QRect(1230, 370, 71, 21),QRect(1230, 420, 71, 21)};


const QRect LiftAllCheckBoxRect[8] = {QRect(1060, 500, 71, 19),QRect(1130, 500, 71, 19),QRect(1060, 550, 71, 19),QRect(1130, 550, 71, 19),
                                       QRect(1060, 600, 71, 19),QRect(1130, 600, 71, 19),QRect(1060, 650, 71, 19),QRect(1130, 650, 71, 19)};

const QRect LineRunErrorAngleXRect[8] = {QRect(1330, 70, 71, 21),QRect(1330, 120, 71, 21),QRect(1330, 170, 71, 21),QRect(1330, 220, 71, 21),
                                          QRect(1330, 270, 71, 21),QRect(1330,320, 71, 21),QRect(1330, 370, 71, 21),QRect(1330, 420, 71, 21)};
const QRect LineRunErrorAngleYRect[8] = {QRect(1410, 70, 71, 21),QRect(1410, 120, 71, 21),QRect(1410, 170, 71, 21),QRect(1410, 220, 71, 21),
                                          QRect(1410, 270, 71, 21),QRect(1410, 320, 71, 21),QRect(1410, 370, 71, 21),QRect(1410, 420, 71, 21)};
const QRect LineRunErrorPullRect[8] = {QRect(1490, 70, 71, 21),QRect(1490, 120, 71, 21),QRect(1490, 170, 71, 21),QRect(1490,220, 71, 21),
                                        QRect(1490, 270, 71, 21),QRect(1490, 320, 71, 21),QRect(1490, 370, 71, 21),QRect(1490, 420, 71, 21)};



const QRect RunAllCheckBoxRect[8] = {QRect(1400, 500, 71, 19),QRect(1480, 500, 71, 19),QRect(1560, 500, 71, 19),QRect(1640, 500, 71, 19),
                                       QRect(1400, 550, 71, 19),QRect(1480, 550, 71, 19),QRect(1560, 550, 71, 19),QRect(1640, 550, 71, 19),
                                      };

const QRect   switchRect[8] = {QRect(1580, 70, 50, 19),QRect(1580, 120, 50, 19),QRect(1580, 170, 50, 19),QRect(1580, 220, 50, 19),
                              QRect(1580, 270, 50, 19),QRect(1580, 320, 50, 19),QRect(1580, 370, 50, 19),QRect(1580, 420, 50, 19)};

const QRect LabelRunRect[8] = {QRect(20, 70, 41, 21),QRect(20, 120, 41, 21),QRect(20, 170, 41, 21),QRect(20, 220, 41, 21),
                                QRect(20, 270, 41, 21),QRect(20, 320, 41, 21),QRect(20, 370, 41, 21),QRect(20, 420, 41, 21)};


const QString labelDetailStrings[10]={"磁栅尺X","磁栅尺Y","水平仪","拉力计","X速度","Y速度","Z速度","W位置","编码器转角","其他"};

const QRect CheckBoxDetail1Rect[9] = {QRect(380, 60, 91, 21),QRect(380, 100, 91, 21),QRect(380, 140, 91, 21),QRect(380, 180, 91, 21),
                                      QRect(380, 220, 91, 21),QRect(380, 260, 91, 21),QRect(380, 300, 91, 21),QRect(380, 340, 101, 21),QRect(380, 380, 101, 21)};



}//ened namespace BASE

/////////qt中声明自定义结构体使用信号和槽/////////////////////////////

Q_DECLARE_METATYPE(BASE::SigalMessages);


#endif // HOST_BASE_MESSAGES_H
