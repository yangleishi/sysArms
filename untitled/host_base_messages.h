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

  float    mTension;         //单位为g
} ARMS_R_USE_MSG;

//多播数据结构
typedef struct
{
  //frame unique dev
  uint32_t   mIdentifier;
  //随机码
  uint32_t   mRandomCode;

  //数据有效标识,标识传输的数据是否有效
  int32_t    mMark[8];
  //8套机械臂拉力计
  int32_t    mTension[8];         //单位为g
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

/********************************标签名字***************************/
const QString labelConfStrings[9]={"选择单元","拉力设置(kg)","X向编码器","Y向编码器","Z向编码器","拉力平衡编码器","板转编码器","磁栅尺","水平仪"};
const QRect   LabelConfRect[9] = {QRect(10, 10, 80, 20),QRect(120, 10, 100, 20),QRect(330, 10, 80, 20),
                                  QRect(530, 10, 80, 20),QRect(730, 10, 80, 20),QRect(900, 10, 120, 20),
                                  QRect(1120, 10, 80, 20),QRect(1340, 10, 80, 20),QRect(1540, 10, 80, 20)};

const QString ModulesStrings[11]={"单元1","单元2","单元3","单元4","单元5","单元6","单元7","单元8","单元9","单元10","单元11"};
const QRect   checkConfRect[11] = {QRect(10, 70, 91, 19),QRect(10, 120, 91, 19),QRect(10, 170, 91, 19),QRect(10, 220, 91, 19),
                                  QRect(10, 270, 91, 19),QRect(10, 320, 91, 19),QRect(10, 370, 91, 19),QRect(10, 420, 91, 19),
                                  QRect(10, 470, 91, 19),QRect(10, 520, 91, 19),QRect(10, 570, 91, 19)};


const QRect LineConfReadPullRect[11] = {QRect(700, 70, 81, 21),QRect(700, 120, 81, 21),QRect(700, 170, 81, 21),QRect(700, 220, 81, 21),
                                        QRect(700, 270, 81, 21),QRect(700, 320, 81, 21),QRect(700, 370, 81, 21),QRect(700, 420, 81, 21),
                                        QRect(700, 470, 81, 21),QRect(700, 520, 81, 21),QRect(700, 570, 81, 21)};


const QRect LineConfReadEncoderTRect[11] = {QRect(800, 70, 81, 21),QRect(800, 120, 81, 21),QRect(800, 170, 81, 21),QRect(800, 220, 81, 21),
                                            QRect(800, 270, 81, 21),QRect(800, 320, 81, 21),QRect(800, 370, 81, 21),QRect(800, 420, 81, 21),
                                            QRect(800, 470, 81, 21),QRect(800, 520, 81, 21),QRect(800, 570, 81, 21)};


const QRect LineConfReadSikoXRect[11] = {QRect(100, 70, 61, 21),QRect(100, 120, 61, 21),QRect(100, 170, 61, 21),QRect(100, 220, 61, 21),
                                          QRect(100, 270, 61, 21),QRect(100, 320, 61, 21),QRect(100, 370, 61, 21),QRect(100, 420, 61, 21),
                                          QRect(100, 470, 61, 21),QRect(100, 520, 61, 21),QRect(100, 570, 61, 21)};
const QRect LineConfReadSikoYRect[11] = {QRect(170, 70, 71, 21),QRect(170, 120, 71, 21),QRect(170, 170, 71, 21),QRect(170, 220, 71, 21),
                                           QRect(170, 270, 71, 21),QRect(170, 320, 71, 21),QRect(170, 370, 71, 21),QRect(170, 420, 71, 21),
                                           QRect(170, 470, 71, 21),QRect(170, 520, 71, 21),QRect(170, 570, 71, 21)};

const QRect LineConfReadLevelXRect[11] = {QRect(920, 70, 81, 21),QRect(920, 120, 81, 21),QRect(920, 170, 81, 21),QRect(920, 220, 81, 21),
                                         QRect(920, 270, 81, 21),QRect(920, 320, 81, 21),QRect(920, 370, 81, 21),QRect(920, 420, 81, 21),
                                         QRect(920, 470, 81, 21),QRect(920, 520, 81, 21),QRect(920, 570, 81, 21)};
const QRect LineConfReadLevelYRect[11] = {QRect(1020, 70, 81, 21),QRect(1020, 120, 81, 21),QRect(1020, 170, 81, 21),QRect(1020, 220, 81, 21),
                                         QRect(1020, 270, 81, 21),QRect(1020, 320, 81, 21),QRect(1020, 370, 81, 21),QRect(1020, 420, 81, 21),
                                         QRect(1020, 470, 81, 21),QRect(1020, 520, 81, 21),QRect(1020, 570, 81, 21)};


const QRect   LiftRect[11] = {QRect(10, 70, 71, 19),QRect(10, 120, 71, 19),QRect(10, 170, 71, 19),QRect(10, 220, 71, 19),
                              QRect(10, 270, 71, 19),QRect(10, 320, 71, 19),QRect(10, 370, 71, 19),QRect(10, 420, 71, 19),
                              QRect(10, 470, 71, 19),QRect(10, 520, 71, 19),QRect(10, 570, 71, 19)};



const QRect LineLiftReadHandXNowRect[11] = {QRect(300, 70, 71, 21),QRect(300, 120, 71, 21),QRect(300, 170, 71, 21),QRect(300, 220, 71, 21),
                                            QRect(300, 270, 71, 21),QRect(300, 320, 71, 21),QRect(300, 370, 71, 21),QRect(300, 420, 71, 21),
                                            QRect(300, 470, 71, 21),QRect(300, 520, 71, 21),QRect(300, 570, 71, 21)};

const QRect LineLiftReadHandYNowRect[11] = {QRect(400, 70, 71, 21),QRect(400, 120, 71, 21),QRect(400, 170, 71, 21),QRect(400, 220, 71, 21),
                                            QRect(400, 270, 71, 21),QRect(400, 320, 71, 21),QRect(400, 370, 71, 21),QRect(400, 420, 71, 21),
                                            QRect(400, 470, 71, 21),QRect(400, 520, 71, 21),QRect(400, 570, 71, 21)};

const QRect LineLiftReadHandZNowRect[11] = {QRect(500, 70, 71, 21),QRect(500, 120, 71, 21),QRect(500, 170, 71, 21),QRect(500, 220, 71, 21),
                                            QRect(500, 270, 71, 21),QRect(500, 320, 71, 21),QRect(500, 370, 71, 21),QRect(500, 420, 71, 21),
                                            QRect(500, 470, 71, 21),QRect(500, 520, 71, 21),QRect(500, 570, 71, 21)};

const QRect LineLiftReadHandWNowRect[11] = {QRect(600, 70, 71, 21),QRect(600, 120, 71, 21),QRect(600, 170, 71, 21),QRect(600, 220, 71, 21),
                                            QRect(600, 270, 71, 21),QRect(600, 320, 71, 21),QRect(600, 370, 71, 21),QRect(600, 420, 71, 21),
                                            QRect(600, 470, 71, 21),QRect(600, 520, 71, 21),QRect(600, 570, 71, 21)};

const QRect LiftAllCheckBoxRect[11] = {QRect(1060, 500, 71, 19),QRect(1130, 500, 71, 19),
                                       QRect(1060, 550, 71, 19),QRect(1130, 550, 71, 19),
                                       QRect(1060, 600, 71, 19),QRect(1130, 600, 71, 19),
                                       QRect(1060, 650, 71, 19),QRect(1130, 650, 71, 19),
                                       QRect(1560, 100, 71, 19),QRect(1640, 100, 71, 19),
                                       QRect(1720, 100, 71, 19)};


const QRect RunAllCheckBoxRect[11] = {QRect(1300, 500, 71, 19),QRect(1380, 500, 71, 19),QRect(1460, 500, 71, 19),QRect(1540, 500, 71, 19),
                                       QRect(1300, 550, 71, 19),QRect(1380, 550, 71, 19),QRect(1460, 550, 71, 19),QRect(1540, 550, 71, 19),
                                      };



const QRect LabelRunRect[11] = {QRect(20, 70, 41, 21),QRect(20, 120, 41, 21),QRect(20, 170, 41, 21),QRect(20, 220, 41, 21),
                                QRect(20, 270, 41, 21),QRect(20, 320, 41, 21),QRect(20, 370, 41, 21),QRect(20, 420, 41, 21),
                                QRect(20, 470, 41, 21),QRect(20, 520, 51, 21),QRect(20, 570, 51, 21)};


const QRect LineRunPosXRect[11] = {QRect(1120, 70, 71, 21),QRect(1120, 120, 71, 21),QRect(1120, 170, 71, 21),QRect(1120, 220, 71, 21),
                                   QRect(1120, 270, 71, 21),QRect(1120, 320, 71, 21),QRect(1120, 370, 71, 21),QRect(1120, 420, 71, 21),
                                   QRect(1120, 470, 71, 21),QRect(1120, 520, 71, 21),QRect(1120, 570, 71, 21)};
const QRect LineRunPosYRect[11] = {QRect(1200, 70, 71, 21),QRect(1200, 120, 71, 21),QRect(1200, 170, 71, 21),QRect(1200, 220, 71, 21),
                                   QRect(1200, 270, 71, 21),QRect(1200,320, 71, 21),QRect(1200, 370, 71, 21),QRect(1200, 420, 71, 21),
                                   QRect(1200, 470, 71, 21),QRect(1200, 520, 71, 21),QRect(1200, 570, 71, 21)};
const QRect LineRunPosZRect[11] = {QRect(1280, 70, 71, 21),QRect(1280, 120, 71, 21),QRect(1280, 170, 71, 21),QRect(1280, 220, 71, 21),
                                   QRect(1280, 270, 71, 21),QRect(1280, 320, 71, 21),QRect(1280, 370, 71, 21),QRect(1280, 420, 71, 21),
                                   QRect(1280, 470, 71, 21),QRect(1280, 520, 71, 21),QRect(1280, 570, 71, 21)};

const QRect LineRunErrorAngleXRect[11] = {QRect(1360, 70, 71, 21),QRect(1360, 120, 71, 21),QRect(1360, 170, 71, 21),QRect(1360, 220, 71, 21),
                                          QRect(1360, 270, 71, 21),QRect(1360,320, 71, 21),QRect(1360, 370, 71, 21),QRect(1360, 420, 71, 21),
                                          QRect(1360, 470, 71, 21),QRect(1360, 520, 71, 21),QRect(1360, 570, 71, 21)};
const QRect LineRunErrorAngleYRect[11] = {QRect(1440, 70, 71, 21),QRect(1440, 120, 71, 21),QRect(1440, 170, 71, 21),QRect(1440, 220, 71, 21),
                                          QRect(1440, 270, 71, 21),QRect(1440, 320, 71, 21),QRect(1440, 370, 71, 21),QRect(1440, 420, 71, 21),
                                          QRect(1440, 470, 71, 21),QRect(1440, 520, 71, 21),QRect(1440, 570, 71, 21)};
const QRect LineRunErrorPullRect[11] = {QRect(1520, 70, 71, 21),QRect(1520, 120, 71, 21),QRect(1520, 170, 71, 21),QRect(1520,220, 71, 21),
                                        QRect(1520, 270, 71, 21),QRect(1520, 320, 71, 21),QRect(1520, 370, 71, 21),QRect(1520, 420, 71, 21),
                                        QRect(1520, 470, 71, 21),QRect(1520, 520, 71, 21),QRect(1520, 570, 71, 21)};

const QRect   switchRect[11] = {QRect(1610, 70, 50, 19),QRect(1610, 120, 50, 19),QRect(1610, 170, 50, 19),QRect(1610, 220, 50, 19),
                              QRect(1610, 270, 50, 19),QRect(1610, 320, 50, 19),QRect(1610, 370, 50, 19),QRect(1610, 420, 50, 19),
                              QRect(1610, 470, 50, 19),QRect(1610, 520, 50, 19),QRect(1610, 570, 50, 19)};


const QString labelDetailStrings[10]={"磁栅尺X","磁栅尺Y","水平仪","拉力计","X速度","Y速度","Z速度","W位置","编码器转角","其他"};

const QRect CheckBoxDetail1Rect[9] = {QRect(380, 60, 91, 21),QRect(380, 100, 91, 21),QRect(380, 140, 91, 21),QRect(380, 180, 91, 21),
                                      QRect(380, 220, 91, 21),QRect(380, 260, 91, 21),QRect(380, 300, 91, 21),QRect(380, 340, 101, 21),QRect(380, 380, 101, 21)};



}//ened namespace BASE

/////////qt中声明自定义结构体使用信号和槽/////////////////////////////

Q_DECLARE_METATYPE(BASE::SigalMessages);


#endif // HOST_BASE_MESSAGES_H
