/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_supr.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:系统中的超级线程模块，此模块负责创建所有线程，管理其他模块，周期的发送
消息通知其他模块。
*
********************************************************************************/
#include <time.h>
#include <signal.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <sys/stat.h>
#include <fcntl.h>

#include "sys_arms_supr.hpp"
#include "sys_arms_conf.hpp"
#include "sys_arms_daemon.hpp"
#include "sys_arms_leader.hpp"
#include "sys_arms_loger.hpp"
#include "sys_arms_tension_leader.hpp"
#include "sys_arms_interactioner.hpp"


namespace SUPR {

//模块结构体，线程名字，入口函数，pid
typedef struct _MODULEINFOS
{
  const char* mName;
  void *(*mEntry)(void* mModule);
  int32_t mPri;
  pthread_t mPid;
} MODULEINFOS;

//thread parame
BASE::ARMS_THREAD_INFO mArmsModule[DEF_SYS_USE_ARMS_NUMS];
BASE::TENSIONS_THREAD_INFO mArmsTension[DEF_SYS_MAX_TENSIONLEADER_NUMS];
BASE::LOG_THREAD_INFO  mlogsModule;


BASE::INTERACTION_THREAD_INFO mManInteraction;
//interaction to supr
BASE::InteractionDataToSupr  mInteractionDatas;
BASE::SuprDataToInteraction  mSuprDataToInt;


//tensions value
BASE::TENSIONS_NOW_DATA mTensionsData[DEF_SYS_USE_ARMS_NUMS];

//if arms is working then the bit is 0.else 1
//pthread_mutex_t mArmsWorkingMutex;

pthread_mutex_t mPrintQueueMutex;
//log queue all module will use this queue
static BASE::STR_QUEUE *mLogQueue = NULL;

//arms data log queue, only supr use this queue
static BASE::STR_QUEUE *mArmsDataLogQueue = NULL;


//arms,tensions,logs threads and supr
MODULEINFOS gHiMInfo[MODULES_NUMS];


static int suprWorking = 1;

static int32_t prepareEnv(void);
static int32_t initSupr(void);
static void changeSuprThreadInfo(void);
static int32_t startModules(void);
static int32_t suprMainLoop();
static void signalHandle(int mSignal);
static BASE::STR_QUEUE * qCreate(void);
static void checkArmsWorking();
static void handleArmsCrossing();
static void handleInteractionCmd();
static void changeState();

static int32_t deInitSupr(void);
////////////////////////////////////////////////////////////////////////////////
///////internal interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static int latency_target_fd = -1;
static int32_t latency_target_value = 0;

static BASE::M_STATE mSysState = BASE::M_STATE_INIT;

/* Latency trick
 * if the file /dev/cpu_dma_latency exists,
 * open it and write a zero into it. This will tell
 * the power management system not to transition to
 * a high cstate (in fact, the system acts like idle=poll)
 * When the fd to /dev/cpu_dma_latency is closed, the behavior
 * goes back to the system default.
 *
 * Documentation/power/pm_qos_interface.txt
 */
/******************************************************************************
* 功能：设置电源管理策略，此模块比避免cpu进入节能模式，影响定时器精度
* @return Descriptions
******************************************************************************/
static void set_latency_target(void)
{
    struct stat s;
    int err;

    errno = 0;
    err = stat("/dev/cpu_dma_latency", &s);
    if (err == -1) {
        LOGER::PrintfLog(BASE::S_APP_LOGER, "WARN: stat /dev/cpu_dma_latency failed");
        return;
    }

    errno = 0;
    latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
    if (latency_target_fd == -1) {
        LOGER::PrintfLog(BASE::S_APP_LOGER, "WARN: open /dev/cpu_dma_latency");
        return;
    }

    errno = 0;
    err = write(latency_target_fd, &latency_target_value, 4);
    if (err < 1) {
        LOGER::PrintfLog(BASE::S_APP_LOGER, "# error setting cpu_dma_latency to %d!", latency_target_value);
        close(latency_target_fd);
        return;
    }
    LOGER::PrintfLog(BASE::S_APP_LOGER, "# /dev/cpu_dma_latency set to %dus", latency_target_value);
}

/******************************************************************************
* 功能：创建循环队列
* @return Descriptions
******************************************************************************/
static BASE::STR_QUEUE * qCreate(void)
{
  BASE::STR_QUEUE *q = (BASE::STR_QUEUE*)malloc(sizeof(BASE::STR_QUEUE));	// 分配一个队列空间
  if(NULL == q)	// 分配失败
    return NULL;

  // 分配成功
  q->mRear  = 0;
  q->mFront = 0;
  return q;
}

/******************************************************************************
* 功能：tsnorm函数将timespec中的纳秒，妙格式化。按照1m = 1000000000ns，
* @param ts : ts是时间结构体，包含：妙 纳秒
* @return Descriptions
******************************************************************************/
static inline void tsnorm(struct timespec *ts)
{
    while (ts->tv_nsec >= NSEC_PER_SEC) {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}

/******************************************************************************
* 功能：计算两个时间差
* @param t1 : t1是时间结构体，包含：妙 纳秒
* @param t2 : t2是时间结构体，包含：妙 纳秒
* @return Descriptions
******************************************************************************/
static inline int64_t calcdiff_ns(struct timespec t1, struct timespec t2)
{
    int64_t diff;
    diff = NSEC_PER_SEC * (int64_t)((int) t1.tv_sec - (int) t2.tv_sec);
    diff += ((int) t1.tv_nsec - (int) t2.tv_nsec);
    return diff;
}

/******************************************************************************
* 功能：初始化整个系统的运行环境
* @return Descriptions
******************************************************************************/
static int32_t prepareEnv(void) {
  int32_t iRet = 0;
  printf("ARMS APP STARTING\n");

  //TODO:: msg and msg q need refactor after first version.

  initSupr();

  changeSuprThreadInfo();

  iRet = startModules();
  return iRet;
}

/******************************************************************************
* 功能：初始化supr变量
* @return Descriptions
******************************************************************************/
static int32_t initSupr(void) {
  //need handler error case
  int32_t iRet = 0;
  /* 互斥 con初始化. */
  for (int qIdx = 0; qIdx < DEF_SYS_USE_ARMS_NUMS; qIdx++)
  {
    pthread_mutex_init(&mArmsModule[qIdx].mArmsMsgMutex, NULL);
    pthread_cond_init(&mArmsModule[qIdx].mArmsMsgReady, NULL);

    pthread_mutex_init(&mArmsModule[qIdx].mMotorMutex, NULL);
  }

  //queue cond
  pthread_mutex_init(&mPrintQueueMutex, NULL);
  pthread_cond_init(&mlogsModule.mPrintQueueReady, NULL);
  mLogQueue = qCreate();
  mArmsDataLogQueue = qCreate();

  //interaction
  memset((char*)&mInteractionDatas, 0, sizeof(mInteractionDatas));
  pthread_mutex_init(&mInteractionDatas.mInteractionRecMutex, NULL);
  pthread_mutex_init(&mInteractionDatas.mInteractionSendMutex, NULL);

  //arms now datas mutex,this datas to interaction
  pthread_mutex_init(&mSuprDataToInt.mArmsNowDatasMutex, NULL);

  signal(SIGINT, signalHandle);

  ///////cpu latency set 0us
  set_latency_target();

  //supr run in cpu0
  BASE::hiSetCpuAffinity(CONF::CPU_SUPR);
  return iRet;
}

/******************************************************************************
* 功能：修改supr线程优先级
* @return Descriptions
******************************************************************************/
static void changeSuprThreadInfo(void) {
  BASE::hiSetThreadsched(pthread_self(), CONF::PRI_SUPR);
}

/******************************************************************************
* 功能：各个线程模块初始化，并且启动个各模块线程
* @return Descriptions
******************************************************************************/
static int32_t startModules(void) {
  //need handler error case
  int32_t qIdx = CONF::ARMS_M_SUPR_ID;

  // logs thread
  mlogsModule.mWorking = true;
  mlogsModule.mLogQueue = mLogQueue;
  mlogsModule.mArmsDataQueue = mArmsDataLogQueue;
  mlogsModule.mCpuAffinity  = CONF::CPU_LOGER;
  strcpy(mlogsModule.mThreadName, CONF::MN_LOG_NAME);
  gHiMInfo[CONF::ARMS_LOG_1_ID].mPid         = BASE::hiCreateThread(
                                                                     CONF::MN_LOG_NAME,
                                                                     LOGER::threadEntry,
                                                                     CONF::PRI_LOGER,
                                                                     &mlogsModule);
  // interactioner thread
  memset((char*)&mSuprDataToInt, 0, sizeof(mSuprDataToInt));
  mManInteraction.mWorking = true;
  mManInteraction.mLogQueue = mLogQueue;
  strcpy(mManInteraction.mThreadName, CONF::MN_INTERACTION_NAME);
  memcpy(mManInteraction.mMyIpV4Str, CONF::MN_INTERACTION_SERVER_IP, sizeof(CONF::MN_INTERACTION_SERVER_IP));
  mManInteraction.mMyPort = CONF::MN_INTERACTION_SERVER_PORT;
  mManInteraction.mPrintQueueMutex = &mPrintQueueMutex;
  mManInteraction.mState = BASE::M_STATE_INIT;
  mManInteraction.mCpuAffinity  = CONF::CPU_INTERACTIONER;
  mManInteraction.mInterToSuprDatas = &mInteractionDatas;
  mManInteraction.mSuprDatasToInterasction  = &mSuprDataToInt;
  gHiMInfo[CONF::ARMS_INTERACTION_ID].mPid    =  BASE::hiCreateThread(
                                                                     CONF::MN_INTERACTION_NAME,
                                                                     INTERACTIONER::threadEntry,
                                                                     CONF::PRI_LEAD,
                                                                     &mManInteraction);

  usleep(100000);
  // arms threads
  for (qIdx = CONF::ARMS_M_SUPR_ID + 1; qIdx < DEF_SYS_USE_ARMS_NUMS+1; qIdx++)
  {
    mArmsModule[qIdx-1].mWorking = true;
    mArmsModule[qIdx-1].mLogQueue = mLogQueue;
    strcpy(mArmsModule[qIdx-1].mThreadName, CONF::MN_NAME[qIdx]);
    memcpy(mArmsModule[qIdx-1].mMyIpV4Str, CONF::MN_SERVER_IP[qIdx-1], sizeof(CONF::MN_SERVER_IP[qIdx-1]));
    memcpy(mArmsModule[qIdx-1].mPeerIpV4Str, CONF::MN_PEER_IP[qIdx-1], sizeof(CONF::MN_PEER_IP[qIdx-1]));
    mArmsModule[qIdx-1].mMyPort = CONF::MN_SERVER_PORT[qIdx-1];
    mArmsModule[qIdx-1].mPeerPort = CONF::MN_PEER_PORT[qIdx-1];
    mArmsModule[qIdx-1].mState = BASE::M_STATE_INIT;
    mArmsModule[qIdx-1].mAckState = BASE::ACK_STATE_NULL;
    mArmsModule[qIdx-1].mPrintQueueMutex = &mPrintQueueMutex;
    mArmsModule[qIdx-1].mMsgId = qIdx - 1 + BASE::ID_LEADER_FIR;
    mArmsModule[qIdx-1].mNewRecMsg = false;
    mArmsModule[qIdx-1].mNowTension  = &mTensionsData[qIdx-1];
    mArmsModule[qIdx-1].mCpuAffinity  = CONF::CPU_LEAD;
    mArmsModule[qIdx-1].mIsNowMotorCmd = 0;
    gHiMInfo[qIdx].mPid   = BASE::hiCreateThread(CONF::MN_NAME[qIdx],
                                                 LEADER::threadEntry,
                                                 CONF::PRI_LEAD,
                                                 &(mArmsModule[qIdx-1]));
    usleep(10000);
  }

// tension threads
  for (qIdx = 0; qIdx < CONF::ARMS_T_MAX_ID-CONF::ARMS_T_1_ID; qIdx++)
  {
    mArmsTension[qIdx].mWorking  = true;
    mArmsTension[qIdx].mLogQueue = mLogQueue;
    strcpy(mArmsTension[qIdx].mThreadName, CONF::MN_TENSION_NAME[qIdx]);
    memcpy(mArmsTension[qIdx].mMyIpV4Str, CONF::MN_TENSION_SERVER_IP[qIdx], sizeof(CONF::MN_TENSION_SERVER_IP[qIdx]));
    mArmsTension[qIdx].mMyPort = CONF::MN_TENSION_SERVER_PORT[qIdx];
    mArmsTension[qIdx].mNowTension  = mTensionsData;
    mArmsTension[qIdx].mPrintQueueMutex = &mPrintQueueMutex;
    mArmsTension[qIdx].mCpuAffinity  = CONF::CPU_TENSION;
  }
  for (qIdx = CONF::ARMS_T_1_ID; qIdx < CONF::ARMS_T_MAX_ID; qIdx++)
  {
    gHiMInfo[qIdx].mPid   = BASE::hiCreateThread(CONF::MN_TENSION_NAME[qIdx-CONF::ARMS_T_1_ID],
                                                 TENSIONLEADER::threadEntry,
                                                 CONF::PRI_LEAD,
                                                 &mArmsTension[qIdx-CONF::ARMS_T_1_ID]);
    //LOGER::PrintfLog("pid:%u\t", gHiMInfo[qIdx].mPid);
  }

  return 0;
}

/******************************************************************************
* 功能：检查leader线程模块是否在运行
* @return Descriptions
******************************************************************************/
static void checkArmsWorking()
{
  uint16_t mArmsWorkingBits = 0;
  for (int i=0; i<DEF_SYS_USE_ARMS_NUMS; i++)
  {
    if(!mArmsModule[i].mWorking)
        mArmsWorkingBits++;
  }
  //no arm stop
  if(mArmsWorkingBits == 0)
    return;

  if(mArmsWorkingBits == DEF_SYS_USE_ARMS_NUMS) //all stop
  {
    suprWorking = 0;
    //stop tensions
    for (int qIdx = 0; qIdx < DEF_SYS_USE_TENSIONLEADER_NUMS; qIdx++)
      mArmsTension[qIdx].mWorking = false;
    //stop interaction
    mManInteraction.mWorking = false;
  }
  else //some stop
  {
    for (int qIdx = 0; qIdx < DEF_SYS_USE_ARMS_NUMS; qIdx++)
      if(mArmsModule[qIdx].mWorking)
          mArmsModule[qIdx].mState = BASE::M_STATE_STOP;
    printf("this stop\n");
  }
  /*
  for (int i=0; i<DEF_SYS_USE_ARMS_NUMS; i++)
  {
    if(!mArmsModule[i].mWorking)
        mArmsWorkingBits |= ((0x0001)<<(i));
  }
  //no arm stop
  if(mArmsWorkingBits == 0)
    return;

  if(mArmsWorkingBits == 0x3) //all stop
  {
    suprWorking = 0;
    //stop tensions
    for (int qIdx = 0; qIdx < DEF_SYS_TENSIONLEADER_NUMS; qIdx++)
      mArmsTension[qIdx].mWorking = false;
    //stop interaction
    mManInteraction.mWorking = false;
  }
  else //some stop
  {
    for (int qIdx = 0; qIdx < DEF_SYS_USE_ARMS_NUMS; qIdx++)
      if(mArmsModule[qIdx].mWorking)
          mArmsModule[qIdx].mState = BASE::M_STATE_STOP;
  }
  */

}

//TUDO *********
static void handleArmsCrossing()
{
  //check all arms rec msg
  //TODU*******  calculation and check arms are crossing

  //set all arms rec msg false.until nest msg come
  for (int i=0; i<DEF_SYS_USE_ARMS_NUMS; i++)
    mArmsModule[i].mNewRecMsg = false;
}

/******************************************************************************
* 功能：supr线程处理上位机显示线程发来的cmd命令
* @return Descriptions
******************************************************************************/
static void handleInteractionCmd()
{
  //new cmd from interaction
    if(mInteractionDatas.mIsNewRec > 0)
    {
        BASE::MArmsDownData  mmRecMsg;
        int buttonType = 0;
        pthread_mutex_lock(&mInteractionDatas.mInteractionRecMutex);
        memcpy((char*)&mmRecMsg, (char*)&mInteractionDatas.mRecMsg, sizeof(mmRecMsg));
        buttonType = mInteractionDatas.mIsNewRec;
        mInteractionDatas.mIsNewRec = 0;
        pthread_mutex_unlock(&mInteractionDatas.mInteractionRecMutex);

        printf("mIsNowMotorCmd: %d\n", buttonType);

        switch (buttonType)
        {
          case BASE::CMD_HAND_MOVE_START : case BASE::CMD_HAND_MOVE_STOP:
          {
            BASE::MoveLiftSigalData *mMove = (BASE::MoveLiftSigalData *)mmRecMsg.Datas;
            //copy move to leaders
            if(mMove->mMudoleNum >=  DEF_SYS_USE_ARMS_NUMS)
            {
                int maxNum = DEF_SYS_USE_ARMS_NUMS;
                LOGER::PrintfLog(BASE::S_APP_LOGER, "conf, move module num:%d >= max arms num:%d", mMove->mMudoleNum, maxNum);
            }
            else
            {
                pthread_mutex_lock(&mArmsModule[mMove->mMudoleNum].mMotorMutex);
                mArmsModule[mMove->mMudoleNum].mIsNowMotorCmd = buttonType;
                memcpy((char*)&mArmsModule[mMove->mMudoleNum].mMoveData, mmRecMsg.Datas, sizeof(BASE::MoveLiftSigalData));
                pthread_mutex_unlock(&mArmsModule[mMove->mMudoleNum].mMotorMutex);
            }
            break;
          }
          case BASE::CMD_ALL_MOVE_START:
          {
            BASE::MoveLiftAllData *mAllMove = (BASE::MoveLiftAllData *)mmRecMsg.Datas;
            //copy move to leaders
            for (int i=0; i<DEF_SYS_MAX_ARMS_NUMS; i++)
            {
                if(mAllMove[i].mIsValid)
                {
                    pthread_mutex_lock(&mArmsModule[i].mMotorMutex);
                    mArmsModule[i].mIsNowMotorCmd = buttonType;
                    memcpy((char*)&mArmsModule[i].mAllMoveData, (char*)&mAllMove[i], sizeof(BASE::MoveLiftAllData));
                    pthread_mutex_unlock(&mArmsModule[i].mMotorMutex);
                }
            }
            break;
          }
          case BASE::CMD_ALL_MOVE_STOP:
          case BASE::CMD_ALL_PULL_STOP:
          {
            //copy move to leaders
            for (int i=0; i<DEF_SYS_USE_ARMS_NUMS; i++)
            {
                pthread_mutex_lock(&mArmsModule[i].mMotorMutex);
                mArmsModule[i].mIsNowMotorCmd = buttonType;
                pthread_mutex_unlock(&mArmsModule[i].mMotorMutex);
            }
            break;
          }
          case BASE::CMD_ALL_PULL_START:
          {
            BASE::PullLiftAllData *mAllPull = (BASE::PullLiftAllData *)mmRecMsg.Datas;
            //copy move to leaders
            for (int i=0; i<DEF_SYS_MAX_ARMS_NUMS; i++)
            {
                if(mAllPull[i].mIsValid)
                {
                    pthread_mutex_lock(&mArmsModule[i].mMotorMutex);
                    mArmsModule[i].mIsNowMotorCmd = buttonType;
                    memcpy((char*)&mArmsModule[i].mAllPullData, (char*)&mAllPull[i], sizeof(BASE::PullLiftAllData));
                    pthread_mutex_unlock(&mArmsModule[i].mMotorMutex);
                }
            }
            break;
          }
          case BASE::CMD_RUN_START:
          case BASE::CMD_RUN_STOP:
          {
            //copy move to leaders
            for (int i=0; i<DEF_SYS_USE_ARMS_NUMS; i++)
            {
              pthread_mutex_lock(&mArmsModule[i].mMotorMutex);
              mArmsModule[i].mIsNowMotorCmd = buttonType;
              pthread_mutex_unlock(&mArmsModule[i].mMotorMutex);
            }
            break;
          }
          case BASE::CMD_QUIT:
          {
            //quit...
            mSysState = BASE::M_STATE_QUIT;
            for (int32_t qIdx = 0; qIdx < DEF_SYS_USE_ARMS_NUMS; qIdx++)
              mArmsModule[qIdx].mState = BASE::M_STATE_QUIT;
            break;
          }
          case BASE::CMD_UNLINK:
          {
            //copy move to leaders
            for (int i=0; i<DEF_SYS_USE_ARMS_NUMS; i++)
            {
              pthread_mutex_lock(&mArmsModule[i].mMotorMutex);
              mArmsModule[i].mIsNowMotorCmd = buttonType;
              pthread_mutex_unlock(&mArmsModule[i].mMotorMutex);
            }
            break;
          }
          default:
          {
            break;
          }
        }
    }

}

/******************************************************************************
* 功能：改变系统运行状态
* @return Descriptions
******************************************************************************/
static void changeState()
{
  switch (mSysState)
  {
    case BASE::M_STATE_INIT:
    {
      int iRet = 0;
      //check is conf
      for (int qIdx = 0; qIdx < DEF_SYS_USE_ARMS_NUMS; qIdx++)
        if(mArmsModule[qIdx].mAckState == BASE::ACK_STATE_INIT_OK)
          iRet++;
      //change all modules CONF state
      if(iRet == DEF_SYS_USE_ARMS_NUMS)
      {
        for (int qIdx = 0; qIdx < DEF_SYS_USE_ARMS_NUMS; qIdx++)
          mArmsModule[qIdx].mState = BASE::M_STATE_CONF;
        mSysState = BASE::M_STATE_CONF;
      }
      break;
    }
    case BASE::M_STATE_CONF:
    {
      //check is change to run or stop
      if(mManInteraction.mIsStataChange && (mManInteraction.mNewState == BASE::M_STATE_RUN || mManInteraction.mNewState == BASE::M_STATE_STOP))
      {
        for (int qIdx = 0; qIdx < DEF_SYS_USE_ARMS_NUMS; qIdx++)
          mArmsModule[qIdx].mState = mManInteraction.mNewState;

        mSysState = mManInteraction.mNewState;
        mManInteraction.mIsStataChange = false;
        LOGER::PrintfLog(BASE::S_APP_LOGER,"++++++++++++++++++++therr");
      }
      break;
    }
    case BASE::M_STATE_RUN:
    {
      //check is change to stop
      if(mManInteraction.mIsStataChange && mManInteraction.mNewState == BASE::M_STATE_STOP)
      {
        for (int qIdx = 0; qIdx < DEF_SYS_USE_ARMS_NUMS; qIdx++)
          mArmsModule[qIdx].mState = mManInteraction.mNewState;

        mSysState = mManInteraction.mNewState;
        mManInteraction.mIsStataChange = false;
      }
      break;
    }
    case BASE::M_STATE_STOP:
    {
      //check is change to conf or run
      if(mManInteraction.mIsStataChange && (mManInteraction.mNewState == BASE::M_STATE_CONF || mManInteraction.mNewState == BASE::M_STATE_RUN))
      {
        for (int qIdx = 0; qIdx < DEF_SYS_USE_ARMS_NUMS; qIdx++)
          mArmsModule[qIdx].mState = mManInteraction.mNewState;

        mSysState = mManInteraction.mNewState;
        mManInteraction.mIsStataChange = false;
      }
      break;
    }
    default:
    {
      break;
    }
  }

}

/******************************************************************************
* 功能：supr线程陷入函数，周期性的控制leader等线程运行
* @return Descriptions
******************************************************************************/
static int32_t suprMainLoop(){
  //TODU
  struct timespec now, next, interval;
  unsigned int nDelay = 4000;        /* usec */

  int  ret;

  while(suprWorking)
  {
    /* Retrieve current value of CLOCK_REALTIME clock */
    interval.tv_sec  = nDelay / USEC_PER_SEC;
    interval.tv_nsec = (nDelay % USEC_PER_SEC) * 1000;
    if (clock_gettime(CLOCK_MONOTONIC, &now) == -1)
        continue;

    //printf("ms:%d \n", (now.tv_nsec -  next.tv_nsec)/1000);

    next = now;
    next.tv_sec  += interval.tv_sec;
    next.tv_nsec += interval.tv_nsec;
    tsnorm(&next);
    if ((ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL)))
    {
      if (ret != EINTR)
        LOGER::PrintfLog(BASE::S_APP_LOGER, "clock_nanosleep failed. errno:");
      continue;
    }

    // if one arm error then set other stop, or man-interaction error set stop
    checkArmsWorking();

    //aotu change arms state,init conf run stop ...
    changeState();

    // check arms  whether  conf move control
    if(mSysState == BASE::M_STATE_CONF || mSysState == BASE::M_STATE_RUN || mSysState == BASE::M_STATE_STOP)
      handleInteractionCmd();

    // check arms  whether  crossed
    if(mSysState == BASE::M_STATE_RUN)
      handleArmsCrossing();

    //test save app arms data log
    LOGER::PrintfLog(BASE::S_ARMS_DATA, "13.6 12 10.3 1111.9 23.8 90");


    //copy now datas
    pthread_mutex_lock(&mSuprDataToInt.mArmsNowDatasMutex);
    for (int qIdx = 0; qIdx < DEF_SYS_USE_ARMS_NUMS; qIdx++)
    {
      //copy conf lift datas
      memcpy((char*)&mSuprDataToInt.mReadLiftNowDatas[qIdx], (char*)&mArmsModule[qIdx].mReadLiftSigalNowData, sizeof(BASE::ReadLiftSigalNowData));
      //copy HZ datas
      memcpy((char*)&mSuprDataToInt.mReadLiftHzDatas[qIdx], (char*)&mArmsModule[qIdx].mReadLiftHzData, sizeof(BASE::ReadLiftSigalNowData));
      //copy run datas
      memcpy((char*)&mSuprDataToInt.mReadRunDatas[qIdx], (char*)&mArmsModule[qIdx].mReadRunData, sizeof(BASE::ReadRunAllData));
    }
    pthread_mutex_unlock(&mSuprDataToInt.mArmsNowDatasMutex);

    //notice leader cycle
    for (int qIdx = 0; qIdx < DEF_SYS_USE_ARMS_NUMS; qIdx++)
    {
      pthread_cond_signal(&mArmsModule[qIdx].mArmsMsgReady);
    }
    //notice loger cycle write log
    pthread_cond_signal(&mlogsModule.mPrintQueueReady);

  }
}

/******************************************************************************
* 功能：按键中断函数，ctrl+c结束系统运行
* @return Descriptions
******************************************************************************/
static void signalHandle(int mSignal)
{
  //mSysState = BASE::M_STATE_STOP;
  mSysState = BASE::M_STATE_QUIT;
  for (int32_t qIdx = 0; qIdx < DEF_SYS_USE_ARMS_NUMS; qIdx++)
  {
    mArmsModule[qIdx].mState = BASE::M_STATE_QUIT;
  }
  printf("oops! stop!!!\n");
}

/******************************************************************************
* 功能：释放、销毁线程资源
* @return Descriptions
******************************************************************************/
static int32_t deInitSupr(void)
{
  int32_t iRet = 0;

  for (int qIdx = 0; qIdx < DEF_SYS_USE_ARMS_NUMS; qIdx++)
  {
    pthread_mutex_destroy(&mArmsModule[qIdx].mArmsMsgMutex);
    pthread_cond_destroy(&mArmsModule[qIdx].mArmsMsgReady);
    //LOGER::PrintfLog("pid:%u\t", gHiMInfo[qIdx].mPid);
  }

  pthread_cond_destroy(&mlogsModule.mPrintQueueReady);
  pthread_mutex_destroy(&mPrintQueueMutex);
  pthread_mutex_destroy(&mSuprDataToInt.mArmsNowDatasMutex);

  if(mLogQueue != NULL)
  {
    free(mLogQueue);
    mLogQueue = NULL;
  }

  if(mArmsDataLogQueue != NULL)
  {
    free(mArmsDataLogQueue);
    mArmsDataLogQueue = NULL;
  }

  /* close the latency_target_fd if it's open */
  if (latency_target_fd >= 0)
      close(latency_target_fd);

  return iRet;
}
////////////////////////////////////////////////////////////////////////////////
///////external interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void clientTest()
{
//TUDO
}

/******************************************************************************
* 功能：系统入口函数，main函数调用
* @return Descriptions
******************************************************************************/
int32_t dmsAppStartUp() {
  int32_t iRet = 0;

  iRet = prepareEnv();
  if (0 == iRet) {
    printf("all modules runing now! ctrl+c to endup\n");
    iRet = suprMainLoop();
  }

  int32_t qIdx = CONF::ARMS_M_SUPR_ID;
  for (qIdx = CONF::ARMS_M_1_ID; qIdx < DEF_SYS_USE_ARMS_NUMS+CONF::ARMS_M_1_ID; qIdx++) {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "live until re-program qIdx=%d", qIdx);
    pthread_join(gHiMInfo[qIdx].mPid, NULL);
  }

  for (qIdx = CONF::ARMS_T_1_ID; qIdx < DEF_SYS_USE_TENSIONLEADER_NUMS+CONF::ARMS_T_1_ID; qIdx++) {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "live until re-program qIdx=%d", qIdx);
    pthread_join(gHiMInfo[qIdx].mPid, NULL);
  }

  for (qIdx = CONF::ARMS_INTERACTION_ID; qIdx < CONF::ARMS_INTERACTION_MAX_ID; qIdx++) {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "live until re-program qIdx=%d", qIdx);
    pthread_join(gHiMInfo[qIdx].mPid, NULL);
  }

  //notice loger to endup
  LOGER::PrintfLog(BASE::S_APP_LOGER, "quit loger modules");
  mlogsModule.mWorking = false;
  pthread_cond_signal(&mlogsModule.mPrintQueueReady);
  pthread_join(gHiMInfo[MODULES_NUMS-1].mPid, NULL);

  deInitSupr();

  printf("quit sysArms all modules\n");
  return iRet;
}


} //namespace
