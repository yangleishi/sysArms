/******************************************************************************
**
* Copyright (c)2021 SHI YANGLEI
* All Rights Reserved
*
*
******************************************************************************/
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

//thread parame
BASE::ARMS_THREAD_INFO mArmsModule[DEF_SYS_ARMS_NUMS];
BASE::ARMS_THREAD_INFO mArmsTension[DEF_SYS_TENSIONLEADER_NUMS];
BASE::LOG_THREAD_INFO  mlogsModule;
BASE::INTERACTION_THREAD_INFO mManInteraction;

static BASE::STR_QUEUE *mLogQueue = NULL;


typedef struct _MODULEINFOS {
  const char* mName;
  void *(*mEntry)(void* mModule);
  int32_t mPri;
  pthread_t mPid;

} MODULEINFOS;

//arms,tensions,logs threads and supr
MODULEINFOS gHiMInfo[MODULES_NUMS];
BASE::ARMS_MSGS mArmsMsgs;

static int suprWorking = 1;

static int32_t prepareEnv(void);
static int32_t initSupr(void);
static void changeSuprThreadInfo(void);
static int32_t startModules(void);
static int32_t suprMainLoop();
static void signalHandle(int mSignal);
static BASE::STR_QUEUE * qCreate(void);
static void checkSysError();
static void changeState();

static int32_t deInitSupr(void);
////////////////////////////////////////////////////////////////////////////////
///////int32_ternal int32_terface //////////////////////////////////////////////////////
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
static void set_latency_target(void)
{
    struct stat s;
    int err;

    errno = 0;
    err = stat("/dev/cpu_dma_latency", &s);
    if (err == -1) {
        printf("WARN: stat /dev/cpu_dma_latency failed");
        return;
    }

    errno = 0;
    latency_target_fd = open("/dev/cpu_dma_latency", O_RDWR);
    if (latency_target_fd == -1) {
        printf("WARN: open /dev/cpu_dma_latency");
        return;
    }

    errno = 0;
    err = write(latency_target_fd, &latency_target_value, 4);
    if (err < 1) {
        printf("# error setting cpu_dma_latency to %d!", latency_target_value);
        close(latency_target_fd);
        return;
    }
    printf("# /dev/cpu_dma_latency set to %dus\n", latency_target_value);
}

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

static inline void tsnorm(struct timespec *ts)
{
    while (ts->tv_nsec >= NSEC_PER_SEC) {
        ts->tv_nsec -= NSEC_PER_SEC;
        ts->tv_sec++;
    }
}

static inline int64_t calcdiff_ns(struct timespec t1, struct timespec t2)
{
    int64_t diff;
    diff = NSEC_PER_SEC * (int64_t)((int) t1.tv_sec - (int) t2.tv_sec);
    diff += ((int) t1.tv_nsec - (int) t2.tv_nsec);
    return diff;
}



static int32_t prepareEnv(void) {
  int32_t iRet = 0;
  printf("ARMS APP STARTING");

  ///TODO:: msg and msg q need refactor after first version.

  initSupr();

  changeSuprThreadInfo();

  iRet = startModules();
  return iRet;
}

static int32_t initSupr(void) {
  //need handler error case
  int32_t iRet = 0;
  /* 互斥 con初始化. */
  for (int qIdx = 0; qIdx < DEF_SYS_ARMS_NUMS; qIdx++)
  {
    pthread_mutex_init(&mArmsModule[qIdx].mArmsMsgMutex, NULL);
    pthread_cond_init(&mArmsModule[qIdx].mArmsMsgReady, NULL);
  }

  //queue cond
  pthread_cond_init(&mlogsModule.mPrintQueueReady, NULL);
  mLogQueue = qCreate();

  signal(SIGINT, signalHandle);

  ///////cpu latency set 0us
  set_latency_target();
  return iRet;
}

static void changeSuprThreadInfo(void) {
  BASE::hiSetThreadsched(pthread_self(), CONF::PRI_SUPR);
}



static int32_t startModules(void) {
  //need handler error case
  int32_t qIdx = CONF::ARMS_M_SUPR_ID;

  // logs thread
  mlogsModule.mWorking = true;
  mlogsModule.mLogQueue = mLogQueue;
  strcpy(mlogsModule.mThreadName, CONF::MN_LOG_NAME);
  gHiMInfo[CONF::ARMS_LOG_1_ID].mPid         = BASE::hiCreateThread(
                                                                     CONF::MN_LOG_NAME,
                                                                     LOGER::threadEntry,
                                                                     CONF::PRI_LEAD,
                                                                     &mlogsModule);
  // interactioner thread
  mManInteraction.mWorking = true;
  mManInteraction.mLogQueue = mLogQueue;
  strcpy(mManInteraction.mThreadName, CONF::MN_INTERACTION_NAME);
  memcpy(mManInteraction.mIpV4Str, CONF::MN_INTERACTION_SERVER_IP, sizeof(CONF::MN_INTERACTION_SERVER_IP));
  mManInteraction.mSerPort = CONF::MN_INTERACTION_SERVER_PORT;
  mManInteraction.mState = BASE::M_STATE_INIT;
  gHiMInfo[CONF::ARMS_INTERACTION_ID].mPid         =  BASE::hiCreateThread(
                                                                     CONF::MN_INTERACTION_NAME,
                                                                     INTERACTIONER::threadEntry,
                                                                     CONF::PRI_LEAD,
                                                                     &mManInteraction);


  // arms threads
  for (qIdx = CONF::ARMS_M_SUPR_ID + 1; qIdx < CONF::ARMS_M_MAX_ID; qIdx++)
  {
    mArmsModule[qIdx-1].mWorking = true;
    mArmsModule[qIdx-1].mLogQueue = mLogQueue;
    strcpy(mArmsModule[qIdx-1].mThreadName, CONF::MN_NAME[qIdx]);
    memcpy(mArmsModule[qIdx-1].mIpV4Str, CONF::MN_SERVER_IP[qIdx-1], sizeof(CONF::MN_SERVER_IP[qIdx-1]));
    mArmsModule[qIdx-1].mSerPort = CONF::MN_SERVER_PORT[qIdx-1];
    mArmsModule[qIdx-1].mState = BASE::M_STATE_INIT;
  }


  for (qIdx = CONF::ARMS_M_SUPR_ID + 1; qIdx < CONF::ARMS_M_MAX_ID; qIdx++)
  {
    gHiMInfo[qIdx].mPid   = BASE::hiCreateThread(CONF::MN_NAME[qIdx],
                                                 LEADER::threadEntry,
                                                 CONF::PRI_LEAD,
                                                 &(mArmsModule[qIdx-1]));

    //printf("pid:%u\t", gHiMInfo[qIdx].mPid);
  }



// tension threads
  for (qIdx = 0; qIdx < CONF::ARMS_T_MAX_ID-CONF::ARMS_T_1_ID; qIdx++)
  {
    mArmsTension[qIdx].mWorking  = true;
    mArmsTension[qIdx].mLogQueue = mLogQueue;
    strcpy(mArmsTension[qIdx].mThreadName, CONF::MN_TENSION_NAME[qIdx]);
    memcpy(mArmsTension[qIdx].mIpV4Str, CONF::MN_TENSION_SERVER_IP[qIdx], sizeof(CONF::MN_TENSION_SERVER_IP[qIdx]));
    mArmsTension[qIdx].mSerPort = CONF::MN_TENSION_SERVER_PORT[qIdx];
  }


  for (qIdx = CONF::ARMS_T_1_ID; qIdx < CONF::ARMS_T_MAX_ID; qIdx++)
  {
    gHiMInfo[qIdx].mPid   = BASE::hiCreateThread(CONF::MN_TENSION_NAME[qIdx-CONF::ARMS_T_1_ID],
                                                 TENSIONLEADER::threadEntry,
                                                 CONF::PRI_LEAD,
                                                 &mArmsTension[qIdx-CONF::ARMS_T_1_ID]);
    //printf("pid:%u\t", gHiMInfo[qIdx].mPid);
  }

  return 0;
}


static void checkSysError()
{
  int32_t iRet = 0;

  for (int qIdx = 0; qIdx < DEF_SYS_ARMS_NUMS; qIdx++)
  {
    if(!mArmsModule[qIdx].mWorking)
      iRet++;
  }

  //no arm stop
  if(iRet == 0)
    return;

  //all stop
  if(iRet == DEF_SYS_ARMS_NUMS)
  {
    suprWorking = 0;

    //stop tensions
    for (int qIdx = 0; qIdx < DEF_SYS_TENSIONLEADER_NUMS; qIdx++)
      mArmsTension[qIdx].mWorking = false;

    //stop interaction
    mManInteraction.mWorking = false;
    return;
  }


  if(iRet != 0)
  {
    for (int qIdx = 0; qIdx < DEF_SYS_ARMS_NUMS; qIdx++)
      if(mArmsModule[qIdx].mWorking)
          mArmsModule[qIdx].mState = BASE::M_STATE_STOP;
  }

}

static void changeState()
{
  switch (mSysState)
  {
    case BASE::M_STATE_INIT:
    {
      int iRet = 0;
      //check is conf
      for (int qIdx = 0; qIdx < DEF_SYS_ARMS_NUMS; qIdx++)
        if(mArmsModule[qIdx].mAckState == BASE::ACK_STATE_INIT_OK)
          iRet++;
      //change all modules CONF state
      if(iRet == DEF_SYS_ARMS_NUMS)
      {
        for (int qIdx = 0; qIdx < DEF_SYS_ARMS_NUMS; qIdx++)
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
        for (int qIdx = 0; qIdx < DEF_SYS_ARMS_NUMS; qIdx++)
          mArmsModule[qIdx].mState = mManInteraction.mNewState;

        mSysState = mManInteraction.mNewState;
        mManInteraction.mIsStataChange = false;
      }
      break;
    }
    case BASE::M_STATE_RUN:
    {
      //check is change to stop
      if(mManInteraction.mIsStataChange && mManInteraction.mNewState == BASE::M_STATE_STOP)
      {
        for (int qIdx = 0; qIdx < DEF_SYS_ARMS_NUMS; qIdx++)
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
        for (int qIdx = 0; qIdx < DEF_SYS_ARMS_NUMS; qIdx++)
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

static int32_t suprMainLoop(){
  //TODU
  struct timespec now, next, interval;
  unsigned int nDelay = 10000;        /* usec */

  int  ret;

  while(suprWorking)
  {
    /* Retrieve current value of CLOCK_REALTIME clock */
    interval.tv_sec  = nDelay / USEC_PER_SEC;
    interval.tv_nsec = (nDelay % USEC_PER_SEC) * 1000;
    if (clock_gettime(CLOCK_MONOTONIC, &now) == -1)
        continue;

    next = now;
    next.tv_sec  += interval.tv_sec;
    next.tv_nsec += interval.tv_nsec;
    tsnorm(&next);

    if ((ret = clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next, NULL))) {
        if (ret != EINTR)
            printf("clock_nanosleep failed. errno: %d\n", errno);
        continue;
     }

    // if one arm error then set other stop, or man-interaction error set stop
    checkSysError();

    changeState();

    for (int qIdx = 0; qIdx < DEF_SYS_ARMS_NUMS; qIdx++)
    {
      pthread_cond_signal(&mArmsModule[qIdx].mArmsMsgReady);
    }

    //notice loger cycle log
    pthread_cond_signal(&mlogsModule.mPrintQueueReady);

  }
}

static void signalHandle(int mSignal)
{
  mSysState = BASE::M_STATE_STOP;
  for (int32_t qIdx = 0; qIdx < DEF_SYS_ARMS_NUMS; qIdx++)
  {
    //mArmsModule[qIdx].mWorking = false;
    // cond
    mArmsModule[qIdx].mState = BASE::M_STATE_STOP;
    pthread_cond_signal(&mArmsModule[qIdx].mArmsMsgReady);
  }
  //suprWorking = 0;
  printf("oops! stop!!!\n");
}

static int32_t deInitSupr(void)
{
  int32_t iRet = 0;

  for (int qIdx = 0; qIdx < DEF_SYS_ARMS_NUMS; qIdx++)
  {
    pthread_mutex_destroy(&mArmsModule[qIdx].mArmsMsgMutex);
    pthread_cond_destroy(&mArmsModule[qIdx].mArmsMsgReady);
    //printf("pid:%u\t", gHiMInfo[qIdx].mPid);
  }

  pthread_cond_destroy(&mlogsModule.mPrintQueueReady);
  if(mLogQueue != NULL)
  {
    free(mLogQueue);
    mLogQueue = NULL;
  }

  /* close the latency_target_fd if it's open */
  if (latency_target_fd >= 0)
      close(latency_target_fd);

  return iRet;
}
////////////////////////////////////////////////////////////////////////////////
///////external int32_terface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void clientTest()
{


}

int32_t dmsAppStartUp() {
  int32_t iRet = 0;

  iRet = prepareEnv();
  if (0 == iRet) {
    printf("all modules runing now! ctrl+c to endup\n");
    iRet = suprMainLoop();
  }

  int32_t qIdx = CONF::ARMS_M_SUPR_ID;
  for (qIdx = CONF::ARMS_M_SUPR_ID+1; qIdx < CONF::ARMS_INTERACTION_MAX_ID; qIdx++) {
    LOGER::PrintfLog("live until re-program qIdx=%d", qIdx);
    pthread_join(gHiMInfo[qIdx].mPid, NULL);
  }


  //notice loger to endup
  LOGER::PrintfLog("quit loger modules");
  mlogsModule.mWorking = false;
  pthread_cond_signal(&mlogsModule.mPrintQueueReady);
  pthread_join(gHiMInfo[MODULES_NUMS-1].mPid, NULL);

  deInitSupr();

  printf("quit sysArms all modules\n");
  return iRet;
}


} //namespace
