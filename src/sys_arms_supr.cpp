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


namespace SUPR {

//thread parame
BASE::ARMS_THREAD_INFO mArmsModule[DEF_SYS_ARMS_NUMS];

typedef struct _MODULEINFOS {
  const char* mName;
  void *(*mEntry)(void* mModule);
  int32_t mPri;
  pthread_t mPid;

} MODULEINFOS;


MODULEINFOS gHiMInfo[CONF::ARMS_M_MAX_ID];
BASE::ARMS_MSGS mArmsMsgs;


static int suprWorking = 1;

static int32_t prepareEnv(void);
static int32_t initSupr(void);
static void changeSuprThreadInfo(void);
static int32_t startModules(void);
static int32_t suprMainLoop();
static void signalHandle(int mSignal);


static int32_t deInitSupr(void);
////////////////////////////////////////////////////////////////////////////////
///////int32_ternal int32_terface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////

static int latency_target_fd = -1;
static int32_t latency_target_value = 0;

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

  startModules();

  return 0;
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
  printf("startModule");
  int32_t qIdx = CONF::ARMS_M_SUPR_ID;


//  BASE::hiGetThreadPri(pthread_self());
  for (qIdx = CONF::ARMS_M_SUPR_ID + 1; qIdx < CONF::ARMS_M_MAX_ID; qIdx++)
  {
    //mArmsModule[qIdx-1].mMsgId   = qIdx-1;
    mArmsModule[qIdx-1].mWorking = true;
    //memcpy(mArmsModule[qIdx-1].mIpV4Str, CONF::MN_SERVER_IP[qIdx-1], sizeof(CONF::MN_SERVER_IP[qIdx-1]));
    //test
    memcpy(mArmsModule[qIdx-1].mIpV4Str, CONF::MN_SERVER_IP[qIdx-1], sizeof(CONF::MN_SERVER_IP[qIdx-1]));
    mArmsModule[qIdx-1].mSerPort = CONF::MN_SERVER_PORT[qIdx-1];
    mArmsModule[qIdx-1].mState = BASE::M_STATE_INIT;
  }


  for (qIdx = CONF::ARMS_M_SUPR_ID + 1; qIdx < CONF::ARMS_M_MAX_ID; qIdx++)
  {
    gHiMInfo[qIdx].mPid   = BASE::hiCreateThread(CONF::MN_NAME[qIdx],
                                                 LEADER::threadEntry,
                                                 CONF::PRI_LEAD,
                                                 &mArmsModule[qIdx-1]);
    //printf("pid:%u\t", gHiMInfo[qIdx].mPid);
  }
  return 0;
}


static int32_t suprMainLoop(){
  //TODU
  struct timespec now, next, interval;
  unsigned int nDelay = 990;        /* usec */

  int  ret;
  int64_t diff, max = 0, printfIndex = 0;

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

    for (int qIdx = 0; qIdx < DEF_SYS_ARMS_NUMS; qIdx++)
    {
      clock_gettime(CLOCK_MONOTONIC, & mArmsModule[qIdx].startTime);
      pthread_cond_signal(&mArmsModule[qIdx].mArmsMsgReady);
    }

      /*
    if ((ret = clock_gettime(CLOCK_MONOTONIC, &now))) {
        if (ret != EINTR)
            printf("clock_getttime() failed. errno: %d\n", errno);
        continue;
    }

    diff = calcdiff_ns(now, next);
    if(diff > max)
      max = diff;
    */
    //timeout notice cond

  }
}

static void signalHandle(int mSignal)
{
  suprWorking = 0;

  for (int32_t qIdx = 0; qIdx < DEF_SYS_ARMS_NUMS; qIdx++)
  {
    mArmsModule[qIdx].mWorking = false;
    // cond
    pthread_cond_signal(&mArmsModule[qIdx].mArmsMsgReady);
  }

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
    iRet = suprMainLoop();
  }

  printf("quit dms partial modules for begin re-program ");
  int32_t qIdx = CONF::ARMS_M_SUPR_ID;
  for (qIdx = CONF::ARMS_M_SUPR_ID+1; qIdx < CONF::ARMS_M_MAX_ID; qIdx++) {
    printf("live until re-program qIdx=%d\n", qIdx);
    pthread_join(gHiMInfo[qIdx].mPid, NULL);
  }

  deInitSupr();

  printf("quit sysArms all modules");
  return iRet;
}


} //namespace
