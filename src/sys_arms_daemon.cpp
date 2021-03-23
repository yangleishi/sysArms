/******************************************************************************
**
* Copyright (c)2021 SHI YANGLEI
* All Rights Reserved
*
*
******************************************************************************/

#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <pthread.h>
#include <string.h>

#include "sys_arms_daemon.hpp"

namespace BASE {

void hiCreateDaemon(const char *vDaemonName) {

  //This is daemon since ppid==1
  if (getppid() == 1) {
    exit(0);
  }

  pid_t hiPid = fork();

  if (hiPid < 0)
  {
    //TODO Something wrong
    exit(1);
  }
  if (hiPid > 0)
  {
    exit(0);
  }

  pid_t hiSid = setsid();
  if (hiSid < 0) {
    //TODO Something wrong
    exit(2);
  }

  //Redirect standard files to /dev/null
  if (freopen("/dev/null", "r", stdin) == NULL) {
    //TODO Something wrong
  }
  if (freopen("/dev/null", "w", stdout) == NULL) {
    //TODO Something wrong
  }
  if (freopen("/dev/null", "w", stderr) == NULL) {
    //TODO Something wrong
  }

  //Set umask to 022, it mean file mode mask is 0644 */
  umask(022);

  //Change current working directory
  if (chdir("/") < 0) {
    exit(3);
  }

  //Cancel certain signals */
//  signal(SIGCHLD, SIG_DFL); /* A child process dies */
//  signal(SIGTSTP, SIG_IGN); /* Various TTY signals */
//  signal(SIGTTOU, SIG_IGN);
//  signal(SIGTTIN, SIG_IGN);
//  signal(SIGTERM, SIG_DFL); /* Die on SIGTERM */
  //syslog(5, "Create Daemon OK, %s", vDaemonName);
}

void * thread_start1(void *)
{

}

pthread_t hiCreateThread(const char *cThreadName,
                         void *(*thread_start)(void *),
                         int32_t iPriority,
                         void* pModule) {

  pthread_t pthreadId;
  int32_t iRet;
  pthread_attr_t attr;
  struct sched_param schedParam;

  iRet = pthread_attr_init(&attr);
  if (iRet != 0) {
       printf("error2\n");
//something wrong
  }

  iRet = pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_JOINABLE);
  if (iRet != 0) {
       printf("error2\n");
//something wrong
  }

  iRet = pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);
  if (iRet != 0) {
       printf("error2\n");
//something wrong
  }


  iRet = pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
  if (iRet != 0) {
       printf("error2\n");
//something wrong
  }

  iRet = pthread_attr_getschedparam(&attr, &schedParam);
  if (iRet != 0) {
//something wrong
      printf("error2\n");
  }

  schedParam.sched_priority = iPriority;
  iRet = pthread_attr_setschedparam(&attr, &schedParam);
  if (iRet != 0) {
//something wrong
      printf("error3\n");
  }

  iRet = pthread_attr_setscope(&attr, PTHREAD_SCOPE_SYSTEM);
  if (iRet != 0) {
//something wrong
      printf("error4\n");
  }

  iRet = pthread_create(&pthreadId, &attr, thread_start, pModule);
  if (iRet != 0) {
    //some thing wrong
    printf("error5:%d\n",iRet);
  }

  iRet = pthread_setname_np(pthreadId, cThreadName);
  if (iRet != 0) {
    //some thing wrong
  }
  iRet = pthread_attr_destroy(&attr);
  if (iRet != 0) {
    //some thing wrong
  }

  return pthreadId;
}


void hiGetThreadPri(pthread_t pPid) {
  struct sched_param schedParam;
  int policy = 0;

  pthread_getschedparam(pPid, &policy, &schedParam);
  printf("policy=0x%x schedParam.sched_priority=0x%x\n", policy,
         schedParam.sched_priority);
  return;
}


void hiSetThreadsched(pthread_t pPid, const int32_t iPriority) {
  struct sched_param schedParam;
  memset(&schedParam, 0, sizeof(schedParam));
  schedParam.sched_priority = iPriority;
  pthread_setschedparam(pPid, SCHED_FIFO, &schedParam);
  return;
}

}  //namespace

//end of the file
