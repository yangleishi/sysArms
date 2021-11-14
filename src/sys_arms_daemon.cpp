/********************************************************************************
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

#include <unistd.h>
#include <stdio.h>
#include <sys/stat.h>
#include <pthread.h>
#include <string.h>

#include "sys_arms_daemon.hpp"

namespace BASE {
/******************************************************************************
* @param vDaemonName : [in/out]待创建进程的名字，字符串类型

* @return Descriptions
* @TUDO
******************************************************************************/
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


/******************************************************************************
* @param cThreadName : [in]待创建线程的名字，字符串类型
* @param thread_start : [in]待创建线程的函数入口
* @param iPriority : [in]待创建线程的优先级
* @param pModule : [in]待创建线程的参数指针
* @return Descriptions
* @exception pthread_t: 此函数返回新创建的线程的pid
******************************************************************************/
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

/******************************************************************************
* @param pPid : [in]待设置线程的pid
* @param iPriority : [in]待设置线程的优先级
* @return Descriptions
* 此函数设置线程的优先级
******************************************************************************/
void hiSetThreadsched(pthread_t pPid, const int32_t iPriority) {
  struct sched_param schedParam;
  memset(&schedParam, 0, sizeof(schedParam));
  schedParam.sched_priority = iPriority;
  pthread_setschedparam(pPid, SCHED_FIFO, &schedParam);
  return;
}

/******************************************************************************
* @param mCpuI : [in]当前线程要绑定的cpu号，**特别注意自己控制器有多少核
* @return Descriptions
* 此函数将当前线程绑定到mCpuI核上
******************************************************************************/
void hiSetCpuAffinity(int mCpuI)
{
    //return;
  cpu_set_t mask;
  CPU_ZERO(&mask);
  CPU_SET(mCpuI, &mask);

  pthread_setaffinity_np(pthread_self(), sizeof(mask),&mask);
}

}  //namespace

//end of the file
