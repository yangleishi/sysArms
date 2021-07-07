/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_loger.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:日志模块，系统运行的情况记录在这个模块中。
*
********************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "sys_arms_loger.hpp"
#include "sys_arms_conf.hpp"
#include "sys_arms_daemon.hpp"


namespace LOGER {

pthread_mutex_t mPrintQueueMutex;
//app loger
static BASE::STR_QUEUE *mPrintQueue = NULL;
static FILE *   pPrintFile = NULL;

//app running data
static BASE::STR_QUEUE *mArmsDataQueue = NULL;
static FILE *   pArmsDataFile = NULL;

/////queue //////////////
static bool qEmpty(BASE::STR_QUEUE *q);

static bool qFull(BASE::STR_QUEUE *q);

static bool qInsert(BASE::STR_QUEUE *q, char* pStr);

static bool qDelete(BASE::STR_QUEUE *q, char* pStr);
///////////////////
static int32_t initLoger(BASE::LOG_THREAD_INFO *pTModule);

////////////////////////////////////////////////////////////////////////////////
///////internal interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
* 功能：判断log消息队列是否为空
* @param q : q是STR_QUEUE类型的指针，循环消息队列
* @return Descriptions
******************************************************************************/
static bool qEmpty(BASE::STR_QUEUE *q)
{
  bool iRet = true;
  iRet = (q->mFront == q->mRear) ? true : false;
  return iRet;
}

/******************************************************************************
* 功能：判断log消息队列是否满
* @param q : q是STR_QUEUE类型的指针，循环消息队列
* @return Descriptions
******************************************************************************/
static bool qFull(BASE::STR_QUEUE *q)
{
  bool iRet = true;
  iRet = ((q->mRear + 1)%PRINT_QUEUE_MAX_ITEMS == q->mFront) ? true : false;
  return iRet;
}

/******************************************************************************
* 功能：消息队列中插入一个待打印的字符串
* @param q : q是STR_QUEUE类型的指针，循环消息队列
* @param pStr : pStr是待存储的字符串
* @return Descriptions
******************************************************************************/
static bool qInsert(BASE::STR_QUEUE *q, char* pStr)
{
  bool iRet = true;
  if(qFull(q))
    return false;
  // 如果循环队列未满
  // 先将元素插入到循环队列的尾部，然后循环队列尾部的rear指针加一
  strcpy(q->mPrintPond[q->mRear].mString, pStr);
  q->mRear = (q->mRear + 1)%PRINT_QUEUE_MAX_ITEMS;

  return iRet;
}

/******************************************************************************
* 功能：消息队列中删除一个待打印的字符串
* @param q : q是STR_QUEUE类型的指针，循环消息队列
* @param pStr : pStr是存储的字符串
* @return Descriptions
******************************************************************************/
static bool qDelete(BASE::STR_QUEUE *q, char* pStr)
{
  if(qEmpty(q))
    return false;
  // 如果堆栈不为空,copy
  // 先将循环队列头部的元素取出，然后将头部指针加一
  strcpy(pStr, q->mPrintPond[q->mFront].mString);
  q->mFront = (q->mFront + 1)%PRINT_QUEUE_MAX_ITEMS;
  return true;
}


///////////////////////////////////
/******************************************************************************
* 功能：初始化消息队列
* @param pTModule : pTModule是线程信息结构体
* @return Descriptions
******************************************************************************/
static int32_t initLoger(BASE::LOG_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;

  mPrintQueue = pTModule->mLogQueue;
  mArmsDataQueue = pTModule->mArmsDataQueue;

  pthread_mutex_init(&mPrintQueueMutex, NULL);

  //open log
  pPrintFile = fopen(CONF::MN_LOGER_PRINTF_FILE, "a+");
  if(pPrintFile == NULL)
  {
    return  -1;
  }
  //open data log
  pArmsDataFile = fopen(CONF::MN_ARMS_DATA_FILE, "a+");
  if(pArmsDataFile == NULL)
  {
    return  -1;
  }
  return iRet;
}

/******************************************************************************
* 功能：释放消息队列申请的空间，删除锁
* @param pTModule : pTModule是线程信息结构体
* @return Descriptions
******************************************************************************/
static int moduleEndUp(BASE::LOG_THREAD_INFO *pTModule)
{
  mPrintQueue = NULL;
  mArmsDataQueue = NULL;

  pthread_mutex_destroy(&mPrintQueueMutex);

  if(pPrintFile != NULL)
    fclose(pPrintFile);

  if(pArmsDataFile != NULL)
    fclose(pArmsDataFile);

  printf("loger all endup\n");
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
///////external interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
* 功能：线程入口函数，此模块的生命周期就在此函数中。
* @param pTModule : pTModule是线程信息指针，里边包含发送/接收消息，socket等信息
* @return Descriptions
******************************************************************************/
void* threadEntry(void* pModule)
{
  BASE::LOG_THREAD_INFO *pTModule =(BASE::LOG_THREAD_INFO *) pModule;
  if(NULL == pTModule)
  {
    return 0;
  }
  //leader run in cpux
  BASE::hiSetCpuAffinity(pTModule->mCpuAffinity);

  if(initLoger(pTModule) != 0)
    pTModule->mState = BASE::M_STATE_STOP;

  BASE::PRINT_STR mLog, mArmsDataStr;
  //running state
  pTModule->mState = BASE::M_STATE_RUN;
  LOGER::PrintfLog(BASE::S_APP_LOGER, "%s running!",pTModule->mThreadName);
  static uint32_t mm = 0;
  while(pTModule->mWorking)
  {
    //TUDO*****
    pthread_mutex_lock(&mPrintQueueMutex);

    pthread_cond_wait(&pTModule->mPrintQueueReady, &mPrintQueueMutex);

    //app running loger
    while(qDelete(mPrintQueue, mLog.mString))
    {
      fprintf(pPrintFile,"%s\n", mLog.mString);
      //printf("************%s\n", mLog.mString);
    }

    //app running data
    while(qDelete(mArmsDataQueue, mArmsDataStr.mString))
    {
      fprintf(pArmsDataFile,"%s\n", mArmsDataStr.mString);
      //printf("************%s\n", mLog.mString);
    }

    pthread_mutex_unlock(&mPrintQueueMutex);

    fflush(pPrintFile);
    fflush(pArmsDataFile);
  }

  moduleEndUp(pTModule);
}

/******************************************************************************
* 功能：打印函数，此模块供其他线程使用，将log信息保存
* @param mWhichLog : mWhichLog是选择要保存类型：app系统运行信息，arms运行数据
* @param fm : fm是格式化类型
* @return Descriptions
******************************************************************************/
void PrintfLog(BASE::LOG_SAVA_W mWhichLog, const char *fm, ...)
{
  if(mPrintQueue == NULL)
    return ;

  BASE::PRINT_STR mStr;
  memset(mStr.mString, 0, PRINT_STRING_MAX_LENGTH);
  va_list args;
  va_start(args, fm);
  vsnprintf(mStr.mString, PRINT_STRING_MAX_LENGTH-1, fm, args);
  va_end( args );

  pthread_mutex_lock(&mPrintQueueMutex);
  if(mWhichLog == BASE::S_APP_LOGER)
    qInsert(mPrintQueue, mStr.mString);
  else if(mWhichLog == BASE::S_ARMS_DATA)
    qInsert(mArmsDataQueue, mStr.mString);
  pthread_mutex_unlock(&mPrintQueueMutex);

}

} //namespace
