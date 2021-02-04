/******************************************************************************
**
* Copyright (c)2021 SHI YANGLEI
* All Rights Reserved
*
*
******************************************************************************/
#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "sys_arms_loger.hpp"
#include "sys_arms_defs.h"
#include "sys_arms_conf.hpp"


namespace LOGER {

pthread_mutex_t mPrintQueueMutex;
static BASE::STR_QUEUE *mQueue = NULL;
static FILE *   pFile = NULL;

/////queue //////////////
static BASE::STR_QUEUE * qCreate(void);

static void qDestroy(BASE::STR_QUEUE *q);

static bool qEmpty(BASE::STR_QUEUE *q);

static bool qFull(BASE::STR_QUEUE *q);

static bool qInsert(BASE::STR_QUEUE *q, char* pStr);

static bool qDelete(BASE::STR_QUEUE *q, char* pStr);
///////////////////
static int32_t initLoger(BASE::LOG_THREAD_INFO *pTModule);

////////////////////////////////////////////////////////////////////////////////
///////internal interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
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

static void qDestroy(BASE::STR_QUEUE *q)
{
  if(NULL != q)
  {
    free(q);	// 执行销毁操作
  }
}

static bool qEmpty(BASE::STR_QUEUE *q)
{
  bool iRet = true;
  iRet = (q->mFront == q->mRear) ? true : false;
  return iRet;
}

static bool qFull(BASE::STR_QUEUE *q)
{
  bool iRet = true;
  iRet = ((q->mRear + 1)%PRINT_QUEUE_MAX_ITEMS == q->mFront) ? true : false;
  return iRet;
}

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

static int32_t initLoger(BASE::LOG_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;

  if((mQueue = qCreate()) == NULL)
    return -1;

  pthread_mutex_init(&mPrintQueueMutex, NULL);

  //open log
  pFile = fopen("sysArms.log", "a+");
  if(pFile == NULL)
  {
    return  -1;
  }

  return iRet;
}

static int moduleEndUp(BASE::LOG_THREAD_INFO *pTModule)
{
  qDestroy(mQueue);
  mQueue = NULL;

  pthread_mutex_destroy(&mPrintQueueMutex);

  if(pFile != NULL)
    fclose(pFile);

  printf("loger endup\n");
  return 0;
}

////////////////////////////////////////////////////////////////////////////////
///////external interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
void* threadEntry(void* pModule)
{
  BASE::LOG_THREAD_INFO *pTModule =(BASE::LOG_THREAD_INFO *) pModule;
  if(NULL == pTModule)
  {
    return 0;
  }

  BASE::PRINT_STR mLog;
  //running state
  while(pTModule->mWorking)
  {
    //TUDO*****
    pthread_mutex_lock(&mPrintQueueMutex);

    pthread_cond_wait(&pTModule->mPrintQueueReady, &mPrintQueueMutex);

    while(qDelete(mQueue, mLog.mString))
    {
      fprintf(pFile,"%s\n", mLog.mString);
    }
    pthread_mutex_unlock(&mPrintQueueMutex);

    fflush(pFile);
  }

  moduleEndUp(pTModule);
}

void PrintfLog(const char *fm, ...)
{
  BASE::PRINT_STR mStr;
  memset(mStr.mString, 0, PRINT_STRING_MAX_LENGTH);
  va_list args;
  va_start(args, fm);
  vsnprintf(mStr.mString, PRINT_STRING_MAX_LENGTH-1, fm, args);
  va_end( args );

  pthread_mutex_lock(&mPrintQueueMutex);
  qInsert(mQueue, mStr.mString);
  pthread_mutex_unlock(&mPrintQueueMutex);

}

} //namespace
