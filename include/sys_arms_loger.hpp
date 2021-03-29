/******************************************************************************
**
* Copyright (c)2021 SHI YANGLEI
* All Rights Reserved
*
*
******************************************************************************/

#ifndef SYS_ARMS_LOGER_HPP
#define SYS_ARMS_LOGER_HPP
#include <pthread.h>
#include "sys_arms_defs.h"

namespace LOGER{

void* threadEntry(void* pModule);

void PrintfLog(BASE::LOG_SAVA_W mWhichLog, const char *fm, ...);

}

#endif // SYS_ARMS_LOGER_HPP
