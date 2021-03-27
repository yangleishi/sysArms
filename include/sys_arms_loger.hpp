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

namespace LOGER{

void* threadEntry(void* pModule);

void PrintfLog(const char *fm, ...);
}

#endif // SYS_ARMS_LOGER_HPP
