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

#ifndef SYS_ARMS_LOGER_HPP
#define SYS_ARMS_LOGER_HPP
#include <pthread.h>
#include "sys_arms_defs.h"

namespace LOGER{

void* threadEntry(void* pModule);

void PrintfLog(BASE::LOG_SAVA_W mWhichLog, const char *fm, ...);

}

#endif // SYS_ARMS_LOGER_HPP
