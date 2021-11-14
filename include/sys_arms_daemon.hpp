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
#include <stdlib.h>

#ifndef SYS_ARMS_DAEMON_HPP
#define SYS_ARMS_DAEMON_HPP
namespace BASE {

void hiCreateDaemon(const char *vDaemonName);

pthread_t hiCreateThread(const char * cThreadName,
                         void *(*thread_start)(void *),
                         int32_t iPriority,
                         void* pModule);

void hiGetThreadPri(pthread_t pPid);
void hiSetThreadsched(pthread_t pPid, const int32_t iPriority);

void hiSetCpuAffinity(int mCpuI);
}


#endif // SYS_ARMS_DAEMON_HPP
