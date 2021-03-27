/******************************************************************************
**
* Copyright (c)2021 SHI YANGLEI
* All Rights Reserved
*
*
******************************************************************************/
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
