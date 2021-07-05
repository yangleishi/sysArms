/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_tension_leader.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:拉立计模块，由于此系统中拉力计数据单独传输，因此此模块负责收发拉力计相关的消息。
消息通知其他模块。
*
********************************************************************************/


#ifndef SYS_ARMS_TENSION_LEADER_HPP
#define SYS_ARMS_TENSION_LEADER_HPP

namespace TENSIONLEADER {

void* threadEntry(void* pModule);

}
#endif // SYS_ARMS_TENSION_LEADER_HPP
