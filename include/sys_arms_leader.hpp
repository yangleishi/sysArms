/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_leader.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:机械臂控制模块，控制对应的机械臂，接收supr发来的周期性的控制指令，
将控制指令发送给机械臂板子，并接收机械臂板子发来的运行数据。
*
********************************************************************************/

#ifndef SYS_ARMS_LEADER_HPP
#define SYS_ARMS_LEADER_HPP


namespace LEADER {

void* threadEntry(void* pModule);

}

#endif // SYS_ARMS_LEADER_HPP
