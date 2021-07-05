/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_interaction.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:上位机通信模块，接收上位机发来的控制指令，并将arms运行数据传输给上位机。
*
********************************************************************************/

#ifndef SYS_ARMS_INTERACTIONER_HPP
#define SYS_ARMS_INTERACTIONER_HPP

namespace INTERACTIONER {

void* threadEntry(void* pModule);

}

#endif // SYS_ARMS_INTERACTIONER_HPP
