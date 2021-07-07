/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_supr.hpp
* Module :
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:系统中的超级线程模块，此模块负责创建所有线程，管理其他模块，周期的发送
消息通知其他模块。
*
********************************************************************************/

#ifndef SYS_ARMS_SUPR_HPP
#define SYS_ARMS_SUPR_HPP

#include <stdint.h>

#include "sys_arms_defs.h"

namespace SUPR
{

  int32_t dmsAppStartUp(void);

} // namespace



#endif // SYS_ARMS_SUPR_HPP
