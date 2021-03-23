/******************************************************************************
**
* Copyright (c)2021 SHI YANGLEI
* All Rights Reserved
*
*
******************************************************************************/
#ifndef SYS_ARMS_SUPR_HPP
#define SYS_ARMS_SUPR_HPP

#include <stdint.h>

#include "sys_arms_defs.h"

namespace SUPR
{

  int32_t dmsAppStartUp(void);

  //data param
  extern BASE::ARMS_MSGS mArmsMsgs;
} // namespace



#endif // SYS_ARMS_SUPR_HPP
