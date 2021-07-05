/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name :sys_arms_conf.hpp
* Module : 如果一个 .h 文件声明了多个模块概念, 则文件注释应当对文件的内容做一个大致的说明, 同时说明各
概念之间的联系;
* Create on: 2020/12/12
* Author: 师洋磊
* Email: 546783926@qq.com
* Description about this header file:整个系统的初始配置都在这个文件中，开启多少个线程，线程的名字
server client的IP和port
*
********************************************************************************/

#ifndef SYS_ARMS_CONF_HPP
#define SYS_ARMS_CONF_HPP

namespace CONF {


typedef enum {
  ARMS_M_SUPR_ID = 0, /*0 is special one, don't change it */
  ARMS_M_1_ID = 1,
  ARMS_M_2_ID,
  ARMS_M_3_ID,
  ARMS_M_4_ID,
  ARMS_M_5_ID,
  ARMS_M_6_ID,
  ARMS_M_7_ID,
  ARMS_M_8_ID,
  ARMS_M_9_ID,
  ARMS_M_10_ID,
  ARMS_M_11_ID,
  ARMS_M_MAX_ID,

  ARMS_T_1_ID = 12,
  ARMS_T_2_ID,
  ARMS_T_MAX_ID,

  ARMS_INTERACTION_ID = 14,
  ARMS_INTERACTION_MAX_ID,

  ARMS_LOG_1_ID = 15,
  ARMS_LOG_MAX_ID,
} MODULE_NAME_ID;


#define MODULES_NUMS (CONF::ARMS_LOG_MAX_ID)
//MN=Module Name
//
/*
 * arms size
 */
#define DEF_SYS_ARMS_NUMS 11
#define DEF_SYS_TENSIONLEADER_NUMS 2
#define DEF_SYS_USE_ARMS_NUMS 1

#define DEF_INTERACTION_TRANS_DATA_SIZE 2000
const char MN_NAME[][15] = {"MN_SUPR", "MN_SERVER1", "MN_SERVER2",
                            "MN_SERVER3", "MN_SERVER4", "MN_SERVER5",
                            "MN_SERVER6", "MN_SERVER7", "MN_SERVER8",
                            "MN_SERVER9", "MN_SERVER10", "MN_SERVER11"};

const char MN_TENSION_NAME[][15] = {"MN_TENSION1", "MN_TENSION2"};

const char MN_LOG_NAME[] = "MN_LOGS";

const char MN_INTERACTION_NAME[] = "MN_INTERACTION";
/*
const char MN_SERVER_IP[][16] = {"127.0.0.0", "127.0.0.0", "127.0.0.0",
                                 "127.0.0.0", "127.0.0.0", "127.0.0.0",
                                 "127.0.0.0", "127.0.0.0", "127.0.0.0",
                                 "127.0.0.0", "127.0.0.0", "127.0.0.0" };
*/

const char MN_SERVER_IP[][16] = {"192.168.1.100", "192.168.1.100", "192.168.1.100",
                                 "192.168.1.100", "192.168.1.100", "192.168.1.100",
                                 "192.168.1.100", "192.168.1.100", "192.168.1.100",
                                 "192.168.1.100", "192.168.1.100", "192.168.1.100",
                                 "192.168.1.100", "192.168.1.100", "192.168.1.100"};

const char MN_PEER_IP[][16] = {"192.168.1.2", "192.168.1.2", "192.168.1.2",
                                 "192.168.1.2", "192.168.1.2", "192.168.1.2",
                                 "192.168.1.2", "192.168.1.2", "192.168.1.2",
                                 "192.168.1.2", "192.168.1.2", "192.168.1.2",
                                 "192.168.1.2", "192.168.1.2", "192.168.1.2"};

const char MN_TENSION_SERVER_IP[][16] = {"192.168.1.100", "192.168.1.100"};

const int  MN_SERVER_PORT[DEF_SYS_ARMS_NUMS + 1] = { 8001, 8002, 8003,
                                                     8004, 8005, 8006,
                                                     8007, 8008, 8009,
                                                     8010, 8011, 8012};
const int  MN_PEER_PORT[DEF_SYS_ARMS_NUMS + 1] = { 9001, 9002, 9003,
                                                     9004, 9005, 9006,
                                                     9007, 9008, 9009,
                                                     9010, 9011, 9012};

const int  MN_TENSION_SERVER_PORT[DEF_SYS_TENSIONLEADER_NUMS + 1] = { 9001, 9002, 9003};


const char MN_INTERACTION_SERVER_IP[] = "192.168.1.100";
const int  MN_INTERACTION_SERVER_PORT = 10001;


const char MN_INTERACTION_CONF_FILE[] = ".armsConf.conf";
const char MN_LOGER_PRINTF_FILE[] = "sysArms.log";
const char MN_ARMS_DATA_FILE[] = "sysArms.data";

//*********////////// UDP timeout///////////////////
//m us,arms ctrl
const int SERVER_UDP_TIMEOUT_S = 0;
const int SERVER_UDP_TIMEOUT_US = 5000;

//m us rec tensions
const int SERVER_UDP_TENSION_TIMEOUT_S = 1;
const int SERVER_UDP_TENSION_TIMEOUT_US = 0;

//m us rec interaction
const int SERVER_UDP_INTERACTION_TIMEOUT_S = 1;
const int SERVER_UDP_INTERACTION_TIMEOUT_US = 0;

//Define priority for modules
const int PRI_SUPR = 50;
const int PRI_LEAD = 40;
const int PRI_LOGER = 30;

//Define modules CPU affinity
const int CPU_SUPR = 1;
const int CPU_LEAD = 2;
const int CPU_LOGER = 3;
const int CPU_TENSION = 4;
const int CPU_INTERACTIONER = 5;

//*********////////// interaction conf///////////////////
const float  IN_MAX_TENSION[DEF_SYS_USE_ARMS_NUMS] = { 100 };
const float  IN_OFFSET_X[DEF_SYS_USE_ARMS_NUMS] = { 0 };
const float  IN_OFFSET_Y[DEF_SYS_USE_ARMS_NUMS] = { 0 };
const float  IN_OFFSET_Z[DEF_SYS_USE_ARMS_NUMS] = { 0 };
const float  IN_OFFSET_W[DEF_SYS_USE_ARMS_NUMS] = { 0 };
const float  IN_OFFSET_ANGLE[DEF_SYS_USE_ARMS_NUMS] = { 0 };




}  //namespace

#endif // SYS_ARMS_CONF_HPP
