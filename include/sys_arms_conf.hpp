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

//最大可以使用的机械臂leader数量
#define DEF_SYS_MAX_ARMS_NUMS 11
//系统中使用的机械臂leader数量
#define DEF_SYS_USE_ARMS_NUMS 1

//最大可以使用的拉力计接收模块数量
#define DEF_SYS_MAX_TENSIONLEADER_NUMS 2
//系统中使用的拉力计接收模块数量，0时候采用有线传输（拉力计信息在有线传输协议里）
#define DEF_SYS_USE_TENSIONLEADER_NUMS 0

//编码器0点位置,千分度。25度，这个值是标定值
#define DEF_SYS_ENCODER_ZERO 20000
//编码器最大转动角度
#define DEF_SYS_ENCODER_MAX 180000

//定义PI
#define DEF_SYS_PI     3.1415926
#define DEF_SYS_DEGREE_TO_RADIAN    0.00001745
//g 重力加速度
#define DEF_SYS_MG    0.0098

//1 rad/s 转换成电机执行的速度 10000 脉冲/s
#define DEF_SYS_RADIAN_TO_PULSE    1591.54

static const unsigned int nDelay = 10000;        /* usec */


//系统中所有线程模块ID
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
  ARMS_T_MAX_ID = ARMS_M_MAX_ID  + DEF_SYS_USE_TENSIONLEADER_NUMS,

  ARMS_INTERACTION_ID = 14,
  ARMS_INTERACTION_MAX_ID,

  ARMS_LOG_1_ID = 15,
  ARMS_LOG_MAX_ID,
} MODULE_NAME_ID;

//最大的线程模块数量
#define MODULES_NUMS (CONF::ARMS_LOG_MAX_ID)

//线程模块的名字
const char MN_NAME[][15] = {"MN_SUPR", "MN_SERVER1", "MN_SERVER2",
                            "MN_SERVER3", "MN_SERVER4", "MN_SERVER5",
                            "MN_SERVER6", "MN_SERVER7", "MN_SERVER8",
                            "MN_SERVER9", "MN_SERVER10", "MN_SERVER11"};

const char MN_TENSION_NAME[][15] = {"MN_TENSION1", "MN_TENSION2"};

const char MN_LOG_NAME[] = "MN_LOGS";

const char MN_INTERACTION_NAME[] = "MN_INTERACTION";


//线程模块使用的IP和端口
const char MN_SERVER_IP[][16] = {"192.168.1.100", "192.168.1.100", "192.168.1.100",
                                 "192.168.1.100", "192.168.1.100", "192.168.1.100",
                                 "192.168.1.100", "192.168.1.100", "192.168.1.100",
                                 "192.168.1.100", "192.168.1.100", "192.168.1.100",
                                 "192.168.1.100", "192.168.1.100", "192.168.1.100"};

const char MN_PEER_IP[][16] = {"192.168.1.99", "192.168.1.99", "192.168.1.99",
                                 "192.168.1.99", "192.168.1.99", "192.168.1.99",
                                 "192.168.1.99", "192.168.1.99", "192.168.1.99",
                                 "192.168.1.99", "192.168.1.99", "192.168.1.99",
                                 "192.168.1.99", "192.168.1.99", "192.168.1.99"};

const char MN_TENSION_SERVER_IP[][16] = {"192.168.1.100", "192.168.1.100"};

const int  MN_SERVER_PORT[DEF_SYS_MAX_ARMS_NUMS + 1] = { 8001, 8002, 8003,
                                                     8004, 8005, 8006,
                                                     8007, 8008, 8009,
                                                     8010, 8011, 8012};
const int  MN_PEER_PORT[DEF_SYS_MAX_ARMS_NUMS + 1] = { 8889, 8889, 8889,
                                                     8889, 8889, 8889,
                                                     8889, 8889, 8889,
                                                     8889, 8889, 8889};

const int  MN_TENSION_SERVER_PORT[DEF_SYS_MAX_TENSIONLEADER_NUMS] = { 9001, 9002};

const char MN_INTERACTION_SERVER_IP[] = "192.168.1.100";
const int  MN_INTERACTION_SERVER_PORT = 10001;

//log日志保存的几个文件路径
const char MN_INTERACTION_CONF_FILE[] = ".armsConf.conf";
const char MN_LOGER_PRINTF_FILE[] = "sysArms.log";
const char MN_ARMS_DATA_FILE[] = "sysArms.data";

//*********////////// UDP timeout///////////////////
//定义模块线程UDP接收消息超时
const int SERVER_UDP_TIMEOUT_S = 0;
const int SERVER_UDP_TIMEOUT_US = 1000;
const int SERVER_UDP_TENSION_TIMEOUT_S = 1;
const int SERVER_UDP_TENSION_TIMEOUT_US = 0;
const int SERVER_UDP_INTERACTION_TIMEOUT_S = 1;
const int SERVER_UDP_INTERACTION_TIMEOUT_US = 0;

//定义线程模块优先级
const int PRI_SUPR = 50;
const int PRI_LEAD = 40;
const int PRI_LOGER = 30;

//定义线程模块的CPU亲和度，绑定核
const int CPU_SUPR = 1;
const int CPU_LEAD = 2;
const int CPU_LOGER = 3;
const int CPU_TENSION = 4;
const int CPU_INTERACTIONER = 5;

//*********////////// 上位机线程 显示界面 配置数据interaction conf///////////////////
const float  IN_MAX_TENSION[DEF_SYS_USE_ARMS_NUMS] = { 100 };
const float  IN_OFFSET_X[DEF_SYS_USE_ARMS_NUMS] = { 0 };
const float  IN_OFFSET_Y[DEF_SYS_USE_ARMS_NUMS] = { 0 };
const float  IN_OFFSET_Z[DEF_SYS_USE_ARMS_NUMS] = { 0 };
const float  IN_OFFSET_W[DEF_SYS_USE_ARMS_NUMS] = { 0 };
const float  IN_OFFSET_ANGLE[DEF_SYS_USE_ARMS_NUMS] = { 0 };

//*********////////// 控制算法PID参数///////////////////
const float  PID_P_FOLLOWUP = 1000.0;
const float  PID_D_FOLLOWUP = 4000.0;
//绳索的长度，暂时设置成固定值，等调试完毕该值是变动的
const float  ROPE_LEN = 1.5;


}  //namespace

#endif // SYS_ARMS_CONF_HPP
