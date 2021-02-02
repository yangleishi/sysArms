/******************************************************************************
**
* Copyright (c)2021 SHI YANGLEI
* All Rights Reserved
*
*
******************************************************************************/

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
} DMS_MODULE_NAME_ID;

typedef enum {
  ARMS_T_1_ID = 0,
  ARMS_T_2_ID,
  ARMS_T_MAX_ID,
} DMS_MODULE_TENSION_NAME_ID;

//MN=Module Name
//
/*
 * arms size
 */
#define DEF_SYS_ARMS_NUMS 11
#define DEF_SYS_TENSIONLEADER_NUMS 2

const char MN_NAME[][15] = {"MN_SUPR", "MN_SERVER1", "MN_SERVER2",
                            "MN_SERVER3", "MN_SERVER4", "MN_SERVER5",
                            "MN_SERVER6", "MN_SERVER7", "MN_SERVER8",
                            "MN_SERVER9", "MN_SERVER10", "MN_SERVER11"};

const char MN_TENSION_NAME[][15] = {"MN_TENSION1", "MN_TENSION2"};

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

const char MN_TENSION_SERVER_IP[][16] = {"192.168.1.100", "192.168.1.100"};

const int  MN_SERVER_PORT[DEF_SYS_ARMS_NUMS + 1] = { 8001, 8002, 8003,
                                                     8004, 8005, 8006,
                                                     8007, 8008, 8009,
                                                     8010, 8011, 8012};

const int  MN_TENSION_SERVER_PORT[DEF_SYS_TENSIONLEADER_NUMS + 1] = { 9001, 9002, 9003};

const int SERVER_UDP_TIMEOUT = 5000;

const int SERVER_UDP_TENSION_TIMEOUT = 1000000;

//Define priority for modules
const int PRI_SUPR = 50;
const int PRI_LEAD = 70;


}  //namespace

#endif // SYS_ARMS_CONF_HPP
