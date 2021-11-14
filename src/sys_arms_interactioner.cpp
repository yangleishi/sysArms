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

#include <stdio.h>
#include <unistd.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

#include "sys_arms_loger.hpp"
#include "sys_arms_defs.h"
#include "sys_arms_conf.hpp"
#include "sys_arms_interactioner.hpp"
#include "sys_arms_daemon.hpp"


static BASE::ReadConfData mParames[DEF_SYS_MAX_ARMS_NUMS] = {0};

namespace INTERACTIONER {

static char printfC[100] = {0};
pthread_mutex_t *mPrintQueueMutex = NULL;


static int initServer(BASE::INTERACTION_THREAD_INFO *pTModule);
static void setFdTimeout(int sockfd, const int mSec, const int mUsec);

static void sendMsgToUpper(BASE::INTERACTION_THREAD_INFO *pTModule, const uint16_t StatusWord, const uint16_t StatusCode, const char* pData, int dataSize);

static void changeState(BASE::INTERACTION_THREAD_INFO *pTModule, uint16_t mCmd);

//config file
static int readConfig(BASE::ReadConfData  * mParame);
static int writeConfig(BASE::SaveConfData * sParame, const int mPNum);

static int sendPlayBack(BASE::INTERACTION_THREAD_INFO *pTModule, int32_t mStart);
//config handle move
static int sendMsgToSupr(BASE::INTERACTION_THREAD_INFO *pTModule, int pNewRecType);

////////////////////////////////////////////////////////////////////////////////
///////internal interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
* 功能：此函数设置socket套接字超时时间。
* @param sockfd : [in]套接子id，
* @param mSec : [in]套接字超时妙
* @param mUsec : [in]套接字超时微妙
* @return Descriptions
******************************************************************************/
static void setFdTimeout(int sockfd, const int mSec, const int mUsec)
{
  struct timeval timeout;
  timeout.tv_sec = mSec;//秒
  timeout.tv_usec = mUsec;//微秒
  if (setsockopt(sockfd, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout)) == -1)
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "interaction set fd error\n");
  }
}

/******************************************************************************
* 功能：此函数初始化模块，interaction作为服务器端，初始化套接字，绑定ip端口
* @param pTModule : pTModule是线程信息结构体指针，里边存储的线程运行期间用到的数据和交换通信数据
* @return Descriptions
******************************************************************************/
static int initServer(BASE::INTERACTION_THREAD_INFO *pTModule)
{
  int32_t iRet = 0;

  if((pTModule->mSocket = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
      LOGER::PrintfLog(BASE::S_APP_LOGER, "socket creat Failed");
      return -1;
  }

  //setFdNonblocking(pTModule->mSocket);
  setFdTimeout(pTModule->mSocket, CONF::SERVER_UDP_INTERACTION_TIMEOUT_S, CONF::SERVER_UDP_INTERACTION_TIMEOUT_US);

  bzero(&(pTModule->mPeerAddr), sizeof(pTModule->mPeerAddr));

  bzero(&(pTModule->mSerAddr), sizeof(pTModule->mSerAddr));
  pTModule->mSerAddr.sin_family = AF_INET;
  pTModule->mSerAddr.sin_port = htons(pTModule->mMyPort);
  pTModule->mSerAddr.sin_addr.s_addr = inet_addr(pTModule->mMyIpV4Str);

  iRet = bind(pTModule->mSocket, (struct sockaddr*)&(pTModule->mSerAddr), sizeof(pTModule->mSerAddr));

  if(iRet < 0)
    return iRet;

  return 0;
}

/******************************************************************************
* 功能：此函数销毁初始化模块，释放线程指针里的内容
* @param pTModule : pTModule是线程信息结构体指针，里边存储的线程运行期间用到的数据和交换通信数据
* @return Descriptions
******************************************************************************/
static int moduleEndUp(BASE::INTERACTION_THREAD_INFO *pTModule)
{
  if(pTModule->mSocket >= 0)
  {
    close(pTModule->mSocket);
    pTModule->mSocket = -1;
  }
  LOGER::PrintfLog(BASE::S_APP_LOGER, "interaction endup \n");
  return 0;
}

/******************************************************************************
* 功能：此函数读取配置文件，将系统配置文件读取到ReadConfData结构体中
* @param mParame : mParame是保存配置信息结构体指针，主要是供上位机显示，内部系统使用。
                   该配置信息可以通过上位机设置
* @return Descriptions
******************************************************************************/
static int readConfig(BASE::ReadConfData * mParame)
{
  int iRet = 0;
  BASE::ReadConfData * mTParame = mParame;
  FILE *pFile = fopen(CONF::MN_INTERACTION_CONF_FILE, "r");

  if(pFile == NULL)
  {
    if((pFile = fopen(CONF::MN_INTERACTION_CONF_FILE, "w")) == NULL)
    {
      LOGER::PrintfLog(BASE::S_APP_LOGER, "--------------\n");
      return  -1;
    }

    for (int i=0; i<DEF_SYS_MAX_ARMS_NUMS; i++)
    {
      fprintf(pFile, "%f %f %f %f %f %f\n", CONF::IN_MAX_TENSION[i], CONF::IN_OFFSET_X[i], CONF::IN_OFFSET_Y[i], CONF::IN_OFFSET_Z[i], CONF::IN_OFFSET_W[i], CONF::IN_OFFSET_ANGLE[i]);
      mTParame->mConfSaveWeight = CONF::IN_MAX_TENSION[i];
      mTParame->mConfSaveEncoderX = CONF::IN_OFFSET_X[i];
      mTParame->mConfSaveEncoderY = CONF::IN_OFFSET_Y[i];
      mTParame->mConfSaveEncoderZ = CONF::IN_OFFSET_Z[i];
      mTParame->mConfSaveEncoderP = CONF::IN_OFFSET_W[i];
      mTParame->mConfSaveEncoderT = CONF::IN_OFFSET_ANGLE[i];
      mTParame++;
    }
    fclose(pFile);
    return 0;
  }

  memset((char*)mTParame, 0, sizeof(BASE::ReadConfData)*DEF_SYS_MAX_ARMS_NUMS);
  for (int i=0; i<DEF_SYS_MAX_ARMS_NUMS; i++)
  {

    fscanf(pFile, "%f %f %f %f %f %f\n",
                    &(mTParame->mConfSaveWeight),
                    &(mTParame->mConfSaveEncoderX),
                    &(mTParame->mConfSaveEncoderY),
                    &(mTParame->mConfSaveEncoderZ),
                    &(mTParame->mConfSaveEncoderP),
                    &(mTParame->mConfSaveEncoderT)
                    );

    //printf("%f %f %f %f %f %f\n",mTParame->mConfSaveWeight, mTParame->mConfSaveEncoderX, mTParame->mConfSaveEncoderY,
    //                             mTParame->mConfSaveEncoderZ, mTParame->mConfSaveEncoderP, mTParame->mConfSaveEncoderT);
    mTParame++;
  }

  fclose(pFile);
  return iRet;
}


/******************************************************************************
* 功能：此函数保存配置文件模块，将系统sParame保存到配置文件中
* @param mParame : sParame是保存配置信息结构体指针，
* @param mPNum : smPNum是保存配置信息条数，
* @return Descriptions
******************************************************************************/
static int writeConfig(BASE::SaveConfData * sParame, const int mPNum)
{
  int iRet = 0;
  FILE *pFile = fopen(CONF::MN_INTERACTION_CONF_FILE, "rw+");

  if(pFile == NULL)
    return -1;

  readConfig(mParames);
  for (int i=0; i<DEF_SYS_MAX_ARMS_NUMS;i++)
  {
      if(sParame[i].mIsValid == 1)
      {
          mParames[i].mConfSaveWeight = sParame[i].mConfSaveWeight;
          mParames[i].mConfSaveEncoderX = sParame[i].mConfSaveEncoderX;
          mParames[i].mConfSaveEncoderY = sParame[i].mConfSaveEncoderY;
          mParames[i].mConfSaveEncoderZ = sParame[i].mConfSaveEncoderZ;
          mParames[i].mConfSaveEncoderP = sParame[i].mConfSaveEncoderP;
          mParames[i].mConfSaveEncoderT = sParame[i].mConfSaveEncoderT;
      }
  }

  for (int i=0; i<mPNum; i++)
  {
    fprintf(pFile, "%f %f %f %f %f %f\n",
                   mParames[i].mConfSaveWeight,
                   mParames[i].mConfSaveEncoderX,
                   mParames[i].mConfSaveEncoderY,
                   mParames[i].mConfSaveEncoderZ,
                   mParames[i].mConfSaveEncoderP,
                   mParames[i].mConfSaveEncoderT);
    printf("%f %f %f %f %f %f  %d\n",mParames[i].mConfSaveWeight, mParames[i].mConfSaveEncoderX, mParames[i].mConfSaveEncoderY,
                                     mParames[i].mConfSaveEncoderZ, mParames[i].mConfSaveEncoderP, mParames[i].mConfSaveEncoderT, mParames[i].mIsValid);
  }

  fclose(pFile);
  return iRet;
}


/******************************************************************************
* 功能：此函数保将msg消息发送给上位机
* @param pTModule : pTModule是线程信息指针，里边包含发送/接收消息，socket等信息
* @param StatusWord : StatusWord是msg状态字，
* @param StatusCode : StatusCode是msg状态码
* @param pData : pData是msg的数据段指针，要发送的数据
* @param dataSize : dataSize是msg的数据的大小
* @return Descriptions
******************************************************************************/
static void sendMsgToUpper(BASE::INTERACTION_THREAD_INFO *pTModule, const uint16_t StatusWord, const uint16_t StatusCode, const char* pData, int dataSize)
{
    BASE::MArmsUpData mSendMsg;
    mSendMsg.StatusWord = StatusWord;
    mSendMsg.StatusCode = StatusCode;
    if(dataSize > 0 && dataSize < MSG_UP_DATA_MAX)
        memcpy(mSendMsg.Datas, pData, dataSize);

    sendto(pTModule->mSocket, (char*)&mSendMsg, sizeof(BASE::MArmsUpData), 0, (sockaddr*)&pTModule->mPeerAddr, sizeof(pTModule->mPeerAddr));
}

static void changeState(BASE::INTERACTION_THREAD_INFO *pTModule, uint16_t mCmd)
{
    /*
    switch (pTModule->mState)
    {
      case BASE::M_STATE_INIT:
      {
        if((mCmd == BASE::CMD_SAVE_CONF) || (mCmd == BASE::CMD_READ_CONF))
        {
            pTModule->mNewState = BASE::M_STATE_CONF;
            pTModule->mIsStataChange = true;
        }
        break;
      }
      case BASE::M_STATE_CONF:
      {
        if(mCmd == BASE::CMD_RUN_START)
        {
            pTModule->mNewState = BASE::M_STATE_RUN;
            pTModule->mIsStataChange = true;
        }
        break;
      }
      case BASE::M_STATE_RUN:
      {
        break;
      }
      case BASE::M_STATE_STOP:
      {
        break;
      }
      default:
      {
        break;
      }
    }
    */
    ;
}


/******************************************************************************
* 功能：此函数保将上位机msg消息supr线程，supr发送周期控制指令给leader
* @param pTModule : pTModule是线程信息指针，里边包含发送/接收消息，socket等信息
* @param pNewRecType :发送给supr的cmd类型
* @return Descriptions
******************************************************************************/
static int sendMsgToSupr(BASE::INTERACTION_THREAD_INFO *pTModule, int pNewRecType)
{
    LOGER::PrintfLog(BASE::S_APP_LOGER, "interaction CMD %d", pNewRecType);
    pthread_mutex_lock(&pTModule->mInterToSuprDatas->mInteractionRecMutex);
    pTModule->mInterToSuprDatas->mIsNewRec = pNewRecType;
    memcpy((char*)&pTModule->mInterToSuprDatas->mRecMsg, (char*)&pTModule->mRecMsg, sizeof(pTModule->mRecMsg));
    pthread_mutex_unlock(&pTModule->mInterToSuprDatas->mInteractionRecMutex);
}

static int sendPlayBack(BASE::INTERACTION_THREAD_INFO *pTModule, int32_t mStart)
{
  int iRet = 0;
  FILE * pPlayBackFile = NULL;
  if((pPlayBackFile = fopen(CONF::MN_ARMS_DATA_FILE, "r")) == NULL)
  {
    return  -1;
  }

  int readIndex = 0;
  BASE::ReadRunAllData *mSendDatas = (BASE::ReadRunAllData *)malloc(sizeof(BASE::ReadRunAllData)*DEF_SYS_MAX_ARMS_NUMS);
  memset((char*)mSendDatas, 0, sizeof(BASE::ReadRunAllData)*DEF_SYS_MAX_ARMS_NUMS);

  while (iRet != EOF)
  {
    iRet = fscanf(pPlayBackFile, "%d %f %f %f %f %f %f %f %f %f %f %f %f %f %f %f\n",
            &readIndex, &(mSendDatas[0].runD_Rsiko1),&(mSendDatas[0].runD_Rsiko2),&(mSendDatas[0].runD_Level1),&(mSendDatas[0].runD_Level2),
            &(mSendDatas[0].runD_RencoderT),&(mSendDatas[0].runD_PullNow),&(mSendDatas[0].runD_PullSet),
            &(mSendDatas[0].runD_VXNow),&(mSendDatas[0].runD_VXSet),&(mSendDatas[0].runD_VYNow),&(mSendDatas[0].runD_VYSet),
            &(mSendDatas[0].runD_VZNow),&(mSendDatas[0].runD_VZSet),&(mSendDatas[0].runD_VWNow),&(mSendDatas[0].runD_VWSet));
    if(readIndex == mStart)
    {
      break;
    }
  }
  sendMsgToUpper(pTModule, BASE::CMD_ACK_READ_RUNNING_DATAS, 0, (char*)mSendDatas, sizeof(BASE::ReadRunAllData)*DEF_SYS_MAX_ARMS_NUMS);
  free(mSendDatas);
  fclose(pPlayBackFile);
}
////////////////////////////////////////////////////////////////////////////////
///////external interface //////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////
/******************************************************************************
* 功能：线程入口函数，此模块的生命周期就在此函数中。
* @param pTModule : pTModule是线程信息指针，里边包含发送/接收消息，socket等信息
* @param mCmdCyc  : mCmdCyc是循环读取周期数据cmd命令
* @return Descriptions
******************************************************************************/
void* threadEntry(void* pModule)
{
  BASE::INTERACTION_THREAD_INFO *pTModule =(BASE::INTERACTION_THREAD_INFO *) pModule;
  if(NULL == pTModule)
  {
    return 0;
  }
  //设置此线程的cpu亲和度
  BASE::hiSetCpuAffinity(pTModule->mCpuAffinity);

  //log打印队列锁
  mPrintQueueMutex = pTModule->mPrintQueueMutex;

  //read conf
  readConfig(mParames);

  //初始化模块
  if(initServer(pTModule) != 0)
  {
    LOGER::PrintfLog(BASE::S_APP_LOGER, "interaction  bind server ip failed, check network again !");
    moduleEndUp(pTModule);
    pTModule->mWorking = false;
    return 0;
  }

  socklen_t mun = sizeof(pTModule->mPeerAddr);

  //线程运行起初设置配置状态
  pTModule->mState = BASE::M_STATE_CONF;

  LOGER::PrintfLog(BASE::S_APP_LOGER, "interaction running!");
  BASE::ARMS_R_USE_MSG *mArmsDatas = (BASE::ARMS_R_USE_MSG *)malloc(sizeof(BASE::ARMS_R_USE_MSG)*DEF_SYS_MAX_ARMS_NUMS);
  //线程陷入循环
  while(pTModule->mWorking)
  {
    //接收上位机指令，如没有指令，阻塞一定时间。
    int size = recvfrom(pTModule->mSocket , (char*)&(pTModule->mRecMsg), sizeof(BASE::MArmsDownData), 0, (sockaddr*)&(pTModule->mPeerAddr), &mun);

    //读取每个机械臂运行数据,
    pthread_mutex_lock(&pTModule->mSuprDatasToInterasction->mArmsNowDatasMutex);
    memcpy((char*)mArmsDatas, (char*)pTModule->mSuprDatasToInterasction->mRecMsgsDatas, sizeof(BASE::ARMS_R_USE_MSG)*DEF_SYS_MAX_ARMS_NUMS);
    pthread_mutex_unlock(&pTModule->mSuprDatasToInterasction->mArmsNowDatasMutex);

    //接收上位机指令，如没有指令，阻塞一定时间。
    switch (pTModule->mRecMsg.CmdIdentify)
    {
      case BASE::CMD_LINK:
      {
        //LOGER::PrintfLog(BASE::S_APP_LOGER, "test log hear");
        pTModule->mNewState = BASE::M_STATE_CONF;
        pTModule->mIsStataChange = true;
        pTModule->mState = BASE::M_STATE_CONF;

        printf("interaction CMD link\n");
        //TUDO  send msg to arms concl
        memset((char*)mParames, 0, sizeof(BASE::ReadConfData)*DEF_SYS_MAX_ARMS_NUMS);
        readConfig(mParames);

        sendMsgToUpper(pTModule, BASE::CMD_ACK_LINK_OK, 0, (char*)mArmsDatas, sizeof(BASE::ARMS_R_USE_MSG)*DEF_SYS_MAX_ARMS_NUMS);
        break;
      }
      case BASE::CMD_UNLINK:
      {
        printf("interaction CMD unlink\n");
        //改变整个系统的运行状态到STOP
        pTModule->mNewState = BASE::M_STATE_STOP;
        pTModule->mIsStataChange = true;
        pTModule->mState = BASE::M_STATE_STOP;

        sendMsgToUpper(pTModule, BASE::CMD_ACK_UNLINK_OK, 0, 0, 0);
        break;
      }
      case BASE::CMD_SAVE_CONF:
      {
        //BASE::SaveConfData *mSavaConf = (BASE::SaveConfData *)pTModule->mRecMsg.Datas;
        writeConfig((BASE::SaveConfData *)pTModule->mRecMsg.Datas, DEF_SYS_MAX_ARMS_NUMS);
        printf("interaction CMD Sava conf\n");
        break;
      }
      case BASE::CMD_READ_CONF:
      {
        memset((char*)mParames, 0, sizeof(BASE::ReadConfData)*DEF_SYS_MAX_ARMS_NUMS);
        readConfig(mParames);

        printf("interaction CMD read conf\n");
        sendMsgToUpper(pTModule, BASE::CMD_ACK_READCONF_OK, 0, (char*)mArmsDatas, sizeof(BASE::ARMS_R_USE_MSG)*DEF_SYS_MAX_ARMS_NUMS);
        break;
      }
      case BASE::CMD_HAND_MOVE_START:
      case BASE::CMD_HAND_MOVE_STOP:
      {
        if(pTModule->mState = BASE::M_STATE_CONF)
            sendMsgToSupr(pTModule, pTModule->mRecMsg.CmdIdentify);
        break;
      }
      case BASE::CMD_CYC_READ_LIFT_DATAS:
      {
        if(pTModule->mState == BASE::M_STATE_CONF)
            sendMsgToUpper(pTModule, BASE::CMD_ACK_READ_LIFT_DATAS, 0, (char*)mArmsDatas, sizeof(BASE::ARMS_R_USE_MSG)*DEF_SYS_MAX_ARMS_NUMS);

        break;
      }
      case BASE::CMD_CYC_READ_SYS_DELAYED:
      {
        sendMsgToUpper(pTModule, BASE::CMD_ACK_READ_LIFT_DATAS, 0, (char*)mArmsDatas, sizeof(BASE::ARMS_R_USE_MSG)*DEF_SYS_MAX_ARMS_NUMS);
        break;
      }
      case BASE::CMD_ALL_MOVE_START:
      {
        if(pTModule->mState == BASE::M_STATE_CONF)
            sendMsgToSupr(pTModule, BASE::CMD_ALL_MOVE_START);
        printf("interaction CMD all move start\n");
        break;
      }
      case BASE::CMD_ALL_MOVE_STOP:
      {
        if(pTModule->mState == BASE::M_STATE_CONF)
            sendMsgToSupr(pTModule, BASE::CMD_ALL_MOVE_STOP);
        printf("interaction CMD all move stop\n");
        break;
      }
      case BASE::CMD_ALL_PULL_START:
      {
        if(pTModule->mState == BASE::M_STATE_CONF)
        {
            //tension turn kg
            BASE::LiftCmdData *mAllPull = (BASE::LiftCmdData *)pTModule->mRecMsg.Datas;
            for (int i=0; i<DEF_SYS_MAX_ARMS_NUMS; i++)
            {
                //if(mAllPull[i].mIsValid)
                //    mAllPull[i].v_p[3] *= ((mParames[i].mConfSaveWeight)/100.0);
            }
            sendMsgToSupr(pTModule, BASE::CMD_ALL_PULL_START);
        }
        printf("interaction CMD all pull start\n");
        break;
      }
      case BASE::CMD_ALL_PULL_STOP:
      {
        if(pTModule->mState == BASE::M_STATE_CONF)
            sendMsgToSupr(pTModule, BASE::CMD_ALL_PULL_STOP);
        printf("interaction CMD all pull stop\n");
        break;
      }
      case BASE::CMD_RUN_START:
      {
        //改变整个系统的运行状态到RUN
        if(pTModule->mState == BASE::M_STATE_CONF)
        {
          pTModule->mNewState = BASE::M_STATE_RUN;
          pTModule->mIsStataChange = true;
          pTModule->mState = BASE::M_STATE_RUN;
        }

        //在RUN状态下修改，Start命令发送
        sendMsgToSupr(pTModule, BASE::CMD_RUN_START);
        printf("interaction CMD run start\n");
        break;
      }
      case BASE::CMD_CYC_READ_RUNNING_DATAS:
      {
        sendMsgToUpper(pTModule, BASE::CMD_ACK_READ_LIFT_DATAS, 0, (char*)mArmsDatas, sizeof(BASE::ARMS_R_USE_MSG)*DEF_SYS_MAX_ARMS_NUMS);
        printf("interaction CMD run start\n");
        break;
      }
      case BASE::CMD_CYC_READ_SHOWDE_DATAS:
      {
        printf("interaction CMD run start\n");
        break;
      }
      case BASE::CMD_RUN_STOP:
      {
        if(pTModule->mState == BASE::M_STATE_RUN)
        {
          pTModule->mNewState = BASE::M_STATE_CONF;
          pTModule->mIsStataChange = true;
          pTModule->mState = BASE::M_STATE_CONF;
        }

        sendMsgToSupr(pTModule, BASE::CMD_RUN_STOP);
        printf("all modules trun conf state...\n");
        break;
      }
      case BASE::CMD_RUN_STOP_E:
      {
        if(pTModule->mState == BASE::M_STATE_RUN)
            sendMsgToSupr(pTModule, BASE::CMD_RUN_STOP);
        printf("interaction CMD run  stop E\n");
        break;
      }
      case BASE::CMD_QUIT:
      {
        sendMsgToSupr(pTModule, BASE::CMD_QUIT);
        printf("interaction CMD quit... \n");
        break;
      }
      case BASE::CMD_READ_PLAYBACK:
      {
        int32_t *mTemp = (int32_t*)pTModule->mRecMsg.Datas;
        sendPlayBack(pTModule, *mTemp);
        printf("interaction CMD quit... \n");
        break;
      }
      default:
      {
        break;
      }
    }

    //每个循环周期结束后，将msg置空
    memset((char*)&(pTModule->mRecMsg), 0, sizeof(BASE::MArmsDownData));
  }

  free(mArmsDatas);
  moduleEndUp(pTModule);
}

} //namespace
