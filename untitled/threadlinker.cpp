/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name : basethread.h
* Module : sendNoticeTowind
* Create on: 2021/5/10
* Author: 师洋磊
* Email: 546783926@qq.com
* Description: 连接机械臂控制器线程，
*
********************************************************************************/

#include "threadlinker.h"

#include<QDebug>
#include<QThread>
#include<QEventLoop>
#include <QTimerEvent>
#include<complex>


#define TIMER_TIMEOUT  (100)

/*****************************************************
* @param parent : [in]父窗体
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    线程构造函数
******************************************************/
ThreadLinker::ThreadLinker(QObject *parent) : BaseThread(parent)
{
    //这可以让发往 ThreadReadVicon 实例的信号,最终调用ThreadReadVicon的slot，并且是在ThreadReadVicon的线程循环中
    moveToThread(this);

    qMTimer = nullptr;

    sysIsLinked = false;
    isShowHz = false;
    isShowRunMsg = false;
    isShowRunPlayBackMsg = false;
    startPlayBackIndex1 = -1;
}

ThreadLinker::~ThreadLinker()
{

}

/*****************************************************
* @param mMsg : [in]mMsg是收到机械臂控制器消息
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    接收到机械臂控制器处理函数
******************************************************/
void ThreadLinker::handleRecDatagrams(BASE::MArmsUpData &mMsg)
{
    switch (mMsg.StatusWord)
    {
        case BASE::CMD_ACK_LINK_OK :{
            //将配置信息显示在界面
            sendNoticeTowind(BASE::MSG_NOTICE_ACK_LINK, 0);
            BASE::ARMS_R_USE_MSG *pShow = (BASE::ARMS_R_USE_MSG *)malloc(sizeof(BASE::ARMS_R_USE_MSG)*SYS_ARMS_MAX_SIZE);
            memcpy((char*)pShow, mMsg.Datas, sizeof(BASE::ARMS_R_USE_MSG)*SYS_ARMS_MAX_SIZE);
            sendDataTowind(BASE::MSG_NOTICE_ACK_RCONF, 0, (char*)pShow);
            sysIsLinked = true;
            isShowHz = true;
            break;
        }
        case BASE::CMD_ACK_LINK_FAILD :{
            break;
        }
        case BASE::CMD_ACK_UNLINK_OK :{
            //将配置信息显示在界面
            sendNoticeTowind(BASE::MSG_NOTICE_ACK_UNLINK, 0);
            sysIsLinked = false;
            break;
        }
        case BASE::CMD_ACK_READCONF_OK :{
            BASE::ConfData *pShow = (BASE::ConfData *)malloc(sizeof(BASE::ConfData)*SYS_ARMS_MAX_SIZE);
            memcpy((char*)pShow, mMsg.Datas, sizeof(BASE::ConfData)*SYS_ARMS_MAX_SIZE);
            sendDataTowind(BASE::MSG_NOTICE_ACK_RCONF, 0, (char*)pShow);
            break;
        }
        case BASE::CMD_ACK_READ_CYC :{
            BASE::ARMS_R_USE_MSG *pShow = (BASE::ARMS_R_USE_MSG *)malloc(sizeof(BASE::ARMS_R_USE_MSG)*SYS_ARMS_MAX_SIZE);
            memcpy((char*)pShow, mMsg.Datas, sizeof(BASE::ARMS_R_USE_MSG)*SYS_ARMS_MAX_SIZE);
            sendDataTowind(BASE::MSG_NOTICE_ACK_CYC, 0, (char*)pShow);
            break;
        }
        case BASE::CMD_ACK_READ_LIFT_DATAS :{
            BASE::ARMS_R_USE_MSG *pShow = (BASE::ARMS_R_USE_MSG *)malloc(sizeof(BASE::ARMS_R_USE_MSG)*SYS_ARMS_MAX_SIZE);
            memcpy((char*)pShow, mMsg.Datas, sizeof(BASE::ARMS_R_USE_MSG)*SYS_ARMS_MAX_SIZE);
            sendDataTowind(BASE::MSG_ACK_HAND_MOVE, 0, (char*)pShow);
            break;
        }
        case BASE::CMD_ACK_READ_DELAYED_DATAS :{
            //将配置信息显示在界面
            BASE::ARMS_R_USE_MSG *pShow = (BASE::ARMS_R_USE_MSG *)malloc(sizeof(BASE::ARMS_R_USE_MSG)*SYS_ARMS_MAX_SIZE);
            memcpy((char*)pShow, mMsg.Datas, sizeof(BASE::ARMS_R_USE_MSG)*SYS_ARMS_MAX_SIZE);
            sendDataTowind(BASE::MSG_ACK_HZ_SHAKE, 0, (char*)pShow);
            break;
        }
        case BASE::CMD_ACK_READ_RUNNING_DATAS :{
            //将配置信息显示在界面
            BASE::ARMS_R_USE_MSG *pShow = (BASE::ARMS_R_USE_MSG *)malloc(sizeof(BASE::ARMS_R_USE_MSG)*SYS_ARMS_MAX_SIZE);
            memcpy((char*)pShow, mMsg.Datas, sizeof(BASE::ARMS_R_USE_MSG)*SYS_ARMS_MAX_SIZE);
            sendDataTowind(BASE::MSG_ACK_RUN_SHOW, 0, (char*)pShow);
            break;
        }
        case BASE::CMD_ACK_READ_SHOWDE_DATAS :{
            break;
        }
        default: {
            break;
        }
    }
}

/*****************************************************
* @param  : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    接收到机械臂控制器消息，UDP
******************************************************/
void ThreadLinker::readDataGram()
{
    BASE::MArmsUpData mMsg;
    while(udpReceiver->hasPendingDatagrams())
    {
        memset((char*)&mMsg, 0, sizeof(BASE::MArmsUpData));
        qint64 recSize =  udpReceiver->readDatagram((char*)&mMsg, sizeof(BASE::MArmsUpData));
        if(recSize == sizeof(BASE::MArmsUpData))
        {
            mNowRandomCode = mMsg.mRandomCode;
            handleRecDatagrams(mMsg);
        }
    }
}
/////函数///////////////////
/*****************************************************
* @param  mCmd: [in]mCmd机械臂控制命令
* @param  pData: [in]pData发送给机械臂控制器数据
* @param  pDataSize: [in]发送给机械臂控制器数据的大小
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    发送控制命令给机械臂控制器
******************************************************/
void ThreadLinker::sendControlCmd(const uint16_t mCmd, const char* pData, const int pDataSize)
{
    BASE::MArmsDownData mMsg;
    mMsg.CmdIdentify = mCmd;
    if(pDataSize>0)
        memcpy(mMsg.Datas, pData, pDataSize);

    udpReceiver->writeDatagram((char*)&mMsg, sizeof(mMsg), QHostAddress(mServerIp), mServerPort);
}

/*****************************************************
* @param  mServerIp: [in]mServerIp机械臂控制命令IP
* @param  mServerPort: [in]mServerPort机械臂控制器端口
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    接收连接控制器命令，并下发
******************************************************/
int ThreadLinker::recSignalLink(QString mServerIp, int mServerPort)
{
    //TODU 发送请求连接服务器命令
    this->mServerIp = mServerIp;
    this->mServerPort = (quint16)mServerPort;
    if(!sysIsLinked)
        sendControlCmd(BASE::CMD_LINK, 0, 0);

    qDebug() << "ip:"<<mServerIp;
    return 0;
}

/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    断开接收连接控制器命令，并下发
******************************************************/
int ThreadLinker::recSignalUnLink()
{
    //TODU 发送请求断开连接服务器命令
    if(sysIsLinked)
        sendControlCmd(BASE::CMD_UNLINK, 0, 0);

    isShowHz = false;
    isShowRunMsg = false;
    isShowRunPlayBackMsg = false;
    return 0;
}

/*****************************************************
* @param pConfData: [in]pConfData配置数据
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    保存配置机械臂命令
******************************************************/
int ThreadLinker::recSignalSaveConf(BASE::ConfData *pConfData)
{
    //拷贝下发控制命令数据
    if(sysIsLinked)
        sendControlCmd(BASE::CMD_SAVE_CONF, (char*)pConfData, sizeof(BASE::ConfData)*SYS_ARMS_MAX_SIZE);

    for(int i=0; i<SYS_ARMS_MAX_SIZE; i++)
    {
        if(pConfData[i].mIsValid)
        {
            ;
        }
    }
    return 0;
}

int ThreadLinker::recSignalCalibrate(BASE::CalibrateData *pData)
{
    //拷贝下发控制命令数据
    if(sysIsLinked)
        sendControlCmd(BASE::CMD_CALIBRATE_ARM, (char*)pData, sizeof(BASE::CalibrateData)*SYS_ARMS_MAX_SIZE);

    return 0;
}
/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    读取配置机械臂命令
******************************************************/
int ThreadLinker::recSignalReadConf()
{
    //拷贝下发控制命令数据
    if(sysIsLinked)
        sendControlCmd(BASE::CMD_READ_CONF, 0, 0);

    qDebug() << "read conf***********";
    return 0;
}

/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    读取配置机械臂命令
******************************************************/
int ThreadLinker::recSignalLiftHandMoveModule(BASE::LiftCmdData *pData)
{
    //拷贝下发控制命令数据
    if(sysIsLinked)
        sendControlCmd(BASE::CMD_HAND_MOVE_START, (char*)pData, sizeof(BASE::LiftCmdData));

    return 0;
}

/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    接收手动移动停止命令，下发给机械臂控制器
******************************************************/
int ThreadLinker::recSignalLiftHandMoveStop(BASE::LiftCmdData *pData)
{
    //拷贝下发控制命令数据
    if(sysIsLinked)
        sendControlCmd(BASE::CMD_HAND_MOVE_STOP,  (char*)pData, sizeof(BASE::LiftCmdData));

    qDebug() << "move stop";
    return 0;
}

/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    接收手动拉力开始命令，下发给机械臂控制器
******************************************************/
int ThreadLinker::recSignalLiftAllMoveModule(BASE::LiftCmdData *pData)
{
    //拷贝下发控制命令数据
    if(sysIsLinked)
        sendControlCmd(BASE::CMD_ALL_MOVE_START, (char*)pData, sizeof(BASE::LiftCmdData)*SYS_ARMS_MAX_SIZE);
}

/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    接收手动拉力停止命令，下发给机械臂控制器
******************************************************/
int ThreadLinker::recSignalLiftAllMoveStop()
{
    //拷贝下发控制命令数据
    if(sysIsLinked)
        sendControlCmd(BASE::CMD_ALL_MOVE_STOP, 0, 0);
    qDebug() << "move stop";
    return 0;
}

/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    接收手动拉力开始命令，下发给机械臂控制器
******************************************************/
int ThreadLinker::recSignalLiftAlPullModule(BASE::LiftCmdData *pData)
{
    //拷贝下发控制命令数据
    if(sysIsLinked)
        sendControlCmd(BASE::CMD_ALL_PULL_START, (char*)pData, sizeof(BASE::LiftCmdData)*SYS_ARMS_MAX_SIZE);

}

/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    接收手动拉力停止命令，下发给机械臂控制器
******************************************************/
int ThreadLinker::recSignalLiftAllPullStop()
{
    //拷贝下发控制命令数据
    if(sysIsLinked)
        sendControlCmd(BASE::CMD_ALL_PULL_STOP, 0, 0);

    qDebug() << "pull stop";
    return 0;
}

/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    接收运行界面开始，下发给机械臂控制器
******************************************************/
int ThreadLinker::recSignalRunStart(char* mData)
{
    //拷贝下发控制命令数据
    if(sysIsLinked)
        sendControlCmd(BASE::CMD_RUN_START, mData, sizeof(int)*SYS_ARMS_MAX_SIZE);
    qDebug() << "run start";
    isShowRunMsg = true;
    return 0;
}

int ThreadLinker::recSignalRunPlayStart()
{
    //拷贝下发控制命令数据
    if(sysIsLinked)
        isShowRunPlayBackMsg = true;

    qDebug() << "playback run start";
    return 0;
}

/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    接收运行界面停止，下发给机械臂控制器
******************************************************/
int ThreadLinker::recSignalRunStop(const uint16_t pStopType)
{
    //拷贝下发控制命令数据
    if(sysIsLinked)
        sendControlCmd(pStopType, 0, 0);

    qDebug() << "pull stop";
    isShowRunMsg = false;
    return 0;
}

/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*   定时器，定期请求显示内容
******************************************************/
void ThreadLinker::slotsHandleTimer()
{

    //TUDO  定期发送手控界面下个轴当前pos
    //if(isShowRunMsg)
       // sendControlCmd(BASE::CMD_CYC_READ_RUNNING_DATAS, 0, 0);

    //TUDO 暂且
    if(isShowRunPlayBackMsg)
    {
        sendControlCmd(BASE::CMD_READ_PLAYBACK, (char*)&startPlayBackIndex1, sizeof(startPlayBackIndex1));
        startPlayBackIndex1++;
    }
    //定期请求上传数据
    if(sysIsLinked)
        sendControlCmd(BASE::CMD_READ_CYC, 0, 0);

    //判断随机码是否跟新
    /*
    if(sysIsLinked)
    {
        if(mLastRandomCode != (mNowRandomCode-1))
        {
            qDebug()<<mLastRandomCode<<mNowRandomCode;
            lostLinkTimes++;
            mLastRandomCode = mNowRandomCode;

        }else
        {
            lostLinkTimes = 0;
        }

        if(lostLinkTimes >= 12)
        {
            sysIsLinked = false;
            sendNoticeTowind(BASE::MSG_NOTICE_ACK_UNLINK, 0);
        }

    }
    */

}

/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*   线程主循环
******************************************************/
void ThreadLinker::run()
{
    //windows下线程绑定函数
    //SetThreadAffinityMask(GetCurrentThread(), 1);
    init();
    threadState = 0;
    QEventLoop eventLoop(this);

    qDebug()<< currentThread();

    //TUDO 刚启动就显示一次配置信息。（当连接上控制器时候，主动请求配置信息）
    recSignalReadConf();
    //线程主循环体
    while (isLive)
    {
      eventLoop.processEvents();

      if(sysIsLinked)
      {
          //readDataFromBeffAndSendToWind();
      }
      usleep(1000);
    }

    //释放资源
    deinit();
    //通知主线程，已经正常死亡
    kill = true;
    qDebug()<<"kill readbeff";
    return ;
}

/////////////私有成员函数////////////////////////////////////
////TUDO  连接beff控制器
/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*   初始化，
******************************************************/
int ThreadLinker::init()
{
    //启动定时器
    initTimer();

    //初始化udp
    udpReceiver = new QUdpSocket(this);
    if(nullptr == udpReceiver)
        qDebug() << "udp socket 创建失败!";

    udpReceiver->bind(6000, QUdpSocket::ShareAddress);
    connect(udpReceiver, SIGNAL(readyRead()), this, SLOT(readDataGram()));
    return 0;
}

/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*   de初始化
******************************************************/
int  ThreadLinker::deinit()
{   
    int iRet = 0;
    if(nullptr != qMTimer)
    {
        delete qMTimer;
        qMTimer = nullptr;
    }
    return iRet;
}

/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*   初始化定时器
******************************************************/
int  ThreadLinker::initTimer()
{
    if(qMTimer != nullptr)
        delete qMTimer;

    //启动定时器
    qMTimer = new QTimer();
    connect(qMTimer, SIGNAL(timeout()), this, SLOT(slotsHandleTimer()));
    qMTimer->start(TIMER_TIMEOUT);
    return 0;
}

/*****************************************************
* @param pNotice: [in]pNotice通知号
* @param pValue: [in]pValue消息值
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*   发送消息到主窗体
******************************************************/
void ThreadLinker::sendNoticeTowind(const int pNotice, const int pValue)
{
    //发送通知消息
    BaseThread::sendNoticeTowind(BASE::THREAD_ID_LINKER, BASE::THREAD_ID_WIND, pNotice, pValue, 0);
}

/*****************************************************
* @param pNotice: [in]pNotice通知号
* @param pValue: [in]pValue消息值
* @param pData: [in]pData数据
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*   发送给主窗体数据
******************************************************/
void ThreadLinker::sendDataTowind(const int pNotice, const int pValue, char* pData)
{
    //发送数据消息
    BaseThread::sendNoticeTowind(BASE::THREAD_ID_LINKER, BASE::THREAD_ID_WIND, pNotice, pValue, pData);
}

int ThreadLinker::recSignalLiftCmdModule(int mCmd, BASE::LiftCmdData *pData)
{
    switch (mCmd)
    {
        case BASE::MSG_NOTICE_HAND_MOVE :{
            //拷贝下发控制命令数据
            if(sysIsLinked)
                sendControlCmd(BASE::CMD_HAND_MOVE_START, (char*)pData, sizeof(BASE::LiftCmdData));
            break;
        }
        case BASE::MSG_NOTICE_ALL_MOVE :{
            //拷贝下发控制命令数据
            if(sysIsLinked)
                sendControlCmd(BASE::CMD_ALL_MOVE_START, (char*)pData, sizeof(BASE::LiftCmdData)*SYS_ARMS_MAX_SIZE);
            break;
        }
        case BASE::MSG_NOTICE_ALL_PULL :{
            //拷贝下发控制命令数据
            if(sysIsLinked)
                sendControlCmd(BASE::CMD_ALL_PULL_START, (char*)pData, sizeof(BASE::LiftCmdData)*SYS_ARMS_MAX_SIZE);
            break;
        }
        case BASE::MSG_NOTICE_HAND_MOVE_STOP :{
            if(sysIsLinked)
                sendControlCmd(BASE::CMD_HAND_MOVE_STOP,  (char*)pData, sizeof(BASE::LiftCmdData));
            break;
        }
        case BASE::MSG_NOTICE_ALL_MOVE_STOP :{
            if(sysIsLinked)
                sendControlCmd(BASE::CMD_ALL_MOVE_STOP, 0, 0);
            break;
        }
        case BASE::MSG_NOTICE_ALL_PULL_STOP :{
            if(sysIsLinked)
                sendControlCmd(BASE::CMD_ALL_PULL_STOP, 0, 0);
            break;
        }
        default: {
            break;
        }
    }
}

//接收所有线程的notice消息
/*****************************************************
* @param mNotice: [in]mNotice消息
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*   接收主窗体发来的消息
******************************************************/
void ThreadLinker::recNoticeMessages(QVariant mNotice)
{
    BASE::SigalMessages mMsg = mNotice.value<BASE::SigalMessages>();
    //确认是自己的消息
    if(mMsg.m_Reciver == BASE::THREAD_ID_LINKER)
    {
        switch (mMsg.m_Notice)
        {
            case BASE::MSG_NOTICE_LINK :{
                recSignalLink(*((QString*)mMsg.m_MsgPData), mMsg.m_Value);
                break;
            }
            case BASE::MSG_NOTICE_UNLINK :{
                recSignalUnLink();
                break;
            }
            case BASE::MSG_NOTICE_SCONF :{
                //m_MsgPData存放的是confdata的地址
                recSignalSaveConf((BASE::ConfData *)mMsg.m_MsgPData);
                free(mMsg.m_MsgPData);
                break;
            }
            case BASE::MSG_NOTICE_CALIBRATE :{
              //m_MsgPData存放的是confdata的地址
              recSignalCalibrate((BASE::CalibrateData *)mMsg.m_MsgPData);
              free(mMsg.m_MsgPData);
              break;
            }
            case BASE::MSG_NOTICE_RCONF :{
                //Value存放的是confdata的地址
                recSignalReadConf();
                break;
            }
            case BASE::MSG_NOTICE_HZ_SHAKE :{
                //修改显示标志，定时器自动显示发送
                isShowHz = true;
                break;
            }
            case BASE::MSG_NOTICE_HAND_MOVE :
            case BASE::MSG_NOTICE_ALL_MOVE:
            case BASE::MSG_NOTICE_ALL_PULL :
            case BASE::MSG_NOTICE_HAND_MOVE_STOP :{
                //请求移动
                recSignalLiftCmdModule(mMsg.m_Notice, (BASE::LiftCmdData*)mMsg.m_MsgPData);
                free(mMsg.m_MsgPData);
                break;
            }
            case BASE::MSG_NOTICE_ALL_MOVE_STOP :
            case BASE::MSG_NOTICE_ALL_PULL_STOP :{
                //请求移动
                recSignalLiftCmdModule(mMsg.m_Notice, 0);
                break;
            }
            case BASE::MSG_NOTICE_RUN_START :{
                //运行界面下 开始
                recSignalRunStart(mMsg.m_MsgPData);
                qDebug()<<"开始";
                break;
            }
            case BASE::MSG_NOTICE_RUN_PLAY :{
              //运行界面下 回放
              startPlayBackIndex1 = mMsg.m_Value;
              recSignalRunPlayStart();
              qDebug()<<"开始";
              break;
            }
            case BASE::MSG_NOTICE_RUN_PLAY_STOP :{
              //运行界面下 回放
              isShowRunPlayBackMsg = false;
              startPlayBackIndex1 = -1;
              qDebug()<<"停止";
              break;
            }
            case BASE::MSG_NOTICE_RUN_STOP :{
                //运行界面下 停止
                recSignalRunStop(BASE::CMD_RUN_STOP);
                qDebug()<<"停止";
                break;
            }
            case BASE::MSG_NOTICE_RUN_STOPE :{
                //运行界面下 急停
                recSignalRunStop(BASE::CMD_RUN_STOP_E);
                qDebug()<<"急停";
                break;
            }
            case BASE::MSG_NOTICE_KILL_ALL : {
                sendControlCmd(BASE::CMD_QIUT, 0, 0);
                isLive = false;
                qDebug()<<"kill";
                break;
            }
            default: {
                break;
            }
        }
    }
}
