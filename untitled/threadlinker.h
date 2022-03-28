/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name : threadlinker.h
* Module :
* Create on: 2021/5/10
* Author: 师洋磊
* Email: 546783926@qq.com
* Description: 机械臂控制器连接模块，主要和控制器对接
*
********************************************************************************/
#ifndef THREADLINKER_H
#define THREADLINKER_H

#include <QThread>
#include <QVariant>
#include <QTimer>
#include<QUdpSocket>

#include "host_base_messages.h"
#include "basethread.h"

class ThreadLinker : public BaseThread
{
    Q_OBJECT
public:
    ThreadLinker(QObject *parent = nullptr);
    ~ThreadLinker();
signals:
    //传输notice到主界面
    //void signalViconWriteNoticeToWind(QVariant);
    void signalBeffDataToWind(QVariant);

public slots:
    void slotsHandleTimer();
    void recNoticeMessages(QVariant);

    void readDataGram();

public:
    void run();
private:
    int  init();
    int  initTimer();
    int  deinit();

    int recSignalLink(QString mServerIp, int mServerPort);
    int recSignalUnLink();
    int recSignalSaveConf(BASE::ConfData *pConfData);
    int recSignalCalibrate(BASE::CalibrateData *pData);
    int recSignalReadConf();
    int recSignalLiftHandMoveModule(BASE::LiftCmdData *pData);
    int recSignalLiftHandMoveStop(BASE::LiftCmdData *pData);
    int recSignalLiftAllMoveModule(BASE::LiftCmdData *pData);
    int recSignalLiftAllMoveStop();
    int recSignalLiftAlPullModule(BASE::LiftCmdData *pData);
    int recSignalLiftAllPullStop();
    int recSignalLiftCmdModule(int mCmd, BASE::LiftCmdData *pData);
    int recSignalConfSikoCmdModule(BASE::ConfData *pData);


    int recSignalRunStart(char* mData);
    int recSignalRunStop(const uint16_t pStopType);
    int recSignalRunPlayStart();


    void sendControlCmd(const uint16_t mCmd, const char* pData, const int pDataSize);


    void sendNoticeTowind(const int pNotice, const int pValue);
    void sendDataTowind(const int pNotice, const int pValue, char* pData);

    int readDataFromBeffAndSendToWind();
    void handleRecDatagrams(BASE::MArmsUpData &mMsg);

    //读取天车、机械臂发送 接收数据
    bool sysIsLinked;
    //是否显示系统hz 抖动事件
    bool isShowHz;

    //是否显示起吊界面下的当前手控下各个轴的位置
    bool isShowRunMsg;

    bool isShowRunPlayBackMsg;
    int32_t startPlayBackIndex1,startPlayBackIndex2;
    int32_t module1,module2;

    //设置定时器，定期的读取beff数据，并发送给显示线程。
    QTimer*  qMTimer;
    uint32_t mNowRandomCode,mLastRandomCode;
    int      lostLinkTimes;

    QUdpSocket * udpReceiver;
    char recData[1000];
    QString  mServerIp;
    quint16  mServerPort;

};

#endif // THREADLINKER_H
