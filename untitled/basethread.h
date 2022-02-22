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
* Description: 线程基类，主要成员变量包括：线程状态，线程是否在运行等。成员函数：发送消息给主界面程序（主线程）
*
********************************************************************************/

#ifndef BASETHREAD_H
#define BASETHREAD_H

#include <QThread>
#include <QVariant>
#include "host_base_messages.h"

class BaseThread : public QThread
{
    Q_OBJECT
public:
    BaseThread(QObject *parent = nullptr);
    void sendNoticeTowind(const int p_id, const int p_RecId, const int pNotice, const int value, char* pData);

    bool kill;
    bool isLive;
protected:
    int  threadState;

signals:
void signalNoticeToWind(QVariant);

};

#endif // BASETHREAD_H
