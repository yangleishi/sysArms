/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Source File(*.cpp)
* File Name : basethread
* Module : XXX
* Create on : 2021/5/10
* Author : 师洋磊
*
* [Description about this source file]
* 该文件是线程基类，主要包含发送消息函数
*
********************************************************************************/

#include "basethread.h"
#include<QDebug>

/*****************************************************
* @param parent : [in]线程类继承Qobject类
* @param name2 : [in/out]Descriptions
* @param name3 : [in/out]Descriptions
* @return Descriptions
* @exception Type1: Descriptions
* @exception Type2: Descriptions
* @exception Type3: Descriptions
******************************************************/
BaseThread::BaseThread(QObject *parent) : QThread(parent)
{
    //这可以让发往 ThreadReadVicon 实例的信号,最终调用ThreadReadVicon的slot，并且是在ThreadReadVicon的线程循环中
    moveToThread(this);

    isLive = true;
    threadState = 0;
    kill = false;
}

///////共有的函数//////////////////////
/*****************************************************
* @param p_id : [in]发送线程的pTID
* @param p_RecId : [in]接收线程的tId
* @param pNotice : [in]发送通知消息号
* @param value : [in]发送消息值
* @param pData : [in]发送的数据Data指针
* @return Descriptions
* @exception Type1: Descriptions
* @exception Type2: Descriptions
* @exception Type3: Descriptions
* 描述：
*    该函数是线程消息发送函数。线程之间通信
******************************************************/
void BaseThread::sendNoticeTowind(const int p_id, const int p_RecId,  const int pNotice, const int value, char* pData)
{
    BASE::SigalMessages mMsg;
    mMsg.m_Sender = p_id;
    mMsg.m_Reciver = p_RecId;
    mMsg.m_Notice = pNotice;
    mMsg.m_Value  = value;
    mMsg.m_MsgPData = pData;
    QVariant varnt;
    varnt.setValue(mMsg);
    //发送消息
    emit signalNoticeToWind(varnt);
}

/********************************************************************************
* $log$
*
* Revision 1.1 2021/5/10 17:20:00 师洋磊
* [包含线程发送消息给主线程模块]
********************************************************************************/
