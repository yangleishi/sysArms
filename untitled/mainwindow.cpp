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
* Description: 界面显示主线程，
*
********************************************************************************/

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QPushButton>
#include <QDebug>
#include <QMetaType>
#include <QDateTime>
#include <synchapi.h>
#include<thread>
#include<string>

#include <stdio.h>
#include <process.h>

#include "host_base_messages.h"

#define TIMER_TIMEOUT   (1000)

/*****************************************************
* @param parent : [in]父窗体
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    窗体构造函数
******************************************************/
MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    setUi();

    initBase();
}

void MainWindow::setUi()
{
    mLinkIp = new QLineEdit(ui->centralWidget);
    mLinkIp->setGeometry(QRect(120, 850, 131, 21));
    mLinkIp->raise();
    mLinkIp->setText(QString("192.168.1.100"));
    mLinkPort = new QLineEdit(ui->centralWidget);
    mLinkPort->setGeometry(QRect(360, 850, 51, 21));
    mLinkPort->raise();
    mLinkPort->setText(QString("10001"));


    mLiftHandCheckBoxModule = new QComboBox(ui->tab2);
    mLiftHandCheckBoxModule->setGeometry(QRect(30, 680, 81, 22));

    for(int i=0; i<SYS_ARMS_MAX_SIZE; i++)
    {
        mConfReadSikoX[i] = new QLineEdit(ui->tab2);
        mConfReadSikoX[i]->setGeometry(BASE::LineConfReadSikoXRect[i]);
        mConfReadSikoX[i]->raise();
        mConfReadSikoX[i]->setText("null");
        mConfReadSikoX[i]->setEnabled(false);
        mConfReadSikoY[i] = new QLineEdit(ui->tab2);
        mConfReadSikoY[i]->setGeometry(BASE::LineConfReadSikoYRect[i]);
        mConfReadSikoY[i]->raise();
        mConfReadSikoY[i]->setText("null");
        mConfReadSikoY[i]->setEnabled(false);

        mConfReadLevelX[i] = new QLineEdit(ui->tab2);
        mConfReadLevelX[i]->setGeometry(BASE::LineConfReadLevelXRect[i]);
        mConfReadLevelX[i]->raise();
        mConfReadLevelX[i]->setText("null");
        mConfReadLevelX[i]->setEnabled(false);
        mConfReadLevelY[i] = new QLineEdit(ui->tab2);
        mConfReadLevelY[i]->setGeometry(BASE::LineConfReadLevelYRect[i]);
        mConfReadLevelY[i]->raise();
        mConfReadLevelY[i]->setText("null");
        mConfReadLevelY[i]->setEnabled(false);

        /*****************************起吊界面控件初始化********************************/
        mLiftRadioButton[i] = new QRadioButton(ui->tab2);
        mLiftRadioButton[i]->setEnabled(false);
        mLiftRadioButton[i]->setGeometry(BASE::LiftRect[i]);
        mLiftRadioButton[i]->raise();
        mLiftRadioButton[i]->setText(BASE::ModulesStrings[i]);
        mLiftRadioButton[i]->setStyleSheet("QRadioButton::indicator{width: 17px;height: 17px;border-radius:7px;}"
                                           "QRadioButton::indicator:checked{background-color: green;}"
                                           "QRadioButton::indicator:unchecked{background-color: red;}"
                                           );


        //限位开关状态
        mESwitch[i] = new QLineEdit(ui->tab2);
        mESwitch[i]->setEnabled(true);
        mESwitch[i]->setGeometry(BASE::switchRect[i]);
        mESwitch[i]->raise();

        mESwitch[i]->setStyleSheet("QRadioButton::indicator{width: 17px;height: 17px;border-radius:7px;}"
                                           "QRadioButton::indicator:checked{background-color: red;}"
                                           "QRadioButton::indicator:unchecked{background-color: green;}"
                                           );

        mLiftHandXNow[i] = new QLineEdit(ui->tab2);
        mLiftHandXNow[i]->setGeometry(BASE::LineLiftReadHandXNowRect[i]);
        mLiftHandXNow[i]->raise();
        mLiftHandXNow[i]->setText("0");
        mLiftHandXNow[i]->setEnabled(false);

        mLiftHandYNow[i] = new QLineEdit(ui->tab2);
        mLiftHandYNow[i]->setGeometry(BASE::LineLiftReadHandYNowRect[i]);
        mLiftHandYNow[i]->raise();
        mLiftHandYNow[i]->setText("0");
        mLiftHandYNow[i]->setEnabled(false);


        mLiftHandZNow[i] = new QLineEdit(ui->tab2);
        mLiftHandZNow[i]->setGeometry(BASE::LineLiftReadHandZNowRect[i]);
        mLiftHandZNow[i]->raise();
        mLiftHandZNow[i]->setText("0");
        mLiftHandZNow[i]->setEnabled(false);


        mLiftHandWNow[i] = new QLineEdit(ui->tab2);
        mLiftHandWNow[i]->setGeometry(BASE::LineLiftReadHandWNowRect[i]);
        mLiftHandWNow[i]->raise();
        mLiftHandWNow[i]->setText("0");
        mLiftHandWNow[i]->setEnabled(false);


        //整体提升多选框
        mConfReadPull[i] = new QLineEdit(ui->tab2);
        mConfReadPull[i]->setGeometry(BASE::LineConfReadPullRect[i]);
        mConfReadPull[i]->raise();
        mConfReadPull[i]->setText("null");
        mConfReadPull[i]->setEnabled(false);

        //板转编码器
        mConfReadEncoderT[i] = new QLineEdit(ui->tab2);
        mConfReadEncoderT[i]->setGeometry(BASE::LineConfReadEncoderTRect[i]);
        mConfReadEncoderT[i]->raise();
        mConfReadEncoderT[i]->setText("null");
        mConfReadEncoderT[i]->setEnabled(false);

        mLiftHandCheckBoxModule->addItem(BASE::ModulesStrings[i]);

        mLiftCheckBoxs[i] = new QCheckBox(ui->tab2);
        mLiftCheckBoxs[i]->setGeometry(BASE::LiftAllCheckBoxRect[i]);
        mLiftCheckBoxs[i]->raise();
        mLiftCheckBoxs[i]->setText(BASE::ModulesStrings[i]);

        mRunCheckBoxs[i] = new QCheckBox(ui->tab2);
        mRunCheckBoxs[i]->setGeometry(BASE::RunAllCheckBoxRect[i]);
        mRunCheckBoxs[i]->raise();
        mRunCheckBoxs[i]->setText(BASE::ModulesStrings[i]);


        /*****************************运行界面控件初始化********************************/

        mRunReadX[i] = new QLineEdit(ui->tab2);
        mRunReadX[i]->setGeometry(BASE::LineRunPosXRect[i]);
        mRunReadX[i]->raise();
        mRunReadX[i]->setEnabled(false);
        mRunReadX[i]->setText("0");
        mRunReadY[i] = new QLineEdit(ui->tab2);
        mRunReadY[i]->setGeometry(BASE::LineRunPosYRect[i]);
        mRunReadY[i]->raise();
        mRunReadY[i]->setEnabled(false);
        mRunReadY[i]->setText("0");
        mRunReadZ[i] = new QLineEdit(ui->tab2);
        mRunReadZ[i]->setGeometry(BASE::LineRunPosZRect[i]);
        mRunReadZ[i]->raise();
        mRunReadZ[i]->setEnabled(false);
        mRunReadZ[i]->setText("0");

        mRunReadAngleX[i] = new QLineEdit(ui->tab2);
        mRunReadAngleX[i]->setGeometry(BASE::LineRunErrorAngleXRect[i]);
        mRunReadAngleX[i]->raise();
        mRunReadAngleX[i]->setEnabled(false);
        mRunReadAngleX[i]->setText("0");
        mRunReadAngleY[i] = new QLineEdit(ui->tab2);
        mRunReadAngleY[i]->setGeometry(BASE::LineRunErrorAngleYRect[i]);
        mRunReadAngleY[i]->raise();
        mRunReadAngleY[i]->setEnabled(false);
        mRunReadAngleY[i]->setText("0");
        mRunReadPullE[i] = new QLineEdit(ui->tab2);
        mRunReadPullE[i]->setGeometry(BASE::LineRunErrorPullRect[i]);
        mRunReadPullE[i]->raise();
        mRunReadPullE[i]->setEnabled(false);
        mRunReadPullE[i]->setText("0");

    }
    mLiftHandCheckBoxModule->raise();

    mLiftRadioButton[0]->setChecked(true);

}

/*****************************************************
* @param  : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    窗体初始化函数，线程启动，界面上控件绑定等
******************************************************/
void MainWindow::initBase()
{
    m_ThreadLinker           = new ThreadLinker();
    m_ThreadLinkerKill         = false;

    signalsSlotsConnects();
    m_ThreadLinker->start();

//初始化线程状态信息
    gMThreadInfo = (BASE::MODULEINFOS*)malloc(sizeof(BASE::MODULEINFOS) * (BASE::THREAD_MAX_ID+1));
    for(int i=BASE::THREAD_ID_WIND; i<=BASE::THREAD_MAX_ID; i++)
    {
        gMThreadInfo[i].mThreadId = i;
        gMThreadInfo[i].mCurRState = BASE::_MODULE_INIT;
    }

    qMTimer = nullptr;

//界面上的所有空间绑定成数组
    initControlBinding();

    mDetailCbChoosSize1 = 0;
    //创建绘制进程
    mDrawP = new QProcess(this);
    m_sender = new  QUdpSocket();
    m_sender->bind(9998, QUdpSocket::ShareAddress);

    //数据下传
    m_MulticastSend = new QUdpSocket();
    m_MulticastSend->bind(9998, QUdpSocket::ShareAddress);

    memset((char*)&mDrawDatas, 0, sizeof(mDrawDatas));
}

/*****************************************************
* @param  : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*    窗体删除初始化
******************************************************/
void MainWindow::deInitBase()
{
    DELETE_P(mLinkIp);
    DELETE_P(mLinkPort);

    for (int i = 0; i<SYS_ARMS_MAX_SIZE; ++i)
    {

        DELETE_P(mConfReadPull[i]);
        DELETE_P(mConfReadEncoderT[i]);
        DELETE_P(mConfReadSikoX[i]);
        DELETE_P(mConfReadSikoY[i]);
        DELETE_P(mConfReadLevelX[i]);
        DELETE_P(mConfReadLevelY[i]);

        DELETE_P(mLiftRadioButton[i]);

        DELETE_P(mLiftHandXNow[i]);
        DELETE_P(mLiftHandYNow[i]);
        DELETE_P(mLiftHandZNow[i]);
        DELETE_P(mLiftHandWNow[i]);
        DELETE_P(mLiftCheckBoxs[i]);

        DELETE_P(mRunReadX[i]);
        DELETE_P(mRunReadY[i]);
        DELETE_P(mRunReadZ[i]);
        DELETE_P(mRunReadAngleX[i]);
        DELETE_P(mRunReadAngleY[i]);
        DELETE_P(mRunReadPullE[i]);
    }

    for (int j = 0; j < SHOW_DETAILS_MUD_MUNS; ++j)
    {
      DELETE_P(widShowCB[j]);
    }


    if(mDrawP != nullptr)
    {
        mDrawP->close();
        delete mDrawP;
        mDrawP = nullptr;
    }
    if(nullptr != m_sender)
    {
        delete m_sender;
        m_sender = nullptr;
    }

    if(nullptr != m_MulticastSend)
    {
        delete m_MulticastSend;
        m_MulticastSend = nullptr;
    }
}


/*****************************************************
* @param baseString : [in]待生成的basestring
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     显示框等控件生产对应字符串
******************************************************/
QStringList MainWindow::createControlName(QString baseString)
{
    QStringList sRet;
    for(int i=0; i<SYS_ARMS_MAX_SIZE; i++)
    {
        QString num = QString::number(i+1, 10);
        sRet.push_back(baseString+num);
    }
    return sRet;
}

/*****************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     将界面中的控件绑定到数组中
******************************************************/
void MainWindow::initControlBinding()
{
    /**********************************细节显示checkbox初始化**********************************/
    for(int i=0; i<SHOW_DETAILS_MUD_MUNS; i++)
    {
        widShowCB[i] = new QCheckBox(ui->tab4);
        widShowCB[i]->setGeometry(BASE::CheckBoxDetail1Rect[i]);
        widShowCB[i]->raise();
        widShowCB[i]->setText(BASE::labelDetailStrings[i]);
    }


    //将多选按钮状态改变事件，确保只能显示5个曲线
    for (int j=0; j<SHOW_DETAILS_MUD_MUNS; j++)
    {
        connect(widShowCB[j], SIGNAL(stateChanged(int)), this, SLOT(detailCb1StateChanged(int)));
    }
}

/*****************************************************
* @param mValue: [in]mValue是link线程发来的连接消息ACK
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     线程发来的连接消息ACK，如果连接机械臂控制器成功，则设置
*     对应的按钮状态
******************************************************/
void MainWindow::handleLinkAck(int mValue)
{
    //link成功，设置对应的按钮逻辑
    if(mValue == 0)
    {
        ui->pb_read_conf->setEnabled(true);
        ui->pb_save_conf->setEnabled(true);

        ui->pb_link->setEnabled(false);
        ui->pb_unlink->setEnabled(true);

        ui->liftPB_allM_start->setEnabled(true);
        ui->liftPB_allP_start->setEnabled(true);

        //读取一次配置
        sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_RCONF, 0, 0);
    }
}

/**********************************************************
* @param mValue: [in]mValue是link线程发来的连接消息ACK
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     线程发来的断开连接消息ACK，如果断开连接机械臂控制器成功，则设置
*     对应的按钮状态
***********************************************************/
void MainWindow::handleUnLinkAck(int mValue)
{
    //unlink成功，设置对应的按钮逻辑
    if(mValue == 0)
    {
        ui->pb_read_conf->setEnabled(false);
        ui->pb_save_conf->setEnabled(false);

        ui->pb_link->setEnabled(true);
        ui->pb_unlink->setEnabled(false);
    }
}

/**********************************************************
* @param mNotice: [in]mNotice是消息
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     接收其他线程的消息函数。并作出对应的动作
***********************************************************/
void MainWindow::recNoticeMessages(QVariant mNotice)
{
    BASE::SigalMessages mMsg = mNotice.value<BASE::SigalMessages>();
    //修改线程状态信息
    switch (mMsg.m_Notice)
    {
        case BASE::MSG_NOTICE_ACK_LINK :
        {
            handleLinkAck(mMsg.m_Value);
            break;
        }
        case BASE::MSG_NOTICE_ACK_UNLINK :
        {
            handleUnLinkAck(mMsg.m_Value);
            break;
        }
        case BASE::MSG_NOTICE_ACK_CYC :
        {
            showCycMessage((BASE::ARMS_R_USE_MSG *)mMsg.m_MsgPData);
            //showRunMessage(mMsg.m_MsgPData);
            break;
        }
        case BASE::MSG_NOTICE_INIT_ALL_ACK :
        {
            if(mMsg.m_Value == BASE::MSG_VALUE_INIT_OK)
                gMThreadInfo[mMsg.m_Sender].mCurRState = BASE::_MODULE_INIT_OK;
            break;
        }
        case BASE::MSG_NOTICE_CONF_ALL_ACK :
        {
            if(mMsg.m_Value == BASE::MSG_VALUE_CONF_OK)
                gMThreadInfo[mMsg.m_Sender].mCurRState = BASE::_MODULE_CONF_OK;
            break;
        }
        case BASE::MSG_NOTICE_ACK_RCONF :
        {
            if(mMsg.m_Value == 0)
                showConfMessage((BASE::ConfData *)mMsg.m_MsgPData);
            free(mMsg.m_MsgPData);
            break;
        }
        case BASE::MSG_ACK_HZ_SHAKE :
        {
            showLiftMessage(BASE::MSG_ACK_HZ_SHAKE, mMsg.m_MsgPData);
            free(mMsg.m_MsgPData);
            break;
        }
        case BASE::MSG_ACK_HAND_MOVE :
        {
            showLiftMessage(BASE::MSG_ACK_HAND_MOVE, mMsg.m_MsgPData);
            free(mMsg.m_MsgPData);
            break;
        }
        case BASE::MSG_ACK_RUN_SHOW :
        {
            showRunMessage(mMsg.m_MsgPData);
            free(mMsg.m_MsgPData);
            break;
        }
        case BASE::MSG_NOTICE_CURVEWID_CLOSE :
        {
            /*qDebug()<<"close wid:"<<mMsg.m_Value;
            widShow1CB[mMsg.m_Value/100][mMsg.m_Value%100]->setChecked(false);
            widShow1CB[mMsg.m_Value/100][mMsg.m_Value%100]->setEnabled(true);

            if(widShow1_3[mMsg.m_Value/100][mMsg.m_Value%100] != nullptr)
            {
                delete widShow1_3[mMsg.m_Value/100][mMsg.m_Value%100];
                widShow1_3[mMsg.m_Value/100][mMsg.m_Value%100] = nullptr;
            }
            */
            break;
        }
        case BASE::MSG_NOTICE_RUN_ALL_ACK :
        {
            if(mMsg.m_Value == BASE::MSG_VALUE_RUN_OK)
            {
                gMThreadInfo[mMsg.m_Sender].mCurRState = BASE::_MODULE_RUNNING;
            }
            break;
        }
        case BASE::MSG_NOTICE_STOP_ALL_ACK :{
            if(mMsg.m_Value == BASE::MSG_VALUE_STOP_OK)
            {
                gMThreadInfo[mMsg.m_Sender].mCurRState = BASE::_MODULE_STOP;
            }
            break;
        }
        default: {
            break;
        }
    }
}

/**********************************************************
* @param sIp: [in]sIp是要link机械臂控制器的IP
* @param port: [in]port是要link机械臂控制器的port
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     接收按钮连接动作，发送给link线程连接机械臂控制器
***********************************************************/
void MainWindow::sendNoticeLink(QString sIp, int port)
{
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_LINK, port, sIp.toLatin1().data());
}

/**********************************************************
* @param mRecId: [in]mRecId接收线程的id
* @param pNotice: [in]pNotice是发送给接收线程的notice号
* @param pValue: [in]pValue是发送给接收线性的消息值
* @param pData: [in]pData是发送给接收线性的数据data
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     发送消息底层函数
***********************************************************/
void MainWindow::sendNotice(const int mRecId, const int pNotice, const int pValue, char *pData)
{
    BASE::SigalMessages mMsg;
    mMsg.m_Sender  = BASE::THREAD_ID_WIND;
    mMsg.m_Reciver = mRecId;
    mMsg.m_Notice  = pNotice;
    mMsg.m_Value   = pValue;
    mMsg.m_MsgPData = pData;

    QVariant varnt;
    varnt.setValue(mMsg);
    //发送消息
    switch (mRecId)
    {
        case BASE::THREAD_ID_DRAWCURVE :{
            emit signalNoticeToDrawCurve(varnt);
            break;
        }
        case BASE::THREAD_ID_LINKER :{
            emit signalNoticeToLinker(varnt);
            break;
        }
        case BASE::THREAD_ID_CURVEWIND :{
            emit signalDrawCurve(varnt);
            break;
        }
        case BASE::THREAD_ID_CONFBEFF :{
            emit signalNoticeToConfBeff(varnt);
            break;
        }
        default: {
            break;
        }
    }
}

/**********************************************************
* @param pNotice: [in]pNotice是发送给接收线程的notice号
* @param pValue: [in]pValue是发送给接收线性的消息值
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     通知消息，通知所有线程
***********************************************************/
void MainWindow::sendNoticeToAll(const int pNotice, const int pValue)
{
    for (int i=BASE::THREAD_ID_WIND+1; i<=BASE::THREAD_MAX_ID; i++)
    {
        sendNotice(i, pNotice, pValue, 0);
    }
}

/**********************************************************
* @param mCheckState: [in]mCheckState是线程状态
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     检查线程模块状态是否是给定状态
***********************************************************/
int MainWindow::checkAllThreadStates(BASE::_MODULE_RUN_STATE mCheckState)
{
    int iRet = 1;
    for (int i=BASE::THREAD_ID_WIND+1; i<=BASE::THREAD_MAX_ID; i++)
    {
        if(gMThreadInfo[i].mCurRState != mCheckState)
        {
            iRet = 0;
            break;
        }
    }
    return iRet;
}

///线程之间的信号和槽之间的连接关系
/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     窗体中信号和槽连接函数
***********************************************************/
void MainWindow::signalsSlotsConnects()
{
    /////////////////////***********所有线程线程消息函数**********//////////////////////
    //连接按钮做的动作
    connect(ui->pb_link, SIGNAL(clicked()), this, SLOT(slotsButtonLink()), Qt::QueuedConnection);
    connect(ui->pb_unlink, SIGNAL(clicked()), this, SLOT(slotsButtonUnLink()), Qt::QueuedConnection);

    //配置界面按钮做的动作
    connect(ui->pb_save_conf, SIGNAL(clicked()), this, SLOT(slotsButtonSaveConf()), Qt::QueuedConnection);
    connect(ui->pb_read_conf, SIGNAL(clicked()), this, SLOT(slotsButtonReadConf()), Qt::QueuedConnection);
    connect(ui->pb_calibrate_arm, SIGNAL(clicked()), this, SLOT(slotsButtonCalibrate()), Qt::QueuedConnection);
    connect(ui->pb_calibrate_Stop, SIGNAL(clicked()), this, SLOT(slotsButtonCalibrateStop()), Qt::QueuedConnection);

    connect(ui->pb_save_sikoxy, SIGNAL(clicked()), this, SLOT(slotsButtonReSaveSiko()), Qt::QueuedConnection);

    //起吊界面按钮做的动作
    connect(mLiftHandCheckBoxModule, SIGNAL(currentIndexChanged(int)), this, SLOT(liftModuleChanged(int)));
    connect(ui->liftPB_allM_start, SIGNAL(clicked()), this, SLOT(slotsButtonLiftAllMoveStart()), Qt::QueuedConnection);
    connect(ui->liftPB_allM_stop, SIGNAL(clicked()), this, SLOT(slotsButtonLiftAllMoveStop()), Qt::QueuedConnection);
    connect(ui->liftPB_allP_start, SIGNAL(clicked()), this, SLOT(slotsButtonLiftAllPullStart()), Qt::QueuedConnection);
    connect(ui->liftPB_allP_stop, SIGNAL(clicked()), this, SLOT(slotsButtonLiftAllPullStop()), Qt::QueuedConnection);

    //正常运行界面按钮做的动作
    connect(ui->runR_start, SIGNAL(clicked()), this, SLOT(slotsButtonRunStart()), Qt::QueuedConnection);
    connect(ui->runR_stop, SIGNAL(clicked()), this, SLOT(slotsButtonRunStop()), Qt::QueuedConnection);
    connect(ui->runR_Estop, SIGNAL(clicked()), this, SLOT(slotsButtonRunEStop()), Qt::QueuedConnection);

    //显示细节界面
    connect(ui->detail_pb_show, SIGNAL(clicked()), this, SLOT(slotsButtonDetailShow()), Qt::QueuedConnection);
    connect(ui->detail_cmodules, SIGNAL(currentIndexChanged(int)),this, SLOT(detailModuleChanged(int)));

    //数据回放
    connect(ui->detail_pb_playback, SIGNAL(clicked()), this, SLOT(slotsButtonRunPlayStart()), Qt::QueuedConnection);
    connect(ui->detail_pb_playback_stop, SIGNAL(clicked()), this, SLOT(slotsButtonRunPlayStop()), Qt::QueuedConnection);


    ////////////////////////////////vicon读取线程消息函数///////////////////////////////
    //notice msg connect
    connect(m_ThreadLinker, &ThreadLinker::signalNoticeToWind, this, &MainWindow::recNoticeMessages,Qt::QueuedConnection);
    connect(this, &MainWindow::signalNoticeToLinker, m_ThreadLinker, &ThreadLinker::recNoticeMessages,Qt::QueuedConnection);
}

MainWindow::~MainWindow()
{
    delete ui;
    free(gMThreadInfo);
}

/**********************************************************
* @param event: [in]event是关闭事件
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     关闭窗体调用函数
***********************************************************/
void MainWindow::closeEvent(QCloseEvent *event)
{
    //结束所有线程
    sendNoticeToAll(BASE::MSG_NOTICE_KILL_ALL, 0);
    m_ThreadLinker->wait();
    m_ThreadLinker->quit();
    deInitBase();
}

/**********************************************************
* @param index: [in]起吊界面中 选择的机械臂index号
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     当选择单元改变时候，对应的修改起吊界面中文本框状态
***********************************************************/
void MainWindow::liftModuleChanged(int index)
{
    /*************************其他未被选中的单元的控件设置false************************/
    for (int j=0; j<SYS_ARMS_MAX_SIZE; j++)
    {
        mLiftRadioButton[j]->setChecked(false);
    }
    mLiftRadioButton[index]->setChecked(true);
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_RCONF, 0, 0);
}

/**********************************************************
* @param com: [in]下拉框
* @param tIndex: [in]下拉框下设置的单元ID
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     当起吊界面选择单元下拉框，选中一个单元后，执行期间将其他单元值false
***********************************************************/
void MainWindow::setItemsDisabledExcept(QComboBox *com, int tIndex)
{
    //设置全部可用
    if(tIndex < 0)
    {
        for(int i=0; i<com->count(); ++i)
        {
            QModelIndex index = com->model()->index(i, 0);  //第i项
            QVariant v(-1);
            com->model()->setData(index, v, Qt::UserRole - 1);
        }
    }
    else
    {
        for(int i=0; i<com->count(); ++i)
        {
            QModelIndex index = com->model()->index(i, 0);  //第i项
            QVariant v(0);
            com->model()->setData(index, v, Qt::UserRole - 1);
        }
        QModelIndex index = com->model()->index(tIndex, 0);  //第tIndex项设置可选
        QVariant v(-1);
        com->model()->setData(index, v, Qt::UserRole - 1);
    }

}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     起吊按钮开始槽函数
***********************************************************/
void MainWindow::slotsButtonLiftHandStart()
{
    /*
    BASE::LiftCmdData *mMoveData = (BASE::LiftCmdData*) malloc(sizeof(BASE::LiftCmdData));
    memset((char*)mMoveData, 0, sizeof(BASE::LiftCmdData));
    mMoveData->mMudoleNum = mLiftHandCheckBoxModule->currentIndex();
    mMoveData->v_p[0] = ui->MoveCmd_x->text().toFloat();
    mMoveData->v_p[1] = ui->MoveCmd_y->text().toFloat();
    mMoveData->v_p[2] = ui->MoveCmd_z->text().toFloat();
    mMoveData->v_p[3] = ui->MoveCmd_w->text().toFloat();
    mMoveData->isVelOrPos = 0;
    mMoveData->mIsValid = 1;
    //将手动移动数据发送给linker线程，
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_HAND_MOVE, 0, (char*)mMoveData);

    //设置按钮逻辑
    ui->liftPB_hand_start->setEnabled(false);
    ui->liftPB_hand_stop->setEnabled(true);
    ui->liftPB_allM_start->setEnabled(false);
    ui->liftPB_allP_start->setEnabled(false);

    ui->pb_read_conf->setEnabled(false);
    ui->pb_save_conf->setEnabled(false);

    ui->runR_start->setEnabled(false);
    //设置item未选择其他项都不可选
    setItemsDisabledExcept(mLiftHandCheckBoxModule, mLiftHandCheckBoxModule->currentIndex());
    */
}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     起吊按钮停止槽函数
***********************************************************/
void MainWindow::slotsButtonLiftHandStop()
{
    /*
    //将手动停止发送给linker线程，
    BASE::LiftCmdData *mMoveData = (BASE::LiftCmdData*) malloc(sizeof(BASE::LiftCmdData));
    memset((char*)mMoveData, 0, sizeof(BASE::LiftCmdData));
    mMoveData->mMudoleNum = mLiftHandCheckBoxModule->currentIndex();
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_HAND_MOVE_STOP, 0, (char*)mMoveData);

    ui->MoveCmd_x->setText("0");
    ui->MoveCmd_y->setText("0");
    ui->MoveCmd_z->setText("0");
    ui->MoveCmd_w->setText("0");

    //设置按钮有效，无效
    ui->liftPB_hand_start->setEnabled(true);
    ui->liftPB_hand_stop->setEnabled(false);
    ui->liftPB_allM_start->setEnabled(true);
    ui->liftPB_allP_start->setEnabled(true);

    ui->pb_read_conf->setEnabled(true);
    ui->pb_save_conf->setEnabled(true);

    ui->runR_start->setEnabled(true);
    //设置item项都可选
    setItemsDisabledExcept(mLiftHandCheckBoxModule, -1);
    */
}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     起吊按钮全部开始槽函数
***********************************************************/
void MainWindow::slotsButtonLiftAllMoveStart()
{
    //检查是否选择了要移动的模块单元
    int checkNums = 0;
    for(int i=0; i<SYS_ARMS_MAX_SIZE; i++)
    {
        if(mLiftCheckBoxs[i]->isChecked())
            checkNums++;
    }
    //未选择模块
    if(checkNums == 0)
        return;

    BASE::LiftCmdData *mMoveData = (BASE::LiftCmdData*) malloc(sizeof(BASE::LiftCmdData)*SYS_ARMS_MAX_SIZE);
    memset((char*)mMoveData, 0, sizeof(BASE::LiftCmdData)*SYS_ARMS_MAX_SIZE);
    for(int i=0; i<SYS_ARMS_MAX_SIZE; i++)
    {
        if(mLiftCheckBoxs[i]->isChecked())
        {
            mMoveData[i].mIsValid = 1;
            mMoveData[i].mMudoleNum = i;
            mMoveData[i].v_p[0] = ui->liftS_all_moveX->text().toFloat();
            mMoveData[i].v_p[1] = ui->liftS_all_moveY->text().toFloat();
            mMoveData[i].v_p[2] = ui->liftS_all_moveZ->text().toFloat();
            mMoveData[i].v_p[3] = ui->liftS_all_moveW->text().toFloat();
            mMoveData[i].isVelOrPos = 1;

        }
    }
    //将手动移动数据发送给linker线程，
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_ALL_MOVE, 0, (char*)mMoveData);
    //设置按钮有效，无效
    ui->liftPB_allP_start->setEnabled(false);

    ui->liftPB_allM_stop->setEnabled(true);

    ui->pb_read_conf->setEnabled(false);
    ui->pb_save_conf->setEnabled(false);

    ui->runR_start->setEnabled(false);
}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     起吊按钮全部停止槽函数
***********************************************************/
void MainWindow::slotsButtonLiftAllMoveStop()
{
    //将手动停止发送给linker线程，
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_ALL_MOVE_STOP, 0, 0);
    //设置按钮有效，无效
    ui->liftPB_allP_start->setEnabled(true);
    ui->liftPB_allM_start->setEnabled(true);
    ui->liftPB_allM_stop->setEnabled(false);

    ui->pb_read_conf->setEnabled(true);
    ui->pb_save_conf->setEnabled(true);

    ui->runR_start->setEnabled(true);

    ui->liftS_all_moveX->setText("0");
    ui->liftS_all_moveY->setText("0");
    ui->liftS_all_moveZ->setText("0");
    ui->liftS_all_moveW->setText("0");
}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     起吊按钮拉力全部开始槽函数
***********************************************************/
void MainWindow::slotsButtonLiftAllPullStart()
{
    //检查是否选择了要移动的模块单元
    int checkNums = 0;
    for(int i=0; i<SYS_ARMS_MAX_SIZE; i++)
    {
        if(mLiftCheckBoxs[i]->isChecked())
            checkNums++;
    }
    //未选择模块
    if(checkNums == 0)
        return;

    if((ui->liftS_all_pull->text().toFloat()>99.9) || (ui->liftS_all_pull->text().toFloat()<0.001))
        return;

    BASE::LiftCmdData *mPullData = (BASE::LiftCmdData*) malloc(sizeof(BASE::LiftCmdData)*SYS_ARMS_MAX_SIZE);
    memset((char*)mPullData, 0, sizeof(BASE::LiftCmdData)*SYS_ARMS_MAX_SIZE);
    for(int i=0; i<SYS_ARMS_MAX_SIZE; i++)
    {
        if(mLiftCheckBoxs[i]->isChecked())
        {
            mPullData[i].mIsValid = 1;
            mPullData[i].mMudoleNum = i;
            mPullData[i].v_p[2] = ui->liftS_all_pull->text().toFloat();

        }
    }
    //将整体pull移动数据发送给linker线程，
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_ALL_PULL, 0, (char*)mPullData);
    //设置按钮有效，无效
    ui->liftPB_allP_start->setEnabled(false);
    ui->liftPB_allM_start->setEnabled(false);
    ui->liftPB_allP_stop->setEnabled(true);

    ui->pb_read_conf->setEnabled(false);
    ui->pb_save_conf->setEnabled(false);

    ui->runR_start->setEnabled(false);
}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     起吊按钮拉力全部停止槽函数
***********************************************************/
void MainWindow::slotsButtonLiftAllPullStop()
{
    //将手动停止发送给linker线程，
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_ALL_PULL_STOP, 0, 0);
    //设置按钮有效，无效
    ui->liftPB_allP_start->setEnabled(true);
    ui->liftPB_allM_start->setEnabled(true);
    ui->liftPB_allP_stop->setEnabled(false);

    ui->pb_read_conf->setEnabled(true);
    ui->pb_save_conf->setEnabled(true);

    ui->runR_start->setEnabled(true);
}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     运行界面全部开始槽函数
***********************************************************/
void MainWindow::slotsButtonRunStart()
{
    memset((char*)mRunMask, 0, sizeof(int)*SYS_ARMS_MAX_SIZE);
    for(int i=0; i<SYS_ARMS_MAX_SIZE; ++i)
    {
      if(mRunCheckBoxs[i]->isChecked())
      {
          mRunMask[i] = ui->chose_magicT->currentIndex() + 1;
      }
    }

    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_RUN_START, 0, (char*)mRunMask);
    //设置按钮有效，无效
    ui->liftPB_allP_start->setEnabled(false);

    ui->pb_read_conf->setEnabled(false);
    ui->pb_save_conf->setEnabled(false);
    ui->runR_start->setEnabled(false);
    ui->runShowD_start->setEnabled(false);

    ui->detail_pb_show->setEnabled(true);
    ui->runShowD_stop->setEnabled(true);
    ui->runR_stop->setEnabled(true);
    ui->runR_Estop->setEnabled(true);
}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     运行界面全部停止槽函数
***********************************************************/
void MainWindow::slotsButtonRunStop()
{
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_RUN_STOP, 0, 0);
    //设置按钮有效，无效
    ui->liftPB_allP_start->setEnabled(true);
    ui->liftPB_allM_start->setEnabled(true);
    ui->pb_read_conf->setEnabled(true);
    ui->pb_save_conf->setEnabled(true);
    ui->runR_start->setEnabled(true);
    ui->runShowD_start->setEnabled(true);

    ui->runShowD_stop->setEnabled(false);
    //ui->runR_stop->setEnabled(false);
    ui->runR_Estop->setEnabled(false);
    ui->detail_pb_show->setEnabled(false);
}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     运行界面全部急停槽函数
***********************************************************/
void MainWindow::slotsButtonRunEStop()
{
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_RUN_STOPE, 0, 0);
    //设置按钮有效，无效
    qDebug()<<"stop E";
    ui->liftPB_allP_start->setEnabled(true);
    ui->liftPB_allM_start->setEnabled(true);
    ui->pb_read_conf->setEnabled(true);
    ui->pb_save_conf->setEnabled(true);
    ui->runR_start->setEnabled(true);
    ui->runShowD_start->setEnabled(true);

    ui->runShowD_stop->setEnabled(false);
    //ui->runR_stop->setEnabled(false);
    ui->runR_Estop->setEnabled(false);
    ui->detail_pb_show->setEnabled(false);
}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     连接开始槽函数
***********************************************************/
void MainWindow::slotsButtonLink()
{
    //读取ip,port
    mIP     = mLinkIp->text();
    mPort   = mLinkPort->text().toInt();
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_LINK, mPort, (char*)&mIP);
}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     断开连接槽函数
***********************************************************/
void MainWindow::slotsButtonUnLink()
{
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_UNLINK, 0, 0);
}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     配置界面中保存配置槽函数
***********************************************************/
void MainWindow::slotsButtonSaveConf()
{
    BASE::ConfData *mModulesConfDatas = (BASE::ConfData *)malloc(sizeof(BASE::ConfData)*SYS_ARMS_MAX_SIZE);
    memset((char*)mModulesConfDatas, 0, sizeof(BASE::ConfData)*SYS_ARMS_MAX_SIZE);
    //读取checkbox待配置单元
    for(int i=0; i<SYS_ARMS_MAX_SIZE; i++)
    {
        if(mLiftRadioButton[i]->isChecked())
        {
            mModulesConfDatas[i].mIsValid = 1;
            mModulesConfDatas[i].mConfSaveWeight   = ui->conf_Weight->text().toFloat();
            mModulesConfDatas[i].mConfSaveEncoderT = ui->conf_encoderT->text().toFloat();
            mModulesConfDatas[i].mConfSaveSikoX    = ui->conf_sikoX->text().toFloat();
            mModulesConfDatas[i].mConfSaveSikoY    = ui->conf_sikoY->text().toFloat();
            mModulesConfDatas[i].mFollowKp         = ui->conf_Kp->text().toFloat();
            mModulesConfDatas[i].mFollowKd         = ui->conf_Kd->text().toFloat();
            mModulesConfDatas[i].mWn               = ui->conf_Wn->text().toFloat();
            mModulesConfDatas[i].mCo               = ui->conf_Co->text().toFloat();
        }
    }

    //将配置数据发送给linker线程，
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_SCONF, 0, (char*)mModulesConfDatas);
}


/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     配置界面中保存配置槽函数
***********************************************************/
void MainWindow::slotsButtonCalibrate()
{
    //读取checkbox待配置单元
    BASE::CalibrateData *mModuleDatas = (BASE::CalibrateData *)malloc(sizeof(BASE::CalibrateData)*SYS_ARMS_MAX_SIZE);
    memset((char*)mModuleDatas, 0, sizeof(BASE::CalibrateData)*SYS_ARMS_MAX_SIZE);

    for(int i=0; i<SYS_ARMS_MAX_SIZE; i++)
    {
        if(mLiftRadioButton[i]->isChecked())
        {
            mModuleDatas[i].mIsValid = 1;
            mModuleDatas[i].mMudoleNum = i;
            mModuleDatas[i].mCalKg = ui->calibrate_kg->text().toFloat();
        }
    }

    //将配置数据发送给linker线程，
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_CALIBRATE, 0, (char*)mModuleDatas);
}


void MainWindow::slotsButtonCalibrateStop()
{
    //将手动停止发送给linker线程，
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_ALL_MOVE_STOP, 0, 0);
}


void MainWindow::slotsButtonReSaveSiko()
{
    BASE::ConfData *mModulesConfDatas = (BASE::ConfData *)malloc(sizeof(BASE::ConfData)*SYS_ARMS_MAX_SIZE);
    memset((char*)mModulesConfDatas, 0, sizeof(BASE::ConfData)*SYS_ARMS_MAX_SIZE);
    //将手动停止发送给linker线程，
    for(int i=0; i<SYS_ARMS_MAX_SIZE; i++)
    {
        if(mLiftRadioButton[i]->isChecked())
        {
            mModulesConfDatas[i].mIsValid = 1;
            mModulesConfDatas[i].mConfSaveWeight   = ui->conf_Weight->text().toFloat();
            mModulesConfDatas[i].mConfSaveEncoderT = ui->conf_encoderT->text().toFloat();
            mModulesConfDatas[i].mConfSaveSikoX    = ui->conf_sikoX->text().toFloat() + mConfReadSikoX[i]->text().toFloat();
            mModulesConfDatas[i].mConfSaveSikoY    = ui->conf_sikoY->text().toFloat() + mConfReadSikoY[i]->text().toFloat();
            mModulesConfDatas[i].mFollowKp         = ui->conf_Kp->text().toFloat();
            mModulesConfDatas[i].mFollowKd         = ui->conf_Kd->text().toFloat();
            mModulesConfDatas[i].mWn               = ui->conf_Wn->text().toFloat();
            mModulesConfDatas[i].mCo               = ui->conf_Co->text().toFloat();
        }
    }
    //将配置数据发送给linker线程，
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_SCONF, 0, (char*)mModulesConfDatas);
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_RCONF, 0, 0);

}
/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     配置界面中读取配置槽函数
***********************************************************/
void MainWindow::slotsButtonReadConf()
{
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_RCONF, 0, 0);
}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     细节显示1界面中，多选框状态改变函数
***********************************************************/
void MainWindow::detailCb1StateChanged(int state)
{
    if (state == Qt::Checked) // "选中"
    {
        mDetailCbChoosSize1++;
        //选择了三个显示曲线，将多选框设置false，不能再次选择
        for (int i=0; i<SHOW_DETAILS_MUD_MUNS; i++)
        {
            //将新选择的，找到显示曲线下标
            if(widShowCB[i]->isChecked())
            {
                if(mDrawDatas.drawMark[i] == 0)
                {
                    mDrawDatas.drawMark[i] = 1;
                    if(mDrawP->state() == QProcess::ProcessState::Running)
                    {
                        mDrawDatas.m_Value = i;
                        mDrawDatas.m_Notice = BASE::CMD_DRAW_ADD;
                        m_sender->writeDatagram((char*)&mDrawDatas, sizeof(mDrawDatas), QHostAddress::LocalHost, 9999);
                    }
                }
            }
            //如果选择显示数量3 的话，将未选择的置false，下次就不能在选择
            if(!widShowCB[i]->isChecked() && (mDetailCbChoosSize1 >= 5))
                widShowCB[i]->setEnabled(false);
        }
    }
    else if(state == Qt::PartiallyChecked) // "半选"
    {
        qDebug()<<("PartiallyChecked");
    }
    else // 未选中 - Qt::Unchecked
    {
        mDetailCbChoosSize1--;
        for (int i=0; i<SHOW_DETAILS_MUD_MUNS; i++)
        {
            widShowCB[i]->setEnabled(true);
            //找到取消显示，并将显示曲线下标置空
            if(!widShowCB[i]->isChecked())
            {
                //之前显示，清空这个传感器显示
                if(mDrawDatas.drawMark[i] == 1)
                {
                    mDrawDatas.drawMark[i] = 0;
                    if(mDrawP->state() == QProcess::ProcessState::Running)
                    {
                        mDrawDatas.m_Value = i;
                        mDrawDatas.m_Notice = BASE::CMD_DRAW_CLEAR;
                        m_sender->writeDatagram((char*)&mDrawDatas, sizeof(mDrawDatas), QHostAddress::LocalHost, 9999);
                    }
                }
            }

        }
    }

    qDebug()<< mDrawDatas.drawMark[0]<<mDrawDatas.drawMark[1]<<mDrawDatas.drawMark[2]<<mDrawDatas.drawMark[3]<<mDrawDatas.drawMark[4]<<mDrawDatas.drawMark[5]<<mDrawDatas.drawMark[6]<<mDrawDatas.drawMark[7];
}

/**********************************************************
* @param index: [in]细节显示中 选择的机械臂单元
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     当选择单元改变时候，清空多选框所有
***********************************************************/
void MainWindow::detailModuleChanged(int index)
{
    for (int i=0; i<SHOW_DETAILS_MUD_MUNS; i++)
    {
        mDrawDatas.drawMark[i] = 0;
        widShowCB[i]->setChecked(false);

        if(mDrawP->state() == QProcess::ProcessState::Running)
        {
            mDrawDatas.m_Notice = BASE::CMD_DRAW_CLEAR_ALL;
            m_sender->writeDatagram((char*)&mDrawDatas, sizeof(mDrawDatas), QHostAddress::LocalHost, 9999);
        }
    }
}

//TUDO
/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     细节显示界面中启动图形显示
***********************************************************/
void MainWindow::slotsButtonDetailShow()
{
    //启动显示进程
    if(mDrawP->state() == QProcess::ProcessState::NotRunning)
    {
        QStringList list;
        mDrawP->start("showWind.exe", list);
        Sleep(1);
        mDrawDatas.m_Notice = BASE::CMD_DRAW_START;
        mDrawDatas.m_Value  = 90;
        m_sender->writeDatagram((char*)&mDrawDatas, sizeof(mDrawDatas), QHostAddress::LocalHost, 9999);
    }
}

void MainWindow::slotsButtonRunPlayStart()
{
    /*
    //启动显示进程
    if(mDrawP->state() == QProcess::ProcessState::NotRunning)
    {
        QStringList list;
        mDrawP->start("showWind.exe", list);
    }
    //发送给linker要回放的起始周期
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_RUN_PLAY, ui->detail1_starTime1->text().toInt(), 0);
    */

    //TUDU 测试曲线绘制
    static int mm;
    mm++;
    BASE::ARMS_R_USE_MSG *mMs = (BASE::ARMS_R_USE_MSG *)malloc(sizeof(BASE::ARMS_R_USE_MSG)*11);
    for(int i=0;i<11;i++)
    {
        mMs[i].mEncoderTurns = mm+i;
        mMs[i].mInclinometer1_x = mm+i;
        mMs[i].mSiko1 = mm+i;
        mMs[i].mSiko2 = mm+i;
        mMs[i].mMotors[0].mSpeed = mm+i;
        mMs[i].mMotors[1].mSpeed = mm+i;
        mMs[i].mMotors[2].mSpeed = mm+i;
        mMs[i].mMotors[3].mPosition = mm+i;
        mMs[i].mTension = mm+i;
    }
    showRunMessage((char*)mMs);
    free(mMs);
}

void MainWindow::slotsButtonRunPlayStop()
{
    sendNotice(BASE::THREAD_ID_LINKER, BASE::MSG_NOTICE_RUN_PLAY_STOP, 0, 0);
}


/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     TUDO
***********************************************************/
void MainWindow::slotsButtonConf()
{
    //当前已经处于配置模式
    if(checkAllThreadStates(BASE::_MODULE_CONF_OK))
        return ;

    //只有初始化成功 运行模式才能转换成配置模式
    if(checkAllThreadStates(BASE::_MODULE_INIT_OK) || checkAllThreadStates(BASE::_MODULE_RUNNING))
    {
        qDebug()<<"转换到conf模式";
        sendNoticeToAll(BASE::MSG_NOTICE_CONF_ALL, 0);
        return;
    }
    qDebug()<<"转换到conf模式失败";
}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     TUDO
***********************************************************/
void MainWindow::slotsButtonRunning()
{
    //当前已经处于运行模式
    if(checkAllThreadStates(BASE::_MODULE_RUNNING))
        return ;

    //只有初始化成功 配置模式才能转换成运行模式
    if(checkAllThreadStates(BASE::_MODULE_INIT_OK) || checkAllThreadStates(BASE::_MODULE_CONF_OK))
    {
        qDebug()<<"转换到running模式";
        sendNoticeToAll(BASE::MSG_NOTICE_RUN_ALL, 0);
        return;
    }
    qDebug()<<"转换到running模式失败";
}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     TUDO
***********************************************************/
void MainWindow::slotsButtonStop()
{
    //当前已经处于运行模式
    if(checkAllThreadStates(BASE::_MODULE_STOP))
        return ;

    qDebug()<<"************";
    //只有初始化成功 配置模式才能转换成运行模式
    if(checkAllThreadStates(BASE::_MODULE_INIT_OK) || checkAllThreadStates(BASE::_MODULE_RUNNING) || checkAllThreadStates(BASE::_MODULE_CONF_OK))
    {
        qDebug()<<"转换到stop模式";
        sendNoticeToAll(BASE::MSG_NOTICE_STOP_ALL, 0);
        return;
    }
    qDebug()<<"转换到running模式失败";
}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     初始化定时器
***********************************************************/
int  MainWindow::initTimer()
{
    if(qMTimer != nullptr)
        delete qMTimer;

    //启动定时器
    qMTimer = new QTimer(this);
    connect(qMTimer, SIGNAL(timeout()), this, SLOT(slotsHandleTimer()));
    qMTimer->start(TIMER_TIMEOUT);
    return 0;
}


/**********************************showConfMessage**********************************
* pShowMsg : [输入]显示信息的地址
* @return void
* 说明：将pShowMsg中的信息显示在配置界面中，每点击读取配置一次，将发送控制器读取配置信号，然后接收到配置
*      信息后就将此信息显示。
***********************************************************************************/
void MainWindow::showConfMessage(BASE::ConfData *pShowMsg)
{
    memcpy((char*)mConfPa, (char*)pShowMsg, sizeof(BASE::ConfData)*SYS_ARMS_MAX_SIZE);
    for(int i=0; i<SYS_ARMS_MAX_SIZE; i++)
    {
        if(mLiftRadioButton[i]->isChecked())
        {
            ui->conf_Weight->setText(QString("%1").arg(pShowMsg[i].mConfSaveWeight));
            ui->conf_encoderT->setText(QString("%1").arg(pShowMsg[i].mConfSaveEncoderT));
            ui->conf_sikoX->setText(QString("%1").arg(pShowMsg[i].mConfSaveSikoX));
            ui->conf_sikoY->setText(QString("%1").arg(pShowMsg[i].mConfSaveSikoY));
            ui->conf_Kp->setText(QString("%1").arg(pShowMsg[i].mFollowKp));
            ui->conf_Kd->setText(QString("%1").arg(pShowMsg[i].mFollowKd));
            ui->conf_Wn->setText(QString("%1").arg(pShowMsg[i].mWn));
            ui->conf_Co->setText(QString("%1").arg(pShowMsg[i].mCo));
        }
    }
}

/**********************************showConfMessage**********************************
* pShowMsg : [输入]显示信息的地址
* @return void
* 说明：将pShowMsg中的信息显示在配置界面中，每点击读取配置一次，将发送控制器读取配置信号，然后接收到配置
*      信息后就将此信息显示。
***********************************************************************************/
void MainWindow::showCycMessage(BASE::ARMS_R_USE_MSG *pShowMsg)
{
    BASE::ARMS_MULTICAST_UDP mMulticatMsg = {0};
    static int32_t randD = 0;
    mMulticatMsg.mIdentifier = 0;
    mMulticatMsg.mRandomCode = randD++;
    for(int i=0; i<SYS_ARMS_MAX_SIZE; i++)
    {
        mConfReadPull[i]->setText(QString("%1").arg(pShowMsg[i].mTension/0.0098));
        mConfReadEncoderT[i]->setText(QString("%1").arg(pShowMsg[i].mEncoderTurns*57.29));
        mConfReadSikoX[i]->setText(QString("%1").arg(pShowMsg[i].mSiko1*1000.0));
        mConfReadSikoY[i]->setText(QString("%1").arg(pShowMsg[i].mSiko2*1000.0));
        mConfReadLevelX[i]->setText(QString("%1").arg(pShowMsg[i].mInclinometer1_x));
        mConfReadLevelY[i]->setText(QString("%1").arg(pShowMsg[i].mInclinometer1_x));

        mLiftHandXNow[i]->setText(QString("%1").arg(pShowMsg[i].mMotors[0].mSpeed));
        mLiftHandYNow[i]->setText(QString("%1").arg(pShowMsg[i].mMotors[1].mSpeed));
        mLiftHandZNow[i]->setText(QString("%1").arg(pShowMsg[i].mMotors[2].mSpeed));
        mLiftHandWNow[i]->setText(QString("%1").arg(pShowMsg[i].mMotors[3].mSpeed));
        mESwitch[i]->setText(QString("%1").arg(pShowMsg[i].mSwitchStateCode));

        mRunReadX[i]->setText(QString("%1").arg(pShowMsg[i].mMotors[0].mPosition));
        mRunReadY[i]->setText(QString("%1").arg(pShowMsg[i].mMotors[1].mPosition));
        mRunReadZ[i]->setText(QString("%1").arg(pShowMsg[i].mMotors[2].mPosition));

        //多播数据传输
        mMulticatMsg.mMark[i] = 1;
        mMulticatMsg.mTension[i] = (int32_t)(pShowMsg[i].mTension/0.0098);
    }

    //多播传输
    m_MulticastSend->writeDatagram((char*)&mMulticatMsg, sizeof(mMulticatMsg), QHostAddress("192.168.1.201"), 10001);
}


/**********************************showLiftMessage**********************************
* pShowMsg : [输入]显示信息的地址
* @return void
* 说明：将pShowMsg中的信息显示在起重界面中，10hz显示。
***********************************************************************************/
void MainWindow::showLiftMessage(const int MSG_SHOW_TYPE, char *pShowMsg)
{
    //定期显示各个模块单元HZ 抖动时间
    if(MSG_SHOW_TYPE == BASE::MSG_ACK_HZ_SHAKE)
    {
        BASE::ARMS_R_USE_MSG *showMsg = (BASE::ARMS_R_USE_MSG *)pShowMsg;
        for(int i=0; i<SYS_ARMS_MAX_SIZE; i++)
        {
            ;
        }
    }

    //定期显示指定模块单元当前pos
    if(MSG_SHOW_TYPE == BASE::MSG_ACK_HAND_MOVE)
    {
        BASE::ARMS_R_USE_MSG *showMsg = (BASE::ARMS_R_USE_MSG *)pShowMsg;
        for(int i=0; i<SYS_ARMS_MAX_SIZE; i++)
        {
;
        }

    }

}

/**********************************************************
* @param : [in]
* @return Descriptions
* @exception Type1: Descriptions
* 描述：
*     显示运行界面信息
***********************************************************/
void MainWindow::showRunMessage(char *pShowMsg)
{
    BASE::ARMS_R_USE_MSG *showMsg = (BASE::ARMS_R_USE_MSG *)pShowMsg;

    //显示运行界面数据
    for(int i=0; i<SYS_ARMS_MAX_SIZE; i++)
    {
        mRunReadX[i]->setText(QString("%1").arg(showMsg[i].mMotors[0].mSpeed));
        mRunReadY[i]->setText(QString("%1").arg(showMsg[i].mMotors[1].mSpeed));
        mRunReadZ[i]->setText(QString("%1").arg(showMsg[i].mMotors[2].mSpeed));

        mRunReadAngleX[i]->setText(QString("%1").arg(showMsg[i].mInclinometer1_x));
        mRunReadAngleY[i]->setText(QString("%1").arg(showMsg[i].mInclinometer1_y));
        mRunReadPullE[i]->setText(QString("%1").arg(showMsg[i].mTension));
    }

    //显示细节运行界面中的数据
    int detail_cmodules = ui->detail_cmodules->currentIndex();
    ui->detail1_Rsiko1->setText(QString("%1").arg(showMsg[detail_cmodules].mSiko1));
    ui->detail1_Rsiko2->setText(QString("%1").arg(showMsg[detail_cmodules].mSiko2));
    ui->detail1_Rlevel1->setText(QString("%1").arg(showMsg[detail_cmodules].mInclinometer1_x));
    ui->detail1_Rlevel2->setText(QString("%1").arg(showMsg[detail_cmodules].mInclinometer1_y));
    ui->detail1_RencoderT->setText(QString("%1").arg(showMsg[detail_cmodules].mEncoderTurns));
    ui->detail1_Rpull_now->setText(QString("%1").arg(showMsg[detail_cmodules].mTension));
    ui->detail1_Rpull_set->setText(QString("%1").arg(showMsg[detail_cmodules].mTension));
    ui->detail1_VX_now->setText(QString("%1").arg(showMsg[detail_cmodules].mMotors[0].mSpeed));
    ui->detail1_VY_now->setText(QString("%1").arg(showMsg[detail_cmodules].mMotors[1].mSpeed));
    ui->detail1_VZ_now->setText(QString("%1").arg(showMsg[detail_cmodules].mMotors[2].mSpeed));
    ui->detail1_VW_now->setText(QString("%1").arg(showMsg[detail_cmodules].mMotors[3].mSpeed));
    ui->detail1_VX_set->setText(QString("%1").arg(showMsg[detail_cmodules].mCmdSpeed[0]));
    ui->detail1_VY_set->setText(QString("%1").arg(showMsg[detail_cmodules].mCmdSpeed[1]));
    ui->detail1_VZ_set->setText(QString("%1").arg(showMsg[detail_cmodules].mCmdSpeed[2]));
    ui->detail1_VW_set->setText(QString("%1").arg(showMsg[detail_cmodules].mCmdSpeed[3]));

    //将显示信息udp发送
    if(mDrawP->state() == QProcess::ProcessState::Running)
    {
        memcpy((char*)&mDrawDatas.mRunData, (char*)&showMsg[detail_cmodules], sizeof(BASE::ARMS_R_USE_MSG));
        mDrawDatas.m_Notice = BASE::CMD_DRAW_FIRE;
        qDebug()<<"show run:"<<m_sender->writeDatagram((char*)&mDrawDatas, sizeof(mDrawDatas), QHostAddress::LocalHost, 9999);
    }
}


