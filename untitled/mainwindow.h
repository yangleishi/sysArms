/********************************************************************************
* Copyright (c) 2017-2020 NIIDT.
* All rights reserved.
*
* File Type : C++ Header File(*.h)
* File Name : mainwindow.h
* Module :
* Create on: 2021/5/10
* Author: 师洋磊
* Email: 546783926@qq.com
* Description: 主窗体头文件，定义窗体中的控件，消息处理程序等
*
********************************************************************************/
#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QCloseEvent>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QRadioButton>
#include <QProcess>
#include <QComboBox>
#include <QLabel>
#include <QCheckBox>


#include "host_base_messages.h"
#include "threadlinker.h"


#define  DELETE_P(pT) do{ if(pT != nullptr){delete pT;pT = nullptr;}}while(0)

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();


signals:
void signalNoticeToLinker(QVariant);
void signalNoticeToDrawCurve(QVariant);
void signalNoticeToConfBeff(QVariant);
void signalNoticeToWriteViconBeff(QVariant);

//绘制曲线的信号
void signalDrawCurve(QVariant);


public slots:
   void recNoticeMessages(QVariant);

   /***************************按钮做出动作******************************/
   //起吊界面中时候，此函数被触发
   void liftModuleChanged(int index);
   void slotsButtonLiftHandStart();
   void slotsButtonLiftHandStop();
   void slotsButtonLiftAllMoveStart();
   void slotsButtonLiftAllMoveStop();
   void slotsButtonLiftAllPullStart();
   void slotsButtonLiftAllPullStop();

   //运行界面中时候，此函数被触发
   void slotsButtonRunStart();
   void slotsButtonRunStop();
   void slotsButtonRunEStop();
   void detailModuleChanged(int index);


   //数据回放中时候，此函数被触发
   void slotsButtonRunPlayStart();
   void slotsButtonRunPlayStop();


   //界面连接和断开连接时候，此函数被触发
   void slotsButtonLink();
   void slotsButtonUnLink();

   //配置界面读取配置、保存配置时候，此函数被触发
   void slotsButtonSaveConf();
   void slotsButtonReadConf();
   void slotsButtonCalibrate();
   void slotsButtonCalibrateStop();
   void slotsButtonReSaveSiko();

   //显示界面细节1-3点击显示绘曲线，此函数被触发
   void slotsButtonDetailShow();
   //显示界面数据回放，只显示拉力，编码器、输入速度，输出速度
   //void slotsButtonDetailPlayback();

   //细节显示多选按钮触发事件
   void detailCb1StateChanged(int);

   void slotsButtonConf();
   void slotsButtonRunning();
   void slotsButtonStop();
private :
   void handleLinkAck(int mValue);
   void handleUnLinkAck(int mValue);
   void setItemsDisabledExcept(QComboBox *com, int tIndex);

private:
    Ui::MainWindow *ui;
    ThreadLinker         *m_ThreadLinker;

    bool m_ThreadLinkerKill;

    //
    void signalsSlotsConnects();
    void showConfMessage(BASE::ConfData *pShowMsg);
    void showCycMessage(BASE::ARMS_R_USE_MSG *pShowMsg);
    void showLiftMessage(const int MSG_SHOW_TYPE, char *pShowMsg);
    void showRunMessage(char *pShowMsg);

    void sendNotice(const int mRecId, const int pNotice, const int pValue, char *pData);

    void sendNoticeLink(QString sIp, int port);


    void sendNoticeToAll(const int pNotice, const int pValue);
    int  checkAllThreadStates(BASE::_MODULE_RUN_STATE mCheckState);

protected:
    void closeEvent(QCloseEvent *event);

private:
     //私有函数
    void initBase();
    void deInitBase();
    void initControlBinding();

    QStringList createControlName(QString baseString);

    //设置定时器，定期计算频率。
    QTimer*  qMTimer;
    int  initTimer();
    BASE::MODULEINFOS *gMThreadInfo;

    //ip
    QString mIP;
    int     mPort;

    QLineEdit *mLinkIp, *mLinkPort;
/*****************************配置界面**************************************/

    QLineEdit *mConfReadPull[SYS_ARMS_MAX_SIZE];
    QLineEdit *mConfReadEncoderT[SYS_ARMS_MAX_SIZE];
    QLineEdit *mConfReadSikoX[SYS_ARMS_MAX_SIZE];
    QLineEdit *mConfReadSikoY[SYS_ARMS_MAX_SIZE];
    QLineEdit *mConfReadLevelX[SYS_ARMS_MAX_SIZE];
    QLineEdit *mConfReadLevelY[SYS_ARMS_MAX_SIZE];

    //配置界面中十一组要保存的数据
    //BASE::SaveConfData mModulesConfDatas[SYS_ARMS_MAX_SIZE];

/*****************************起重界面*************************************/
    QRadioButton *mLiftRadioButton[SYS_ARMS_MAX_SIZE];

    QLineEdit *mESwitch[SYS_ARMS_MAX_SIZE];


    QLineEdit *mLiftHandXNow[SYS_ARMS_MAX_SIZE];
    QLineEdit *mLiftHandYNow[SYS_ARMS_MAX_SIZE];
    QLineEdit *mLiftHandZNow[SYS_ARMS_MAX_SIZE];
    QLineEdit *mLiftHandWNow[SYS_ARMS_MAX_SIZE];
    QCheckBox *mLiftCheckBoxs[SYS_ARMS_MAX_SIZE];

    QCheckBox *mRunCheckBoxs[SYS_ARMS_MAX_SIZE];
    int       mRunMask[SYS_ARMS_MAX_SIZE];

    QComboBox *mLiftHandCheckBoxModule;

/*****************************运行界面控件*************************************/
    QLabel *mLabelsRun[SYS_ARMS_MAX_SIZE];
    QLineEdit *mRunReadX[SYS_ARMS_MAX_SIZE];
    QLineEdit *mRunReadY[SYS_ARMS_MAX_SIZE];
    QLineEdit *mRunReadZ[SYS_ARMS_MAX_SIZE];
    QLineEdit *mRunReadAngleX[SYS_ARMS_MAX_SIZE];
    QLineEdit *mRunReadAngleY[SYS_ARMS_MAX_SIZE];
    QLineEdit *mRunReadPullE[SYS_ARMS_MAX_SIZE];


/**************************细节显示中绘制曲线*******************************/
    QCheckBox *widShowCB[SHOW_DETAILS_MUD_MUNS];

    int mDetailCbChoosSize1;

    QProcess *mDrawP;
    BASE::DrawData mDrawDatas;//两个细节显示，每个最多显示3个曲线
    //比如磁栅尺要显示在哪个曲线上
    //int  show1CBindex[SHOW_DETAILS_MUD_MUNS];
    QUdpSocket *m_sender;//, *m_rec;

    BASE::ConfData mConfPa[SYS_ARMS_MAX_SIZE];

/****************************************UI中所有控件*********************************/
private:
    void setUi();

};

#endif // MAINWINDOW_H
