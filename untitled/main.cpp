#include "mainwindow.h"
#include <QApplication>
#include "host_base_messages.h"

/*****************************************************
* @param argc : [in]参数个数
* @param argv : [in]参数
* @return Descriptions
* 描述：
*    该函数是程序入口函数
******************************************************/
int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    //qRegisterMetaType<BASE::Vicon_RecMessages>("Vicon_RecMessages");
    //依次启动各个线程

    //最后启动显示
    MainWindow w;
    w.show();

    return a.exec();
}
