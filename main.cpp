#include "mainwindow.h"
#include <QApplication>

#include "cv.h"
#include <highgui.h>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextCodec>

using namespace cv;

int main(int argc, char *argv[])
{
    QTextCodec *codec = QTextCodec::codecForName("System"); //获取系统编码
    QTextCodec::setCodecForLocale(codec);

    QApplication a(argc, argv);
    MainWindow w;
    w.showMaximized();

    return a.exec();

}
