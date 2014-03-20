#include "mainwindow.h"
#include <QApplication>

#include "cv.h"
#include <highgui.h>
#include <QFileDialog>
#include <QMessageBox>
#include <QTextCodec>
#include "glwidget.h"
#include <iostream>

using namespace cv;
using namespace std;


void testBM(){
    QString leftfilename = QFileDialog::getOpenFileName(
       0,
       "Binocular Calibration - Open Left Image",
       QDir::currentPath(),
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!leftfilename.isNull()) { //用户选择了左图文件
        QString rightfilename = QFileDialog::getOpenFileName(
           0,
           "Binocular Calibration - Open Right Image",
           QDir::currentPath(),
           "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
        if (!rightfilename.isNull()) { //用户选择了左图文件
            Mat leftmat = imread(leftfilename.toUtf8().data());
            Mat rightmat = imread(rightfilename.toUtf8().data());
            Mat leftgray,rightgray;
            leftgray.create(leftmat.size(),CV_8UC1);
            rightgray.create(rightmat.size(),CV_8UC1);
            cvtColor(leftmat,leftgray,CV_BGR2GRAY);
            cvtColor(rightmat,rightgray,CV_BGR2GRAY);
            imshow("left_gray",leftgray);
            imshow("right_gray",rightgray);
            StereoBM bm;

           // bm.state->roi1 = roil;
            //bm.state->roi2 = roir;
            bm.state->preFilterCap = 31;
            bm.state->SADWindowSize = 11;
            bm.state->minDisparity = 0;
            bm.state->numberOfDisparities = 16*6;
            bm.state->textureThreshold = 10;
            bm.state->uniquenessRatio = 15;
            bm.state->speckleWindowSize = 100;
            bm.state->speckleRange = 32;
            bm.state->disp12MaxDiff = 1;

            Mat disp,vdisp;
            bm(leftgray,rightgray,disp);//,CV_32F);
            disp.convertTo(vdisp, CV_8U);//, 255/(32*16.));
            //normalize(vdisp,vdisp,0,255,CV_MINMAX);
            imshow("dis",vdisp);
            //freopen("log.txt","w",stdout);
            //imwrite("vdisp.png",vdisp);
            //cout<<vdisp;
        }
    }
}
void testsbm(){
    QString leftfilename = QFileDialog::getOpenFileName(
       0,
       "Binocular Calibration - Open Left Image",
       QDir::currentPath(),
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!leftfilename.isNull()) { //用户选择了左图文件
        QString rightfilename = QFileDialog::getOpenFileName(
           0,
           "Binocular Calibration - Open Right Image",
           QDir::currentPath(),
           "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
        if (!rightfilename.isNull()) { //用户选择了左图文件
            Mat leftmat = imread(leftfilename.toUtf8().data());
            Mat rightmat = imread(rightfilename.toUtf8().data());
            Mat leftgray,rightgray;
            leftgray.create(leftmat.size(),CV_8UC1);
            rightgray.create(rightmat.size(),CV_8UC1);
            cvtColor(leftmat,leftgray,CV_BGR2GRAY);
            cvtColor(rightmat,rightgray,CV_BGR2GRAY);
            imshow("left_gray",leftgray);
            imshow("right_gray",rightgray);



            int SADWindowSize = 11;
            int numberOfDisparities = 16*6;
            StereoSGBM sgbm;
            sgbm.preFilterCap = 63;
            sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

            int cn = 1;//leftgray.channels();

            sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
            sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
            sgbm.minDisparity = 0;
            sgbm.numberOfDisparities = numberOfDisparities;
            sgbm.uniquenessRatio = 10;
            sgbm.speckleWindowSize = 100;
            sgbm.speckleRange = 32;
            sgbm.disp12MaxDiff = 1;
            sgbm.fullDP = true;

            Mat disp,vdisp;

            sgbm(leftgray,rightgray,disp);
            disp.convertTo(vdisp, CV_8U);//, 255/(32*16.));
            imshow("dis",vdisp);
        }
    }
}

int main(int argc, char *argv[])
{
    QTextCodec *codec = QTextCodec::codecForName("System"); //获取系统编码
    QTextCodec::setCodecForLocale(codec);

    QApplication a(argc, argv);

    MainWindow w;
    w.showMaximized();
//    testBM();
//    testsbm();

    return a.exec();

}
