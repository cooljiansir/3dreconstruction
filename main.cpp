#include "mainwindow.h"
#include <QApplication>
#include "cv.h"
#include "highgui.h"
#include <QFileDialog>

using namespace cv;
using namespace std;

#define MAXS 15 //最大搜索视差


void testBM(){
    QString leftfilename = QFileDialog::getOpenFileName(
       0,
       "Binocular Calibration - Open Left Image",
       NULL,
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!leftfilename.isNull()) { //用户选择了左图文件
        QString rightfilename = QFileDialog::getOpenFileName(
           0,
           "Binocular Calibration - Open Right Image",
           NULL,
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
            bm.state->SADWindowSize = 7;
            bm.state->minDisparity = 0;
            bm.state->numberOfDisparities = 16*6;
            bm.state->textureThreshold = 10;
            bm.state->uniquenessRatio = 15;
            bm.state->speckleWindowSize = 100;
            bm.state->speckleRange = 32;
            bm.state->disp12MaxDiff = 1;

            Mat disp,vdisp;
            bm(leftgray,rightgray,disp,CV_32F);
            disp.convertTo(vdisp, CV_8U);//, 255/(32*16.));
            normalize(vdisp,vdisp,0,255,CV_MINMAX);
            imshow("dis",vdisp);
            freopen("log.txt","w",stdout);
            //imwrite("vdisp.png",vdisp);
            cout<<disp;
        }
    }
}
void testsbm(){
    QString leftfilename = QFileDialog::getOpenFileName(
       0,
       "Binocular Calibration - Open Left Image",
       NULL,
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!leftfilename.isNull()) { //用户选择了左图文件
        QString rightfilename = QFileDialog::getOpenFileName(
           0,
           "Binocular Calibration - Open Right Image",
           NULL,
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



            int SADWindowSize = 7;
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
//sad值函数
double sadValue(int x1,int y1,Mat &left,int x2,int y2,Mat &right,int size){
    double sum = 0;
    int x_1,y_1,x_2,y_2;
    for(int i = -size;i<=size;i++){
        for(int j = -size;j<=size;j++){
            y_1 = y1 + i;
            x_1 = x1 + j;
            y_2 = y2 + i;
            x_2 = x2 + j;
            if(x_1>=0&&x_1<left.cols&&y_1>=0&&y_1<left.rows&&
                    x_2>=0&&x_2<right.cols&&y_2>=0&&y_2<right.rows){
                //sum += fabs(left.at<unsigned char>(y_1,x_1) - right.at<unsigned char>(y_2,x_2));
                unsigned char *p1 = left.ptr<unsigned char>(y_1,x_1);
                unsigned char *p2 = right.ptr<unsigned char>(y_2,x_2);
                sum += fabs(*p1 - *p2);
            }
        }
    }
    return sum;
}
double subPix(double v[MAXS],double mindis){
    int mmindex = -1;
    for(int i = 0;i<MAXS;i++){
        if(v[i]!=-1){
            if(mmindex==-1||v[i]<v[mmindex]){
                mmindex = i;
            }
        }
    }
    //if(mmindex==-1)return -1;
    //return v[mmindex];
    //
    if(mmindex!=-1&&v[mmindex]>mindis)
        return -1;
    return mmindex;
}
void StereoMatch1(Mat &left,Mat &right,Mat &dis,int size,double mindis){
    dis.create(left.size(),CV_32F);
    double value[MAXS];
    for(int i = 0;i<left.rows;i++){
        for(int j = 0;j<left.cols;j++){
            for(int k = 0;k<MAXS;k++){
                value[k] = -1;
                if(j>=k){
                    value[k] = sadValue(j,i,left,j-k,i,right,size);
                }
            }
            //dis.at<float>(i,j) = subPix(value,mindis);
            float *f = dis.ptr<float>(i,j);
            *f = subPix(value,mindis);
        }
    }
}

void testMine(){
    QString leftfilename = QFileDialog::getOpenFileName(
       0,
       "Binocular Calibration - Open Left Image",
       NULL,
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!leftfilename.isNull()) { //用户选择了左图文件
        QString rightfilename = QFileDialog::getOpenFileName(
           0,
           "Binocular Calibration - Open Right Image",
           NULL,
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

            Mat dis,vdisp;
            StereoMatch1(leftgray,rightgray,dis,9,16*1000);
            dis.convertTo(vdisp,CV_8U);
            normalize(vdisp,vdisp,0,255,CV_MINMAX);
            imshow("mine disp",vdisp);
        }
    }
}


void testRealTime(){
    QString leftfilename = QFileDialog::getOpenFileName(
       0,
       "Binocular Calibration - Open Left Image",
       NULL,
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!leftfilename.isNull()) { //用户选择了左图文件
        QString rightfilename = QFileDialog::getOpenFileName(
           0,
           "Binocular Calibration - Open Right Image",
           NULL,
           "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
        if (!rightfilename.isNull()) { //用户选择了左图文件
            Mat leftmat = imread(leftfilename.toUtf8().data());
            Mat rightmat = imread(rightfilename.toUtf8().data());
        }
    }
}


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
//    MainWindow w;
//    w.show();
    //cvNamedWindow("test");
//    testBM();
    testMine();
//    testRealTime();
    
    return a.exec();
}
