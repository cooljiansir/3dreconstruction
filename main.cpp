#include "mainwindow.h"
#include <QApplication>
#include "cv.h"
#include "highgui.h"
#include <QFileDialog>
#include <algorithm>

#include <QDebug>


using namespace cv;
using namespace std;

#define MAXS 30 //最大搜索视差


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
double subPix(double v[MAXS]){
    int mmindex = -1;
    for(int i = 0;i<MAXS;i++){
        if(v[i]!=-1){
            if(mmindex==-1||v[i]<v[mmindex]){
                mmindex = i;
            }
        }
    }
    return mmindex;
}
double conf(double v[MAXS]){
    int mmindex = -1;
    int mmindex2 = -1;
    for(int i = 0;i<MAXS;i++){
        if(v[i]!=-1){
            if(mmindex==-1||v[i]<v[mmindex]){
                mmindex2 = mmindex;
                mmindex = i;
            }
        }
    }
    return (1 - v[mmindex]/(v[mmindex2]+0.00001))*100;
}

void StereoMatch1(Mat &left,Mat &right,Mat &dis,int size){
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
            *f = subPix(value);
        }
    }
}
void StereoMatchC(Mat &left,Mat &right,Mat &dis,int size){
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
            *f = subPix(value);
        }
    }
}

double tempsort[150];
//中值滤波
void filter(Mat &src,Mat &output,int size){
    output.create(src.rows,src.cols,CV_32F);
    int i_,j_,t_i;
    for(int i = 0;i<src.rows;i++){
        for(int j = 0;j<src.cols;j++){
            t_i = 0;
            for(int a = -size;a<=size;a++){
                for(int b = -size;b<=size;b++){
                    i_ = i+a;
                    j_ = j+b;
                    if(i_>=0&&i_<src.rows&&
                            j_>=0&&j_<src.cols){
                        tempsort[t_i++] = src.at<float>(i_,j_);
                    }else{
                        tempsort[t_i++] = -1;
                    }
                }
            }
            sort(tempsort,tempsort+t_i);
            output.at<float>(i,j) = tempsort[t_i/2];
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
            StereoMatch1(leftgray,rightgray,dis,1);
            Mat dis1;
            //filter(dis,dis1,1);
            dis1.convertTo(vdisp,CV_8U);
            normalize(vdisp,vdisp,0,255,CV_MINMAX);
            imshow("mine disp",vdisp);
        }
    }
}

//可变窗口
int mode[100][100];

//sad值函数
double sadValue2(int x1,int y1,Mat &left,int x2,int y2,Mat &right,int size){
    double sum = 0;
    double thre = 500;
    int x_1,y_1,x_2,y_2;
    Vec3b p0 = left.at<Vec3b>(y1,x1);
    for(int i = -size;i<=size;i++){
        for(int j = -size;j<=size;j++){
            mode[50+i][50+j] = 0;
            y_1 = y1 + i;
            x_1 = x1 + j;
            if(x_1>=0&&x_1<left.cols&&y_1>=0&&y_1<left.rows){
                Vec3b *p = left.ptr<Vec3b>(y_1,x_1);
                double di = ((*p)[0] - p0[0])*((*p)[0] - p0[0])
                        + ((*p)[1] - p0[1])*((*p)[1] - p0[1])
                        + ((*p)[2] - p0[2])*((*p)[2] - p0[2]);
                if(di<thre){
                    mode[50+i][50+j] = 1;
                }
            }
        }
    }



    for(int i = -size;i<=size;i++){
        for(int j = -size;j<=size;j++){
            y_1 = y1 + i;
            x_1 = x1 + j;
            y_2 = y2 + i;
            x_2 = x2 + j;
            if(x_1>=0&&x_1<left.cols&&y_1>=0&&y_1<left.rows&&
                    x_2>=0&&x_2<right.cols&&y_2>=0&&y_2<right.rows
                    &&mode[50+i][50+j]){
                //sum += fabs(left.at<unsigned char>(y_1,x_1) - right.at<unsigned char>(y_2,x_2));
                Vec3b *p1 = left.ptr<Vec3b>(y_1,x_1);
                Vec3b *p2 = right.ptr<Vec3b>(y_2,x_2);
                sum += ((*p1)[0] - (*p2)[0])*((*p1)[0] - (*p2)[0])
                        +((*p1)[1] - (*p2)[1])*((*p1)[1] - (*p2)[1])
                        +((*p1)[2] - (*p2)[2])*((*p1)[2] - (*p2)[2]);
            }
        }
    }
    return sum;
}

void StereoMatch2(Mat &left,Mat &right,Mat &dis,int size){
    dis.create(left.size(),CV_32F);
    double value[MAXS];
    for(int i = 0;i<left.rows;i++){
        for(int j = 0;j<left.cols;j++){
            for(int k = 0;k<MAXS;k++){
                value[k] = -1;
                if(j>=k){
                    value[k] = sadValue2(j,i,left,j-k,i,right,size);
                }
            }
            //dis.at<float>(i,j) = subPix(value,mindis);
            float *f = dis.ptr<float>(i,j);
            *f = subPix(value);
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

            Mat dis,vdisp;
            StereoMatch2(leftmat,rightmat,dis,0);
            dis.convertTo(vdisp,CV_8U);
            normalize(vdisp,vdisp,0,255,CV_MINMAX);
            imshow("real time disp",vdisp);
        }
    }
}
double sad3Value(Mat &left,Mat &right,int x1,int y1,int x2,int y2,int size){
    double sum = 0;
    int x_1,y_1,x_2,y_2;
    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;
    for(int i = -size;i<=size;i++){
        for(int j = -size;j<=size;j++){
            y_1 = y1 + i;
            x_1 = x1 + j;
            y_2 = y2 + i;
            x_2 = x2 + j;
            if(x_1>=0&&x_1<left.cols&&y_1>=0&&y_1<left.rows&&
                    x_2>=0&&x_2<right.cols&&y_2>=0&&y_2<right.rows){
                //Vec3b *p1 = left.ptr<Vec3b>(y_1,x_1);
                //Vec3b *p2 = right.ptr<Vec3b>(y_2,x_2);
                unsigned char *p1 = leftptr + (y_1*left.cols + x_1)*3;
                unsigned char *p2 = rightptr + (y_2*right.cols + x_2)*3;
                sum += fabs(p1[0] - p2[0]) +
                        fabs(p1[1] - p2[1])+
                        fabs(p1[2] - p2[2]);
            }
        }
    }
    return sum;
}
double getPix(double v[MAXS]){
    int mmindex = -1;
    for(int i = 0;i<MAXS;i++){
        if(v[i]>0){
            if(mmindex==-1||v[i]<v[mmindex])
                mmindex = i;
        }
    }
    return mmindex;
}
double confV(double v[MAXS]){
    int mmindex = -1;
    int mmindex2;
    for(int i = 0;i<MAXS;i++){
        if(v[i]>0){
            if(mmindex==-1||v[i]<v[mmindex]){
                mmindex2 = mmindex;
                mmindex = i;
            }
        }
    }
    if(mmindex2==-1)
        return -1;
    return (1 - v[mmindex]/(v[mmindex2]+0.00001))*100;
}

double deeptemp[MAXS];
void stereomatchL(Mat &left,Mat &right,Mat &disL,int size){
    disL.create(left.size(),CV_32F);
    float *disptr = (float*)disL.data;
    for(int i = 0;i<left.rows;i++){
        for(int j = 0;j<left.cols;j++){
            for(int k = 0;k<MAXS;k++)
                deeptemp[k] = -1;
            for(int k = 0;k<MAXS;k++){
                if(j>=k){
                    deeptemp[k] = sad3Value(left,right,j,i,j-k,i,size);
                }
            }
            //float *v = disL.ptr<float>(i,j);
            float *v= disptr + i*disL.cols + j;
            (*v) = getPix(deeptemp);
        }
    }
}
void stereomatchR(Mat &left,Mat &right,Mat &disR,int size){
    disR.create(right.size(),CV_32F);
    float *disptr = (float*)disR.data;
    for(int i = 0;i<right.rows;i++){
        for(int j = 0;j<right.cols;j++){
            for(int k = 0;k<MAXS;k++)
                deeptemp[k] = -1;
            for(int k = 0;k<MAXS;k++){
                if(j+k<right.cols){
                    deeptemp[k] = sad3Value(right,left,j,i,j+k,i,size);
                }
            }
//            float *v = disR.ptr<float>(i,j);
            float *v= disptr + i*disR.cols + j;
            (*v) = getPix(deeptemp);
        }
    }
}
void getConf(Mat &left,Mat &right,Mat &disL,Mat &disR,Mat &conf,double cf,int size){
    double deeptemp[MAXS];
    conf.create(left.size(),CV_32F);
    for(int i = 0;i<left.rows;i++){
        for(int j = 0;j<left.cols;j++){
            for(int k  =0;k<MAXS;k++)
                deeptemp[k] = -1;
            for(int k = 0;k<MAXS;k++){
                if(j>=k){
                    deeptemp[k] = sad3Value(left,right,j,i,j-k,i,size);
                }
            }
            float *v = conf.ptr<float>(i,j);
            *v = confV(deeptemp);
            float *disLp = disL.ptr<float>(i,j);
            float *disRp = disR.ptr<float>(i,j);
            if(fabs((*disLp)-(*disRp))<0.00001){
                *v += cf;
            }else {
                *v -= cf;
            }
        }
    }
}
void constant(Mat &disL,Mat &disR,Mat &output){
    output.create(disL.size(),CV_32F);
    for(int i = 0;i<disL.rows;i++){
        for(int j = 0;j<disL.cols;j++){
            float *p1 = disL.ptr<float>(i,j);
            float *p2 = disR.ptr<float>(i,j);
            float *p = output.ptr<float>(i,j);
            if(fabs((*p1)-(*p2))<=1){
                *p = *p1;
            }else *p = -1;
        }
    }
}

void testLocal(){
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

            Mat disL,vdispL;
            stereomatchL(leftmat,rightmat,disL,1);
            disL.convertTo(vdispL,CV_8U);
            normalize(vdispL,vdispL,0,255,CV_MINMAX);
            imshow("dispL",vdispL);
            Mat disR,vdispR;
            stereomatchR(leftmat,rightmat,disR,1);
            disR.convertTo(vdispR,CV_8U);
            normalize(vdispR,vdispR,0,255,CV_MINMAX);
            imshow("dispR",vdispR);

            Mat conf,vconf;
            getConf(leftmat,rightmat,disL,disR,conf,10,1);
            conf.convertTo(vconf,CV_8U);
            normalize(vconf,vconf,0,255,CV_MINMAX);
            imshow("conf",vconf);

            Mat unit,vunit;
            constant(disL,disR,unit);
            unit.convertTo(vunit,CV_8U);
            normalize(vunit,vunit,0,255,CV_MINMAX);
            imshow("constant check",vunit);
        }
    }
}
void dynamicPro(Mat &left,Mat &right,Mat &disL,double q,double c0){
    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;
    disL.create(left.size(),CV_32F);
    float *ot = (float*)disL.data;

    int cols = left.cols;
    int rows = left.rows;
    double *minres = new double[cols*cols];
    char *stepres = new char[cols*cols];

    double v1,v2,v3;
    double p1,p2;
    for(int i = 0;i<rows;i++){
//        qDebug()<<"开始动归"<<endl;
        for(int a = 0;a<cols;a++){
            for(int b = 0;b<cols;b++){
                p1 = *(leftptr+(i*cols+a)*3)
                        + *(leftptr+(i*cols+a)*3+1)
                        + *(leftptr+(i*cols+a)*3+2);
                p2 = *(rightptr+(i*cols+b)*3)
                        + *(rightptr+(i*cols+b)*3+1)
                        + *(rightptr+(i*cols+b)*3+2);
                p1/=3;
                p2/=3;
                v1 = (p1 - p2)*(p1 - p2)/q/q;
                if(a>0&&b>0)v1 += minres[(a-1)*cols+b-1];
                v2 = c0;
                if(b>0)v2 += minres[a*cols+b-1];
                v3 = c0;
                if(a>0)v3 += minres[(a-1)*cols+b];
                if(v1<v2&&a>=b){//加入a>=b约束，即视差不能为负
                    minres[a*cols+b] = v1;
                    stepres[a*cols+b] = 1;
                }else{
                    minres[a*cols+b] = v2;
                    stepres[a*cols+b] = 2;
                }
                if(v3<minres[a*cols+b]){
                    minres[a*cols+b] = v3;
                    stepres[a*cols+b] = 3;
                }
            }
        }
//        qDebug()<<"动归结束"<<endl;
        int a = cols-1;
        int b = cols-1;
        for(int k = 0;k<cols;k++)
            ot[i*cols+k] = -1;
        while(a>=0&&b>=0){
            if(stepres[a*cols+b]==1){
                if(a>=b){
                    ot[i*cols+a] = a - b;
                    if(a-b>MAXS)
                        ot[i*cols+a] = -1;
                }
                else ot[i*cols+a] = -1;
                a--;
                b--;
            }else if(stepres[a*cols+b]==2){
                b--;
            }else{
                ot[i*cols+a] = -1;
                a--;
            }
        }
        //qDebug()<<"完成"<<i+1<<"/"<<rows<<"\r";
    }
    delete []minres;
    delete []stepres;
}
void testDynamic(){
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

            Mat dis,vdisp;
            dynamicPro(leftmat,rightmat,dis,2,100);
//            freopen("log.txt","w",stdout);
//            cout<<dis;
            dis.convertTo(vdisp,CV_8U);
            normalize(vdisp,vdisp,0,255,CV_MINMAX);
            imshow("dynamic ",vdisp);
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
//    testMine();
//    testRealTime();
//    testLocal();
    testDynamic();
    
    return a.exec();
}
