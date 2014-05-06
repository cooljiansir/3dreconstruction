#include "mainwindow.h"
#include <QApplication>
#include "cv.h"
#include "highgui.h"
#include <QFileDialog>
#include <algorithm>

#include <QDebug>


using namespace cv;
using namespace std;

#define MAXS 100 //最大搜索视差


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
            clock_t t = clock();
            bm(leftgray,rightgray,disp,CV_32F);
            qDebug()<<"use time"<<clock()-t<<"ms"<<endl;
            disp.convertTo(vdisp, CV_8U);//, 255/(32*16.));
            normalize(vdisp,vdisp,0,255,CV_MINMAX);
            imshow("dis",vdisp);
//            freopen("log.txt","w",stdout);
            //imwrite("vdisp.png",vdisp);
            //cout<<disp;
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
    if(disR.size()!=disL.size())
        return;
    output.create(disL.size(),CV_32F);
    float *disLptr = (float*)disL.data;
    float *disRptr = (float*)disR.data;
    float *ot = (float*)output.data;
    int rows = disL.rows;
    int cols = disL.cols;

    for(int i = 0;i<rows;i++){
        for(int j = 0;j<cols;j++){
            int p1 = *(disLptr + i*cols+j);
            int r2 = j - p1;
            if(r2>=0&&r2<cols){
                int p2 = *(disRptr+i*cols+r2);
                if(fabs(p1-p2)<=1)
                    *(ot+i*cols+j) = p1;
                else *(ot+i*cols+j) = -1;
            }else{
                *(ot+i*cols+j) = -1;
            }
        }
    }
}
void constant2(Mat &disL, Mat &disR, Mat &output){
    if(disL.size()!=disR.size()){
        return;
    }
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
//从右往左动态规划
void dynamicProR(Mat &left,Mat &right,Mat &dis,double q,double c0){
    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;
    dis.create(left.size(),CV_32F);
    float *ot = (float*)dis.data;

    int rows = left.rows;
    int cols = left.cols;

    double *minres = new double[cols*cols];
    char *stepres = new char[cols*cols];

    double v1,v2,v3;
    double p1,p2;

    for(int i = 0;i<rows;i++){
        for(int a = cols-1;a>=0;a--){
            for(int b = cols-1;b>=0;b--){
                p1 = *(leftptr+(i*cols+a)*3)
                        + *(leftptr+(i*cols+a)*3+1)
                        + *(leftptr+(i*cols+a)*3+2);
                p2 = *(rightptr+(i*cols+b)*3)
                        + *(rightptr+(i*cols+b)*3+1)
                        + *(rightptr+(i*cols+b)*3+2);
                p1/=3;
                p2/=3;
                v1 = (p1 - p2)*(p1 - p2)/q/q;
                if(a<cols-1&&b<cols-1)
                    v1 += minres[(a+1)*cols+b+1];
                v2 = c0;
                if(b<cols-1)
                    v2 += minres[a*cols+b+1];
                v3 = c0;
                if(a<cols-1)
                    v3 += minres[(a+1)*cols+b];

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
        int a = 0,b = 0;
        while(a<cols&&b<cols){
            if(stepres[a*cols+b]==1){
                if(a>=b){
                    ot[i*cols+a] = a - b;
                    if(a-b>=MAXS)ot[i*cols+a] = -1;
                }
                else ot[i*cols+a] = -1;
                a++;
                b++;
            }else if(stepres[a*cols+b]==2){
                b++;
            }else{
                ot[i*cols+a] = -1;
                a++;
            }
        }
    }

    delete []minres;
    delete []stepres;
}
void stereoIteraDP(Mat &left,Mat &right,Mat &dis,int maxdis){
    if(left.size()!=right.size())
        return ;
    Mat leftgray,rightgray;
    Size size = left.size();
    leftgray.create(size,CV_8U);
    rightgray.create(size,CV_8U);
    cvtColor(left,leftgray,CV_BGR2GRAY);
    cvtColor(right,rightgray,CV_BGR2GRAY);


    unsigned char *leftptr = leftgray.data;
    unsigned char *rightptr = rightgray.data;


    int *disint = new int[size.width*size.height];
    int *disint2 = new int[size.width*size.height];

    //initalborder
    for(int i = 0;i<size.height;i++)
        for(int j = 0;j<size.width;j++)
            if(dis.rows>0)disint[i*size.width+j] = dis.at<float>(i,j);
            else disint[i*size.width+j] = 0;

    if(1){
    int T = -1;
    int Tmax = 10;
    int P1 = 40;
    int P2 = 160;
    while(++T<Tmax){
        for(int i = 1;i+1<size.height;i++){
            int *disinti = disint+i*size.width;
            int *disinti_p = disinti-size.width;
            int *disinti_n = disinti+size.width;
            int *disint2i = disint2+i*size.width;
            unsigned char *leftptri = leftptr+i*size.width;
            unsigned char *rightptri = rightptr+i*size.width;
            for(int j = 1;j<size.width;j++){
                int mmin=1<<29;
                int mmindex;
                /*四邻域
                int near[4]={disinti_p[j],disinti_n[j],disinti[j-1],disinti[j+1]};
                for(int d = 0;d<maxdis&&d<=j;d++){
                    int cost = (leftptri[j] - rightptri[j-d])*(leftptri[j] - rightptri[j-d]);
                    for(int ni = 0;ni<4;ni++){
                        if(abs(d-near[ni])==1){
                            cost += P1;
                        }else if(abs(d-near[ni])>1){
                            cost += P2;
                        }
                    }
                    if(cost<mmin){
                        mmin = cost;
                        mmindex = d;
                    }
                }
                */
                //八邻域
                int near[8]={disinti_p[j-1],disinti_p[j],disinti_p[j+1],
                             disinti[j-1],disinti[j+1],
                             disinti_n[j-1],disinti_n[j],disinti_n[j+1]
                            };
                for(int d = 0;d<maxdis&&d<=j;d++){
                    int cost = (leftptri[j] - rightptri[j-d])*(leftptri[j] - rightptri[j-d]);
                    if(T!=0||dis.rows>0){
                        for(int ni = 0;ni<8;ni++){
                            if(abs(d-near[ni])==1){
                                cost += P1;
                            }else if(abs(d-near[ni])>1){
                                cost += P2;
                            }
                        }
                    }
                    if(cost<mmin){
                        mmin = cost;
                        mmindex = d;
                    }
                }
                disint2i[j] = mmindex;
            }
        }

        int *temp = disint;
        disint = disint2;
        disint2 = temp;
    }
    }
    dis.create(size,CV_32F);
    float *disptr = (float*)dis.data;

    for(int i = 0;i<size.height;i++){
        for(int j = 0;j<size.width;j++){
            disptr[i*size.width+j] = disint[i*size.width+j];
        }
    }
    delete []disint;
    delete []disint2;

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

            Mat disL,disR;
//            dynamicPro(leftmat,rightmat,disL,5,20);
//            dynamicProR(leftmat,rightmat,disR,5,20);

            Mat unit,vdisp;
//            constant2(disL,disR,unit);
            stereoIteraDP(leftmat,rightmat,unit,40);
//            freopen("log.txt","w",stdout);
//            cout<<unit;
            unit.convertTo(vdisp,CV_8U);
            normalize(vdisp,vdisp,0,255,CV_MINMAX);
            imshow("dynamic ",vdisp);
        }
    }
}

void improve_DP(Mat &leftmat,Mat &rightmat,Mat &dis,double q,double c0){

    if(leftmat.size()!=rightmat.size())
        return ;
    Size size = leftmat.size();
    //use sgbm algorithm for inital
    Mat leftgray,rightgray;
    leftgray.create(leftmat.size(),CV_8UC1);
    rightgray.create(rightmat.size(),CV_8UC1);
    cvtColor(leftmat,leftgray,CV_BGR2GRAY);
    cvtColor(rightmat,rightgray,CV_BGR2GRAY);

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

    Mat disp;

    sgbm(leftgray,rightgray,disp);

    dis.create(size,CV_32F);
    float *otptr = (float*)dis.data;
    for(int i = 0;i<size.height;i++){
        for(int j = 0;j<size.width;j++){
            otptr[i*size.width+j] = *disp.ptr<short int>(i,j)/16.0;
        }
    }

//    return ;

    short int *disptr_ = (short int*)disp.data;
    unsigned char *leftptr_ = leftgray.data;
    unsigned char *rightptr_ = rightgray.data;


    float *penalize = new float[size.width*size.width];
    char *steps = new char[size.width*size.width];

    otptr = (float *)dis.data;
    for(int i = 0;i<size.height;i++){
        disptr_ += size.width;
        leftptr_ += size.width;
        rightptr_ += size.width;
        otptr += size.width;
        for(int j = 0;j<size.width;j++){
            int d = -1;
            if(disptr_[j]>0){
                d = disptr_[j]/16;
                penalize[j*size.width+d] = 0;
            }
            for(int k = 0;k<size.width;k++){
                if(d==-1||k!=j-d){
                    float a1 = (leftptr_[j] - rightptr_[k])*(leftptr_[j] - rightptr_[k])/q/q;
                    if(j>0&&k>0)a1 += penalize[(j-1)*size.width+k-1];

                    float a2 = c0;
                    if(k>0)a2 += penalize[j*size.width+k-1];

                    float a3 = c0;
                    if(j>0)a3 += penalize[(j-1)*size.width+k];

                    if(a1<a2){
                        if(a3<a1){
                            steps[j*size.width+k] = 3;
                            penalize[j*size.width+k] = a3;
                        }else{
                            steps[j*size.width+k] = 1;
                            penalize[j*size.width+k] = a1;
                        }
                    }else{
                        if(a3<a2){
                            steps[j*size.width+k] = 3;
                            penalize[j*size.width+k] = a3;
                        }else{
                            steps[j*size.width+k] = 2;
                            penalize[j*size.width+k] = a2;
                        }
                    }
                }
            }
        }
        {
            int a1=size.width-1,a2=size.width-1;
            while(a1>=0&&a2>=0){
                if(disptr_[a1]>0){
                    int d = disptr_[a1]/16;
                    a1--;
                    a2 = a1 - d - 1;
                }else{
                    if(steps[a1*size.width+a2]==1){
                        otptr[a1] = a1 - a2;
                        a1--;
                        a2--;
                    }else if(steps[a1*size.width+a2]==2){
                        a2--;
                    }else{
                        a1--;
                    }
                }
            }
        }
    }

    delete []penalize;
    delete []steps;
}

void test_dp(){
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
            improve_DP(leftmat,rightmat,dis,5,20);

            dis.convertTo(vdisp,CV_8U);
            normalize(vdisp,vdisp,0,255,CV_MINMAX);
            imshow("dynamic ",vdisp);
        }
    }
}

void stereoSemi(Mat &leftmat,Mat &rightmat,Mat &dis,double q,double p1,double p2,int maxdis){
    if(leftmat.size()!=rightmat.size())
        return ;
    Size size = leftmat.size();
    //use sgbm algorithm for inital
    Mat leftgray,rightgray;
    leftgray.create(leftmat.size(),CV_8UC1);
    rightgray.create(rightmat.size(),CV_8UC1);
    cvtColor(leftmat,leftgray,CV_BGR2GRAY);
    cvtColor(rightmat,rightgray,CV_BGR2GRAY);


    dis.create(size,CV_32F);
    float *disptr = (float*)dis.data;
    unsigned char *leftptr  = leftgray.data;
    unsigned char *rightptr = rightgray.data;

    double *Lbuff = new double[size.width*maxdis];
    int mmin_d;

    for(int i = 0;i<size.height;i++){
        unsigned char *leftptri = leftptr+i*size.width;
        unsigned char *rightptri = rightptr+i*size.width;
        float *disptri = disptr + i*size.width;
        for(int j = 0;j<size.width;j++){
            unsigned char *leftptrij = leftptri+j;
            unsigned char *rightptrij = rightptri + j;
            double *Lbuffj = Lbuff + j*maxdis;
            double *Lbuffj_1 = j>0?Lbuffj - maxdis:0;
            for(int d = 0;d<maxdis;d++)
                Lbuffj[d] = -1;
            for(int d = 0;d<maxdis&&d<=j;d++){
                Lbuffj[d] = (*leftptrij - rightptrij[-d])*(*leftptrij - rightptrij[-d])/q/q;
                double mmin = -1;
                if(j>d)
                    mmin = Lbuffj_1[d];
                if(d>0&&(mmin<0||Lbuffj_1[d-1]+p1<mmin))
                    mmin = Lbuffj_1[d-1]+p1;
                if(j>d+1&&(mmin<0||Lbuffj_1[d+1]+p1<mmin))
                    mmin = Lbuffj_1[d+1]+p1;
                if(j>0&&mmin_d>=0&&Lbuffj_1[mmin_d]+p2<mmin)
                    mmin = Lbuffj_1[mmin_d]+p2;
                if(mmin>0)Lbuffj[d] += mmin;
            }
            int rankmin = -1;
            for(int k = 0;k<maxdis;k++)
                if(Lbuffj[k]>0&&(rankmin==-1||Lbuffj[k]<Lbuffj[rankmin]))
                    rankmin = k;
            disptri[j] = rankmin;
            mmin_d = rankmin;
        }
    }
    delete []Lbuff;
}
void stereoSemi1(Mat &leftmat,Mat &rightmat,Mat &dis,double q,double p1,double p2,int maxdis){
    if(leftmat.size()!=rightmat.size())
        return ;
    Size size = leftmat.size();
    //use sgbm algorithm for inital
    Mat leftgray,rightgray;
    leftgray.create(leftmat.size(),CV_8UC1);
    rightgray.create(rightmat.size(),CV_8UC1);
    cvtColor(leftmat,leftgray,CV_BGR2GRAY);
    cvtColor(rightmat,rightgray,CV_BGR2GRAY);


    dis.create(size,CV_32F);
    float *disptr = (float*)dis.data;
    unsigned char *leftptr  = leftgray.data;
    unsigned char *rightptr = rightgray.data;

    double *Lbuff = new double[size.height*maxdis];
    for(int j = 0;j<size.width;j++){
        unsigned char *leftptrj = leftptr + j;
        unsigned char *rightptrj = rightptr + j;
        unsigned char *leftptrij = leftptrj;
        unsigned char *rightptrij = rightptrj;
        double *Lbuffi = Lbuff;
        double *Lbuffi_1 = Lbuff - maxdis;
        for(int i = 0;i<size.height;i++){
            unsigned char *rightptrijd = rightptrij;
            int mmind = -1;
            for(int d = 0;d<maxdis&&d<=j;d++){
                Lbuffi[d] = (leftptr[i*size.width+j] - rightptr[i*size.width+j-d])
                        *(leftptr[i*size.width+j] - rightptr[i*size.width+j-d])/q/q;
                rightptrijd -= size.width;
                double mmin = -1;
                if(i>0){
                    if(mmin==-1||Lbuffi_1[d]<mmin)
                        mmin = Lbuffi_1[d];
                    if(j>d)
                        if(mmin==-1||Lbuffi_1[d+1]+p1<mmin)
                            mmin = Lbuffi_1[d+1]+p1;
                    if(d>0)
                        if(mmin==-1||Lbuffi_1[d-1]+p1<mmin)
                            mmin = Lbuffi_1[d-1]+p1;
                }
                if(mmin>0)
                    Lbuffi[d] += mmin;
                if(mmind==-1||Lbuffi[d]<Lbuffi[mmind])
                    mmind = d;
            }
            disptr[i*size.width+j] = mmind;

            Lbuffi += maxdis;
            Lbuffi_1 += maxdis;
        }
    }
    delete []Lbuff;
}
void swap_d(double *&a,double *&b){
    double *t = a;
    a = b;
    b = t;
}
void stereoSemi2(Mat &leftmat,Mat &rightmat,Mat &dis,double q,double p1,double p2,int maxdis){
    if(leftmat.size()!=rightmat.size())
        return ;
    Size size = leftmat.size();
    //use sgbm algorithm for inital
    Mat leftgray,rightgray;
    leftgray.create(leftmat.size(),CV_8UC1);
    rightgray.create(rightmat.size(),CV_8UC1);
    cvtColor(leftmat,leftgray,CV_BGR2GRAY);
    cvtColor(rightmat,rightgray,CV_BGR2GRAY);


    dis.create(size,CV_32F);
    float *disptr = (float*)dis.data;
    unsigned char *leftptr  = leftgray.data;
    unsigned char *rightptr = rightgray.data;

    double *Lbuff = new double[size.width*maxdis*2];
    double *Lbuff1 = Lbuff,*Lbuff2 = Lbuff + size.width*maxdis;
    double *Lmin = new double[size.width*2];
    double *Lmin1 = Lmin;
    double *Lmin2 = Lmin+size.width;

    //inital the first line
    for(int j = 0;j<size.width;j++){
        double mmin = -1;
        for(int d = 0;d<maxdis&&d<=j;d++){
            Lbuff1[j*maxdis+d] = (leftptr[j] - rightptr[j-d])*(leftptr[j] - rightptr[j-d])/q/q;
            if(mmin<0||Lbuff1[j*maxdis+d]<mmin)
                mmin = Lbuff1[j*maxdis+d];
        }
        Lmin1[j] = mmin;
    }
    for(int i = 1;i<size.height;i++){
        for(int j = 0;j<size.width;j++)
            Lmin2[j] = -1;
        for(int j = 0;j<size.width;j++){
            int mmind = -1;
            for(int d = 0;d<maxdis&&d<=j;d++){
                double mmin = -1;

                if(mmin<0||Lbuff1[j*maxdis+d]<mmin)
                    mmin = Lbuff1[j*maxdis+d];
                if(j>d)
                    if(mmin<0||Lbuff1[j*maxdis+d+1]+p1<mmin)
                        mmin = Lbuff1[j*maxdis+d+1]+p1;
                if(d>0)
                    if(mmin<0||Lbuff1[j*maxdis+d-1]+p1<mmin)
                        mmin = Lbuff1[j*maxdis+d-1]+p1;
                if(Lmin1[j]>=0)
                    if(mmin<0||Lmin1[j]+p2<mmin)
                        mmin = Lmin1[j]+p2;

                Lbuff2[j*maxdis+d] = (leftptr[i*size.width+j] - rightptr[i*size.width+j-d])*
                        (leftptr[i*size.width+j] - rightptr[i*size.width+j-d])/q/q;
                if(mmin>0)
                    Lbuff2[j*maxdis+d] += mmin;
                if(mmind==-1||Lbuff2[j*maxdis+d]<Lbuff2[j*maxdis+mmind])
                    mmind = d;
            }
            disptr[i*size.width+j] = mmind;
            if(mmind!=-1)
                Lmin2[j] = Lbuff2[j*maxdis+mmind];
        }
        swap_d(Lbuff1,Lbuff2);
        swap_d(Lmin1,Lmin2);
    }

    delete []Lbuff;
    delete []Lmin;
}


//4 directions semi
void stereoSemi4(Mat &leftmat,Mat &rightmat,Mat &dis,double q,int p1,int p2,int maxdis){
    if(leftmat.size()!=rightmat.size())
        return ;
    Size size = leftmat.size();
    //use sgbm algorithm for inital
    Mat leftgray,rightgray;
    leftgray.create(leftmat.size(),CV_8UC1);
    rightgray.create(rightmat.size(),CV_8UC1);
    cvtColor(leftmat,leftgray,CV_BGR2GRAY);
    cvtColor(rightmat,rightgray,CV_BGR2GRAY);


    dis.create(size,CV_32F);
    float *disptr = (float*)dis.data;
    for(int i = 0;i<size.height;i++)
        for(int j = 0;j<size.width;j++)
            disptr[i*size.width+j] = -1;
    unsigned char *leftptr  = leftgray.data;
    unsigned char *rightptr = rightgray.data;

    /* 1  2  3
     *  \ | /
     * 0-
    */
    double *Lr = new double[size.width*maxdis*7];
    double *minLr = new double[size.width*6];

    double  *Lr0  = Lr,
            *Lr11 = Lr+maxdis*size.width,
            *Lr12 = Lr+maxdis*size.width*2,
            *Lr21 = Lr+maxdis*size.width*3,
            *Lr22 = Lr+maxdis*size.width*4,
            *Lr31 = Lr+maxdis*size.width*5,
            *Lr32 = Lr+maxdis*size.width*6;
    double  *minLr11 = minLr,
            *minLr12 = minLr + size.width,
            *minLr21 = minLr + size.width*2,
            *minLr22 = minLr + size.width*3,
            *minLr31 = minLr + size.width*4,
            *minLr32 = minLr + size.width*5;
    double minLr00;

    //init the first line
    for(int j = 0;j<size.width;j++){
        unsigned char *leftptr0j = leftptr + j;
        unsigned char *rightptr0j = rightptr + j;
        double mmin = -1;
        for(int d = 0;d<maxdis&&d<=j;d++){
            double c = (*leftptr0j - rightptr0j[-d])*(*leftptr0j - rightptr0j[-d])/q/q;
            Lr11[j*maxdis+d] = Lr21[j*maxdis+d] = Lr31[j*maxdis+d] = c;
            if(mmin==-1||c<mmin)
                mmin = c;
        }
        minLr11[j] = minLr21[j] = minLr31[j] = mmin;
    }

    for(int i = 1;i<size.height;i++){
        unsigned char *leftptri = leftptr +i*size.width;
        unsigned char *rightptri = rightptr + i*size.width;

        for(int j = 0;j<size.width;j++)
            minLr12[j] = minLr22[j] = minLr32[j] = -1;
        for(int j = 0;j<size.width;j++){
            unsigned char *leftptrij = leftptri + j;
            unsigned char *rightptrij = rightptri + j;
            double *Lr0j    = Lr0 +j*maxdis,
                   *Lr0j_1  = j>0?Lr0j-maxdis:0,
                   *Lr11j_1 = j>0?Lr11+(j-1)*maxdis:0,
                   *Lr12j   = Lr12+j*maxdis,
                   *Lr21j   = Lr21+j*maxdis,
                   *Lr22j   = Lr22+j*maxdis,
                   *Lr31j_1 = j+1<size.width?Lr31+(j+1)*maxdis:0,
                   *Lr32j   = Lr32+j*maxdis;
            double mminv=-1;
            int mmindex=-1;
            double mminL00_ = -1;
            double mminL1_ = -1;
            double mminL2_ = -1;
            double mminL3_ = -1;
            for(int d = 0;d<maxdis&&d<=j;d++){
                double minL0 =-1,minL1 = -1,minL2 = -1,minL3 = -1;
                if(j>d){
                    if(minL0<0||Lr0j_1[d]<minL0)
                        minL0 = Lr0j_1[d];
                    if(minL1<0||Lr11j_1[d]<minL1)
                        minL1 = Lr11j_1[d];
                    if(minL2<0||Lr21j[d+1]+p1<minL2)
                        minL2 = Lr21j[d+1]+p1;
                }
                if(j>d+1){
                    if(minL0<0||Lr0j_1[d+1]+p1<minL0)
                        minL0 = Lr0j_1[d+1]+p1;
                    if(minL1<0||Lr11j_1[d+1]+p1<minL1)
                        minL1 = Lr11j_1[d+1]+p1;
                }
                if(d>0){
                    if(minL0<0||Lr0j_1[d-1]+p1<minL0)
                        minL0 = Lr0j_1[d-1]+p1;
                    if(minL1<0||Lr11j_1[d-1]+p1<minL1)
                        minL1 = Lr11j_1[d-1]+p1;
                    if(minL2<0||Lr21j[d-1]+p1<minL2)
                        minL2 = Lr21j[d-1]+p1;
                }

                if(minL2<0||Lr21j[d]<minL2)
                    minL2 = Lr21j[d];
                if(j+1<size.width){
                    if(minL3<0||Lr31j_1[d]<minL3)
                        minL3 = Lr31j_1[d];
                    if(d>0)
                        if(minL3<0||Lr31j_1[d-1]+p1<minL3)
                            minL3 = Lr31j_1[d-1]+p1;
                    if(minL3<0||Lr31j_1[d+1]+p1<minL3)
                        minL3 = Lr31j_1[d+1]+p1;
                }

                if(j>0){
                    if(minLr00>=0)
                        if(minL0<0||minLr00+p2<minL0)
                            minL0 = minLr00+p2;
                }
                if(j>0)
                    if(minLr11[j-1]>=0)
                        if(minL1<0||minLr11[j-1]+p2<minL1)
                            minL1 = minLr11[j-1] + p2;
                if(minLr21[j]>=0)
                    if(minL2<0||minLr21[j]+p2<minL2)
                        minL2 = minLr21[j] + p2;
                if(j+1<size.width)
                    if(minL3<0||minLr31[j+1]+p2<minL3)
                        minL3 = minLr31[j+1]+p2;

                double c = (*leftptrij - rightptrij[-d])*(*leftptrij - rightptrij[-d])/q/q;

                if(minL0>0)
                    minL0 +=c;
                else minL0 = c;
                if(minL1>0)
                    minL1 +=c;
                else minL1 = c;
                if(minL2>0)
                    minL2 +=c;
                else minL2 = c;
                if(minL3>0)
                    minL3 +=c;
                else minL3 = c;
                Lr0j[d] = minL0;
                Lr12j[d] = minL1;
                Lr22j[d] = minL2;
                Lr32j[d] = minL3;
                if(mminL00_<0||minL0<mminL00_)
                    mminL00_ = minL0;
                if(mminL1_<0||minL1<mminL1_)
                    mminL1_ = minL1;
                if(mminL2_<0||minL2<mminL2_)
                    mminL2_ = minL2;
                if(mminL3_<0||minL3<mminL3_)
                    mminL3_ = minL3;
                if(mminv<0||(minL0+minL1+minL2+minL3)<mminv){
                    mminv = minL0+minL1+minL2+minL3;
                    mmindex = d;
                }
                /*if(mminv<0||minL3<mminv){
                    mminv = minL3;
                    mmindex = d;
                }*/
            }
            disptr[i*size.width+j] = mmindex;
            minLr00 = mminL00_;
            minLr12[j] = mminL1_;
            minLr22[j] = mminL2_;
            minLr32[j] = mminL3_;
        }
        swap_d(Lr11,Lr12);
        swap_d(Lr21,Lr22);
        swap_d(Lr31,Lr32);
        swap_d(minLr11,minLr12);
        swap_d(minLr21,minLr22);
        swap_d(minLr31,minLr32);

    }

    delete []Lr;
    delete []minLr;

}

//4 directions semi
void stereoSemibm(Mat &leftmat,Mat &rightmat,Mat &dis,int p1,int p2,int maxdis,int winsize){
    if(leftmat.size()!=rightmat.size())
        return ;
//    double q = (winsize+1)*(winsize+1);
    Size size = leftmat.size();
    //use sgbm algorithm for inital
    Mat leftgray,rightgray;
    leftgray.create(leftmat.size(),CV_8UC1);
    rightgray.create(rightmat.size(),CV_8UC1);
    cvtColor(leftmat,leftgray,CV_BGR2GRAY);
    cvtColor(rightmat,rightgray,CV_BGR2GRAY);


    dis.create(size,CV_32F);
    float *disptr = (float*)dis.data;
    for(int i = 0;i<size.height;i++)
        for(int j = 0;j<size.width;j++)
            disptr[i*size.width+j] = -1;
    unsigned char *leftptr  = leftgray.data;
    unsigned char *rightptr = rightgray.data;

    /* 1  2  3
     *  \ | /
     * 0-
    */
    double *Lr = new double[size.width*maxdis*7];
    double *minLr = new double[size.width*6];
    double *aline = new double[size.width*maxdis];
    double *tempcost = new double[maxdis];

    double  *Lr0  = Lr,
            *Lr11 = Lr+maxdis*size.width,
            *Lr12 = Lr+maxdis*size.width*2,
            *Lr21 = Lr+maxdis*size.width*3,
            *Lr22 = Lr+maxdis*size.width*4,
            *Lr31 = Lr+maxdis*size.width*5,
            *Lr32 = Lr+maxdis*size.width*6;
    double  *minLr11 = minLr,
            *minLr12 = minLr + size.width,
            *minLr21 = minLr + size.width*2,
            *minLr22 = minLr + size.width*3,
            *minLr31 = minLr + size.width*4,
            *minLr32 = minLr + size.width*5;
    double minLr00;
    //init aline
    for(int j = 0;j<size.width;j++){
        for(int d = 0;d<maxdis&&d<=j;d++){
            double sum = 0;
            for(int i = 0;i<winsize*2+1;i++){
                sum += fabs(leftptr[i*size.width+j] - rightptr[i*size.width+j-d]);
            }
            aline[j*maxdis+d] = sum;
        }
    }
    //init the first line
    for(int j = winsize;j+winsize<size.width;j++){
        double mmin = -1;
        for(int d = 0;d<maxdis&&d+winsize<=j;d++){
            //double c = (*leftptr0j - rightptr0j[-d])*(*leftptr0j - rightptr0j[-d])/q/q;
            //Lr11[j*maxdis+d] = Lr21[j*maxdis+d] = Lr31[j*maxdis+d] = c;
            if(j>d+winsize)
                tempcost[d] = tempcost[d] - aline[(j-winsize-1)*maxdis+d] + aline[(j+winsize)*maxdis+d];
            else {
                tempcost[d] = 0;
                for(int k = j-winsize;k<=j+winsize;k++)
                    tempcost[d] += aline[k*maxdis+d];
            }
            double c = tempcost[d];
            if(mmin==-1||c<mmin)
                mmin = c;
        }
        minLr11[j] = minLr21[j] = minLr31[j] = mmin;
    }
    for(int i = winsize+1;i+winsize<size.height;i++){
        unsigned char *leftptri = leftptr +i*size.width;
        unsigned char *rightptri = rightptr + i*size.width;

        for(int j = winsize;j+winsize<size.width;j++)
            minLr12[j] = minLr22[j] = minLr32[j] = -1;
        for(int j = 0;j<size.width;j++){
            for(int d = 0;d<maxdis&&d<=j;d++){
                aline[j*maxdis+d] -= fabs((leftptr[(i-1-winsize)*size.width+j]
                        - rightptr[(i-1-winsize)*size.width+j-d]));
                aline[j*maxdis+d] += fabs((leftptr[(i+winsize)*size.width+j]
                        - rightptr[(i+winsize)*size.width+j-d]));
            }
        }

        for(int j = winsize;j+winsize<size.width;j++){
            unsigned char *leftptrij = leftptri + j;
            unsigned char *rightptrij = rightptri + j;
            double *Lr0j    = Lr0 +j*maxdis,
                   *Lr0j_1  = j>winsize?Lr0j-maxdis:0,
                   *Lr11j_1 = j>winsize?Lr11+(j-1)*maxdis:0,
                   *Lr12j   = Lr12+j*maxdis,
                   *Lr21j   = Lr21+j*maxdis,
                   *Lr22j   = Lr22+j*maxdis,
                   *Lr31j_1 = j+winsize+1<size.width?Lr31+(j+1)*maxdis:0,
                   *Lr32j   = Lr32+j*maxdis;
            double mminv=-1;
            int mmindex=-1;
            double mminL00_ = -1;
            double mminL1_ = -1;
            double mminL2_ = -1;
            double mminL3_ = -1;
            for(int d = 0;d<maxdis&&d+winsize<=j;d++){
                double minL0 =-1,minL1 = -1,minL2 = -1,minL3 = -1;
                if(j>d){
                    if(j>winsize){
                        if(minL0<0||Lr0j_1[d]<minL0)
                            minL0 = Lr0j_1[d];
                        if(minL1<0||Lr11j_1[d]<minL1)
                            minL1 = Lr11j_1[d];
                    }
                    if(minL2<0||Lr21j[d+1]+p1<minL2)
                        minL2 = Lr21j[d+1]+p1;
                }
                if(j>d+1&&j>winsize){
                    if(minL0<0||Lr0j_1[d+1]+p1<minL0)
                        minL0 = Lr0j_1[d+1]+p1;
                    if(minL1<0||Lr11j_1[d+1]+p1<minL1)
                        minL1 = Lr11j_1[d+1]+p1;
                }
                if(d>0){
                    if(j>winsize){
                        if(minL0<0||Lr0j_1[d-1]+p1<minL0)
                            minL0 = Lr0j_1[d-1]+p1;
                        if(minL1<0||Lr11j_1[d-1]+p1<minL1)
                           minL1 = Lr11j_1[d-1]+p1;
                    }
                    if(minL2<0||Lr21j[d-1]+p1<minL2)
                        minL2 = Lr21j[d-1]+p1;
                }

                if(minL2<0||Lr21j[d]<minL2)
                    minL2 = Lr21j[d];

                if(j+1+winsize<size.width){
                    if(minL3<0||Lr31j_1[d]<minL3)
                        minL3 = Lr31j_1[d];
                    if(d>0)
                        if(minL3<0||Lr31j_1[d-1]+p1<minL3)
                            minL3 = Lr31j_1[d-1]+p1;
                    if(minL3<0||Lr31j_1[d+1]+p1<minL3)
                        minL3 = Lr31j_1[d+1]+p1;
                }
                if(j>winsize){
                    if(minLr00>=0)
                        if(minL0<0||minLr00+p2<minL0)
                            minL0 = minLr00+p2;
                }
                if(j>winsize)
                    if(minLr11[j-1]>=0)
                        if(minL1<0||minLr11[j-1]+p2<minL1)
                            minL1 = minLr11[j-1] + p2;
                if(minLr21[j]>=0)
                    if(minL2<0||minLr21[j]+p2<minL2)
                        minL2 = minLr21[j] + p2;
                if(j+1+winsize<size.width)
                    if(minL3<0||minLr31[j+1]+p2<minL3)
                        minL3 = minLr31[j+1]+p2;

                //double c = (*leftptrij - rightptrij[-d])*(*leftptrij - rightptrij[-d]);
                double c;
                if(j>d+winsize)
                    c = tempcost[d] - aline[(j-winsize-1)*maxdis+d] + aline[(j+winsize)*maxdis+d];
                else {
                    c = 0;
                    for(int k = j-winsize;k<=j+winsize;k++)
                        c += aline[k*maxdis+d];
                }
                tempcost[d] = c;

                if(minL0>0)
                    minL0 +=c;
                else minL0 = c;
                if(minL1>0)
                    minL1 +=c;
                else minL1 = c;
                if(minL2>0)
                    minL2 +=c;
                else minL2 = c;
                if(minL3>0)
                    minL3 +=c;
                else minL3 = c;
                Lr0j[d] = minL0;
                Lr12j[d] = minL1;
                Lr22j[d] = minL2;
                Lr32j[d] = minL3;
                if(mminL00_<0||minL0<mminL00_)
                    mminL00_ = minL0;
                if(mminL1_<0||minL1<mminL1_)
                    mminL1_ = minL1;
                if(mminL2_<0||minL2<mminL2_)
                    mminL2_ = minL2;
                if(mminL3_<0||minL3<mminL3_)
                    mminL3_ = minL3;
                if(mminv<0||(minL0+minL1+minL2+minL3)<mminv){
                    mminv = minL0+minL1+minL2+minL3;
                    mmindex = d;
                }
                /*if(mminv<0||minL2<mminv){
                    mminv = minL2;
                    mmindex = d;
                }*/
            }
            disptr[i*size.width+j] = mmindex;
            minLr00 = mminL00_;
            minLr12[j] = mminL1_;
            minLr22[j] = mminL2_;
            minLr32[j] = mminL3_;
        }
        swap_d(Lr11,Lr12);
        swap_d(Lr21,Lr22);
        swap_d(Lr31,Lr32);
        swap_d(minLr11,minLr12);
        swap_d(minLr21,minLr22);
        swap_d(minLr31,minLr32);

    }

    delete []Lr;
    delete []minLr;
    delete []aline;
    delete []tempcost;

}

//4 directions semi
void stereoSemi_AW(Mat &leftmat,Mat &rightmat,Mat &dis,int p1,int p2,int maxdis,int winsize){
    if(leftmat.size()!=rightmat.size())
        return ;
//    double q = (winsize+1)*(winsize+1);
    double yc=7,yg=36;
    Size size = leftmat.size();

    unsigned char *leftptr = leftmat.data;
    unsigned char *rightptr = rightmat.data;

    dis.create(size,CV_32F);
    float *disptr = (float*)dis.data;
    for(int i = 0;i<size.height;i++)
        for(int j = 0;j<size.width;j++)
            disptr[i*size.width+j] = -1;


    /* 1  2  3
     *  \ | /
     * 0-
    */
    double *Lr = new double[size.width*maxdis*7];
    double *minLr = new double[size.width*6];
    double *w1buff = new double[(2*winsize+1)*(2*winsize+1)*size.width];
    double *w2buff = new double[(2*winsize+1)*(2*winsize+1)*size.width];

    double  *Lr0  = Lr,
            *Lr11 = Lr+maxdis*size.width,
            *Lr12 = Lr+maxdis*size.width*2,
            *Lr21 = Lr+maxdis*size.width*3,
            *Lr22 = Lr+maxdis*size.width*4,
            *Lr31 = Lr+maxdis*size.width*5,
            *Lr32 = Lr+maxdis*size.width*6;
    double  *minLr11 = minLr,
            *minLr12 = minLr + size.width,
            *minLr21 = minLr + size.width*2,
            *minLr22 = minLr + size.width*3,
            *minLr31 = minLr + size.width*4,
            *minLr32 = minLr + size.width*5;
    double minLr00;

    //init the first line
    //cost prepare
    for(int j = winsize;j+winsize<size.width;j++){
        int p = (winsize*size.width + j)*3;
        int windex = j*(2*winsize+1)*(2*winsize+1);
        for(int i1 = -winsize;i1<=winsize;i1++){
            for(int j1 = -winsize;j1<=winsize;j1++){
                int q = ((winsize+i1)*size.width + j+j1)*3;
                double w1 =
                        exp(-sqrt((leftptr[p] - leftptr[q])*(leftptr[p] - leftptr[q])+
                                  (leftptr[p+1] - leftptr[q+1])*(leftptr[p+1] - leftptr[q+1])+
                                  (leftptr[p+2] - leftptr[q+2])*(leftptr[p+2] - leftptr[q+2]))/yc
                        -sqrt(i1*i1+j1*j1)/yg);
                double w2 =
                        exp(-sqrt((rightptr[p] - rightptr[q])*(rightptr[p] - rightptr[q])+
                                     (rightptr[p+1] - rightptr[q+1])*(rightptr[p+1] - rightptr[q+1])+
                                     (rightptr[p+2] - rightptr[q+2])*(rightptr[p+2] - rightptr[q+2]))/yc
                        -sqrt(i1*i1+j1*j1)/yg);
                w1buff[windex] = w1;
                w2buff[windex] = w2;
                windex++;
            }
        }
    }
    for(int j = winsize;j+winsize<size.width;j++){
        double mmin = -1;
        for(int d = 0;d<maxdis&&d+winsize<=j;d++){
            //double c = (*leftptr0j - rightptr0j[-d])*(*leftptr0j - rightptr0j[-d])/q/q;
            //Lr11[j*maxdis+d] = Lr21[j*maxdis+d] = Lr31[j*maxdis+d] = c;
            //double c = tempcost[d];

            double sum = 0;
            double sumw = 0;
            int p = (winsize*size.width + j)*3;
            int p_ = p-3*d;
            int w1index = j*(2*winsize+1)*(2*winsize+1);
            int w2index = (j-d)*(2*winsize+1)*(2*winsize+1);
            for(int i1 = -winsize;i1<=winsize;i1++){
                for(int j1 = -winsize;j1<=winsize;j1++){
                    int q = ((winsize+i1)*size.width + j+j1)*3;
                    int q_ = q-d*3;
                    double e1 = fabs(leftptr[q]-rightptr[q_])+
                            fabs(leftptr[q+1]-rightptr[q_+1])+
                            fabs(leftptr[q+2]-rightptr[q_+2]);

                    double w1 = w1buff[w1index++];
                    double w2 = w2buff[w2index++];
                    sum += w1*w2*e1;
                    sumw += w1*w2;
                }
            }
            double c = sum/sumw;
            if(mmin==-1||c<mmin)
                mmin = c;
        }
        minLr11[j] = minLr21[j] = minLr31[j] = mmin;
    }
    for(int i = winsize+1;i+winsize<size.height;i++){
        unsigned char *leftptri = leftptr +i*size.width;
        unsigned char *rightptri = rightptr + i*size.width;

        for(int j = winsize;j+winsize<size.width;j++)
            minLr12[j] = minLr22[j] = minLr32[j] = -1;

        for(int j = winsize;j+winsize<size.width;j++){
            int p = (i*size.width + j)*3;
            int windex = j*(2*winsize+1)*(2*winsize+1);
            for(int i1 = -winsize;i1<=winsize;i1++){
                for(int j1 = -winsize;j1<=winsize;j1++){
                    int q = ((i+i1)*size.width + j+j1)*3;
                    /*double w1 =
                            exp(-sqrt((leftptr[p] - leftptr[q])*(leftptr[p] - leftptr[q])+
                                      (leftptr[p+1] - leftptr[q+1])*(leftptr[p+1] - leftptr[q+1])+
                                      (leftptr[p+2] - leftptr[q+2])*(leftptr[p+1] - leftptr[q+2]))/yc
                            -sqrt(i1*i1+j1*j1)/yg);
                    double w2 =
                            exp(-sqrt((rightptr[p] - rightptr[q])*(rightptr[p] - rightptr[q])+
                                         (rightptr[p+1] - rightptr[q+1])*(rightptr[p+1] - rightptr[q+1])+
                                         (rightptr[p+2] - rightptr[q+2])*(rightptr[p+2] - rightptr[q+2]))/yc
                            -sqrt(i1*i1+j1*j1)/yg);
                            */
                    double w1 =
                            exp(-(fabs(leftptr[p] - leftptr[q])+
                                fabs(leftptr[p+1] - leftptr[q+1])+
                                fabs(leftptr[p+2] - leftptr[q+2]))/yc
                            -sqrt(i1*i1+j1*j1)/yg);
                    double w2 =
                            exp(-(fabs(rightptr[p] - rightptr[q])+
                                  fabs(rightptr[p+1] - rightptr[q+1])+
                                  fabs(rightptr[p+2] - rightptr[q+2]))/yc
                            -sqrt(i1*i1+j1*j1)/yg);
                    w1buff[windex] = w1;
                    w2buff[windex] = w2;
                    windex++;
                }
            }
        }
        for(int j = winsize;j+winsize<size.width;j++){
            double *Lr0j    = Lr0 +j*maxdis,
                   *Lr0j_1  = j>winsize?Lr0j-maxdis:0,
                   *Lr11j_1 = j>winsize?Lr11+(j-1)*maxdis:0,
                   *Lr12j   = Lr12+j*maxdis,
                   *Lr21j   = Lr21+j*maxdis,
                   *Lr22j   = Lr22+j*maxdis,
                   *Lr31j_1 = j+winsize+1<size.width?Lr31+(j+1)*maxdis:0,
                   *Lr32j   = Lr32+j*maxdis;
            double mminv=-1;
            int mmindex=-1;
            double mminL00_ = -1;
            double mminL1_ = -1;
            double mminL2_ = -1;
            double mminL3_ = -1;
            for(int d = 0;d<maxdis&&d+winsize<=j;d++){
                double minL0 =-1,minL1 = -1,minL2 = -1,minL3 = -1;
                if(j>d){
                    if(j>winsize){
                        if(minL0<0||Lr0j_1[d]<minL0)
                            minL0 = Lr0j_1[d];
                        if(minL1<0||Lr11j_1[d]<minL1)
                            minL1 = Lr11j_1[d];
                    }
                    if(minL2<0||Lr21j[d+1]+p1<minL2)
                        minL2 = Lr21j[d+1]+p1;
                }
                if(j>d+1&&j>winsize){
                    if(minL0<0||Lr0j_1[d+1]+p1<minL0)
                        minL0 = Lr0j_1[d+1]+p1;
                    if(minL1<0||Lr11j_1[d+1]+p1<minL1)
                        minL1 = Lr11j_1[d+1]+p1;
                }
                if(d>0){
                    if(j>winsize){
                        if(minL0<0||Lr0j_1[d-1]+p1<minL0)
                            minL0 = Lr0j_1[d-1]+p1;
                        if(minL1<0||Lr11j_1[d-1]+p1<minL1)
                           minL1 = Lr11j_1[d-1]+p1;
                    }
                    if(minL2<0||Lr21j[d-1]+p1<minL2)
                        minL2 = Lr21j[d-1]+p1;
                }

                if(minL2<0||Lr21j[d]<minL2)
                    minL2 = Lr21j[d];

                if(j+1+winsize<size.width){
                    if(minL3<0||Lr31j_1[d]<minL3)
                        minL3 = Lr31j_1[d];
                    if(d>0)
                        if(minL3<0||Lr31j_1[d-1]+p1<minL3)
                            minL3 = Lr31j_1[d-1]+p1;
                    if(minL3<0||Lr31j_1[d+1]+p1<minL3)
                        minL3 = Lr31j_1[d+1]+p1;
                }
                if(j>winsize){
                    if(minLr00>=0)
                        if(minL0<0||minLr00+p2<minL0)
                            minL0 = minLr00+p2;
                }
                if(j>winsize)
                    if(minLr11[j-1]>=0)
                        if(minL1<0||minLr11[j-1]+p2<minL1)
                            minL1 = minLr11[j-1] + p2;
                if(minLr21[j]>=0)
                    if(minL2<0||minLr21[j]+p2<minL2)
                        minL2 = minLr21[j] + p2;
                if(j+1+winsize<size.width)
                    if(minL3<0||minLr31[j+1]+p2<minL3)
                        minL3 = minLr31[j+1]+p2;

                //double c = (*leftptrij - rightptrij[-d])*(*leftptrij - rightptrij[-d]);
                double sum = 0;
                double sumw = 0;
                int p = (i*size.width + j)*3;
                int p_ = p-3*d;
                int w1index = j*(2*winsize+1)*(2*winsize+1);
                int w2index = (j-d)*(2*winsize+1)*(2*winsize+1);
                for(int i1 = -winsize;i1<=winsize;i1++){
                    for(int j1 = -winsize;j1<=winsize;j1++){

                        int q = ((i+i1)*size.width + j+j1)*3;
                        int q_ = q-d*3;
                        double e1 = fabs(leftptr[q]-rightptr[q_])+
                                fabs(leftptr[q+1]-rightptr[q_+1])+
                                fabs(leftptr[q+2]-rightptr[q_+2]);

                        double w1 = w1buff[w1index++];
                        double w2 = w2buff[w2index++];
                        sum += w1*w2*e1;
                        sumw += w1*w2;
                    }
                }
                double c = sum/sumw;

                if(minL0>0)
                    minL0 +=c;
                else minL0 = c;
                if(minL1>0)
                    minL1 +=c;
                else minL1 = c;
                if(minL2>0)
                    minL2 +=c;
                else minL2 = c;
                if(minL3>0)
                    minL3 +=c;
                else minL3 = c;
                Lr0j[d] = minL0;
                Lr12j[d] = minL1;
                Lr22j[d] = minL2;
                Lr32j[d] = minL3;
                if(mminL00_<0||minL0<mminL00_)
                    mminL00_ = minL0;
                if(mminL1_<0||minL1<mminL1_)
                    mminL1_ = minL1;
                if(mminL2_<0||minL2<mminL2_)
                    mminL2_ = minL2;
                if(mminL3_<0||minL3<mminL3_)
                    mminL3_ = minL3;
                if(mminv<0||(minL0+minL1+minL2+minL3)<mminv){
                    mminv = minL0+minL1+minL2+minL3;
                    mmindex = d;
                }
                /*if(mminv<0||minL2<mminv){
                    mminv = minL2;
                    mmindex = d;
                }*/
            }
            disptr[i*size.width+j] = mmindex;
            minLr00 = mminL00_;
            minLr12[j] = mminL1_;
            minLr22[j] = mminL2_;
            minLr32[j] = mminL3_;
        }
        swap_d(Lr11,Lr12);
        swap_d(Lr21,Lr22);
        swap_d(Lr31,Lr32);
        swap_d(minLr11,minLr12);
        swap_d(minLr21,minLr22);
        swap_d(minLr31,minLr32);
        qDebug()<<"finished "<<i*100/size.height<<"%\r";

    }

    delete []Lr;
    delete []minLr;
    delete []w1buff;
    delete []w2buff;

}

void test_semi(){
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
//            stereoSemi4(leftmat,rightmat,dis,1,8*4*4,32*4*4,40);
//            stereoSemibm(leftmat,rightmat,dis,8*7*7,32*7*7,40,3);
            stereoSemi_AW(leftmat,rightmat,dis,8*11*11,32*11*11,130,7);
//            stereoSemi2(leftmat,rightmat,dis,1,8*5*5,32*3*3,128);
//            stereoSemi1(leftmat,rightmat,dis,1,8*5*5,32*3*3,128);
//            stereoSemi(leftmat,rightmat,dis,2,8*3*3,32*3*3,128);

            dis.convertTo(vdisp,CV_8U);
            normalize(vdisp,vdisp,0,255,CV_MINMAX);
            imshow("semi ",vdisp);
        }
    }
}

//This colors the segmentations
static void floodFillPostprocess( Mat& img, const Scalar& colorDiff=Scalar::all(1) )
{
    CV_Assert( !img.empty() );
    RNG rng = theRNG();
    Mat mask( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0) );
    for( int y = 0; y < img.rows; y++ )
    {
        for( int x = 0; x < img.cols; x++ )
        {
            if( mask.at<uchar>(y+1, x+1) == 0 )
            {
                Scalar newVal( rng(256), rng(256), rng(256) );
                floodFill( img, mask, Point(x,y), newVal, 0, colorDiff, colorDiff );
            }
        }
    }
}

string winName = "meanshift";
int spatialRad, colorRad, maxPyrLevel;
Mat img, res;

static void meanShiftSegmentation( int, void* )
{
    cout << "spatialRad=" << spatialRad << "; "
         << "colorRad=" << colorRad << "; "
         << "maxPyrLevel=" << maxPyrLevel << endl;
    pyrMeanShiftFiltering( img, res, spatialRad, colorRad, maxPyrLevel );
    if(res.type()==CV_32S){
        cout<<"res.type CV_32S"<<endl;
    }
    else if(res.type()==CV_16U){
        cout<<"res.type CV_16U"<<endl;
    }
    else if(res.type()==CV_16S){
        cout<<"res.type CV_16S"<<endl;
    }
    else if(res.type()==CV_8U){
        cout<<"res.type CV_8U"<<endl;
    }
    else if(res.type()==CV_8UC3){
        cout<<"res.type CV_8UC3"<<endl;
    }
    floodFillPostprocess( res, Scalar::all(2) );
    imshow( winName, res );
}

void testMeanShift(){
    QString leftfilename = QFileDialog::getOpenFileName(
       0,
       "Binocular Calibration - Open Left Image",
       NULL,
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!leftfilename.isNull()) { //用户选择了左图文件
        img = imread( leftfilename.toStdString().c_str() );
        if( img.empty() )
            return ;

        spatialRad = 10;
        colorRad = 10;
        maxPyrLevel = 1;

        namedWindow( winName, WINDOW_AUTOSIZE );

        createTrackbar( "spatialRad", winName, &spatialRad, 80, meanShiftSegmentation );
        createTrackbar( "colorRad", winName, &colorRad, 60, meanShiftSegmentation );
        createTrackbar( "maxPyrLevel", winName, &maxPyrLevel, 5, meanShiftSegmentation );

        meanShiftSegmentation(0, 0);
        waitKey();
    }
}

///*
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    MainWindow w;
    w.show();
    //cvNamedWindow("test");
//    testsegment();
//    testBM();
//    testMine();
//    testRealTime();
//    testLocal();
//    testDynamic();
//    test_dp();
//    test_semi();
//    testMeanShift();

    return app.exec();
}
//*/
