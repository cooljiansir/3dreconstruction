#include <QApplication>
#include "cv.h"
#include "highgui.h"
#include <QFileDialog>
#include <algorithm>
#include <time.h>

#include <QDebug>


using namespace cv;
using namespace std;

void stereoBmProto(Mat &leftmat,Mat &rightmat,Mat &dis,int winsize,int maxdis){
    if(leftmat.size()!=rightmat.size())
        return ;
    Size size = leftmat.size();

    Mat leftgr = Mat::zeros(size.height+2*winsize,size.width+2*winsize,CV_8U);
    Mat rightgr = Mat::zeros(leftgr.size(),CV_8U);
    Size size2 = leftgr.size();

    Mat leftgray  = leftgr(Rect(winsize,winsize,size.width,size.height));
    Mat rightgray = rightgr(Rect(winsize,winsize,size.width,size.height));
    cvtColor(leftmat,leftgray,CV_BGR2GRAY);
    cvtColor(rightmat,rightgray,CV_BGR2GRAY);

    unsigned char *leftptr = leftgray.data;
    unsigned char *rightptr = rightgray.data;

    dis.create(size,CV_32F);
    float *disptr = (float *)dis.data;

    for(int i = 0;i<size.height;i++){
        for(int j = 0;j<size.width;j++){
            int mmin = 1<<29;
            int mindex;
            unsigned char *leftptrij = leftptr + i*size2.width+j;
            unsigned char *rightptrij = rightptr +i*size2.width+j;
            for(int d = 0;d<maxdis&&d<=j;d++){
                int sum  = 0;
                for(int i1 = -winsize;i1<=winsize;i1++){
                    for(int j1 = -winsize;j1<=winsize;j1++){
                        sum += abs(leftptrij[i1*size2.width+j1]-rightptrij[i1*size2.width+j1-d]);
                    }
                }
                if(sum<mmin)
                    mmin = sum,mindex = d;
            }
            disptr[i*size.width+j] = mindex;
        }
    }

}


/*
 *从上往下，BlockMatching
 *
 *Sum(x,y,d) = Sum(x,y-1,d) + s(x,y+winsize,d) - s(x,y-winsize-1,d)
 *s(x,y,d)   = s(x-1,y,d) + I(x+winsize,y,d)-I(x-winsize-1,y,d)
 *I(x,y,d)   = fabs(c(x,y)-c(x-d,y))
 *
 *
 */

void stereo_BMBox(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize){
    if(left.size()!=right.size())
        return;
    Mat leftgr,rightgr;
    Size size = left.size();

    leftgr = Mat::zeros(size.height+2*winsize+1,size.width+2*winsize+1,CV_8U);
    rightgr = Mat::zeros(leftgr.size(),CV_8U);
    Size size2 = leftgr.size();

    Mat leftgray = leftgr(Rect(winsize+1,winsize+1,size.width,size.height));
    Mat rightgray = rightgr(Rect(winsize+1,winsize+1,size.width,size.height));

    cvtColor(left,leftgray,CV_BGR2GRAY);
    cvtColor(right,rightgray,CV_BGR2GRAY);


    unsigned char *leftptr = leftgray.data;
    unsigned char *rightptr = rightgray.data;

    dis.create(size,CV_32F);
    float *disptr = (float *)dis.data;

    int *sad = new int[maxdis];
    int dfafs;
    int *s0_   = new int[(size.height+2*winsize+1)*maxdis];
    int *s1_   = new int[(size.height+2*winsize+1)*maxdis];

    int *s0 = s0_ + (winsize+1)*maxdis;
    int *s1 = s1_ + (winsize+1)*maxdis;


    //initial s0
    for(int i = - winsize-1;i<size.height+winsize;i++){
        for(int d = 0;d<maxdis;d++)
            s0[i*maxdis+d] = s1[i*maxdis+d] = 0;
    }


    for(int j = 0;j<size.width;j++){
        for(int d = 0;d<maxdis&&d<=j;d++){
            sad[d] = 0;
            for(int i1=0;i1<winsize;i1++){
                for(int j1 = -winsize;j1<=winsize;j1++){
                    sad[d] += abs(leftptr[i1*size2.width+j+j1]-rightptr[i1*size2.width+j+j1-d]);
                }
            }
        }

        //cal new s0
        for(int i = - winsize-1;i<size.height+winsize;i++){
            //d<j
            int tem = i*size2.width+j+winsize;
            int tem2 = i*size2.width+j-winsize-1;
            int *s0temp = s0 + i*maxdis;
            int *s1temp = s1 + i*maxdis;
            for(int d = 0;d<maxdis&&d<j;d++){
                s1temp[d] = s0temp[d]
                        + abs(leftptr[tem]-rightptr[tem-d])
                        - abs(leftptr[tem2]-rightptr[tem2-d]);
            }
            //d=j
            if(j<=maxdis){
                s1[i*maxdis+j] = 0;
                for(int j1=-winsize;j1<=winsize;j1++){
                    s1[i*maxdis+j] += abs(leftptr[i*size2.width+j+j1]-rightptr[i*size2.width+j1]);
                }
            }
        }
        int *temp = s0;
        s0 = s1;
        s1 = temp;
        for(int i = 0;i<size.height;i++){
            int mind;
            int minsad=1<<29;
            int tem = (i+winsize)*maxdis;
            int tem2 = (i-1-winsize)*maxdis;
            for(int d = 0;d<maxdis&&d<=j;d++){
                sad[d] = sad[d] + s0[tem+d] - s0[tem2+d];
                if(sad[d]<minsad){
                    minsad = sad[d];
                    mind = d;
                }
            }
            disptr[i*size.width+j] = mind;
        }

    }
    delete []sad;
    qDebug()<<"释放完sad"<<endl;
    delete []s0_;
    qDebug()<<"释放完s0"<<endl;
    delete []s1_;
    qDebug()<<"释放完s1"<<endl;

}


void testAll(){
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

            clock_t t = clock();
//            stereoBmProto(leftmat,rightmat,dis,8,20);
            stereo_BMBox(leftmat,rightmat,dis,20,5);
            qDebug()<<"used time "<<clock()-t<<"ms"<<endl;

            dis.convertTo(vdisp,CV_8U);
            normalize(vdisp,vdisp,0,255,CV_MINMAX);
            imshow("15*15 ",vdisp);
        }
    }
}





int main(int argc, char *argv[]){
    QApplication a(argc, argv);
    testAll();
    return a.exec();
}
