#if 0
#define

#include <QApplication>
#include "cv.h"
#include "highgui.h"
#include <QFileDialog>
#include <algorithm>
#include <time.h>
#include "stereoform.h"

#include <QDebug>


using namespace cv;
using namespace std;

extern void LRC(int *cost,int maxdis,Size size,int maxdiff,int *dis);
extern void uniquenessC(int *cost,int *dis,int maxdis,Size size);
extern void connetFilter(int *dis,Size size,int diff,int minarea);


//灰度图转梯度图
void Xsobel(Mat &img,Mat &res,int thre){
    Size size = img.size();

    Mat temp;
    temp.create(size,CV_8U);
    unsigned char *resptr = temp.data;

    //initial border
    for(int i = 0;i<size.height;i++){
        resptr[i*size.width] = resptr[i*size.width+size.width-1] = 0;
    }
    for(int i = 0;i<size.width;i++){
        resptr[i] = resptr[(size.height-1)*size.width+i] = 0;
    }

    for(int i = 1;i+1<size.height;i++){
        unsigned char *imgptri = img.ptr<unsigned char>(i);
        unsigned char *imgptri_n = img.ptr<unsigned char>(i+1);
        unsigned char *imgptri_p = img.ptr<unsigned char>(i-1);
        unsigned char *resptri = resptr+i*size.width;
        for(int j = 1;j+1<size.width;j++){
            int d = (imgptri[j+1] - imgptri[j-1])*2 + imgptri_n[j+1] - imgptri_n[j-1]+imgptri_p[j+1] - imgptri_p[j-1];
            if(d<-thre)d = -thre;
            if(d>thre)d = thre;
            resptri[j] = d + 128;
        }
    }
    temp.copyTo(res);
}

/*
 *从上往下，BlockMatching
 *加入BT算法
 *
 *如果不使用Sobel预处理，令prefilter<0
 *如果不使用BT算法，令BT=false
 *
 *
 *Sum(x,y,d) = Sum(x,y-1,d) + s(x,y+winsize,d) - s(x,y-winsize-1,d)
 *s(x,y,d)   = s(x-1,y,d) + I(x+winsize,y,d)-I(x-winsize-1,y,d)
 *I(x,y,d)   = fabs(c(x,y)-c(x-d,y))
 *
 *
 */


void stereo_BMBox_BT(Mat &left,Mat &right,int *costout,int maxdis,int winsize,int prefilter,bool BT){
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

    if(prefilter>0){
        Xsobel(leftgray,leftgray,prefilter);
        Xsobel(rightgray,rightgray,prefilter);
    }


    unsigned char *leftptr = leftgray.data;
    unsigned char *rightptr = rightgray.data;


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
                    int temp = i1*size2.width+j+j1;
//                    sad[d] += abs(leftptr[i1*size2.width+j+j1]-rightptr[i1*size2.width+j+j1-d]);
                    if(!BT)
                        sad[d] += abs(leftptr[temp]-rightptr[temp-d]);
                    else{
                        int a1=leftptr[temp-1],a2=leftptr[temp],a3=leftptr[temp+1],
                                b1=rightptr[temp-d-1],b2=rightptr[temp-d],b3=rightptr[temp-d+1];
                        a1 = (a1+a2)/2;
                        a3 = (a2+a3)/2;
                        b1 = (b1+b2)/2;
                        b3 = (b2+b3)/2;
                        sad[d] +=
                        min(
                            min(
                                min(
                                    min(abs(a1-b1),abs(a1-b2)),
                                    min(abs(a1-b3),abs(a2-b1))),
                                min(
                                    min(abs(a2-b2),abs(a2-b3)),
                                    min(abs(a3-b1),abs(a3-b2)))),
                            abs(a3-b3));
                    }

//                    int A =  max(max(0,a2-max(max(b1,b2),b3)),min(min(b1,b2),b3)-a2);
//                    int B =  max(max(0,b2-max(max(a1,a2),a3)),min(min(a1,a2),a3)-b2);

//                    sad[d] += min(A,B);
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
                if(!BT){
                    s1temp[d] = s0temp[d]
                        + abs(leftptr[tem]-rightptr[tem-d])
                        - abs(leftptr[tem2]-rightptr[tem2-d]);
                }
                else{
                    int a1=leftptr[tem-1],a2=leftptr[tem],a3=leftptr[tem+1],
                            b1=rightptr[tem-d-1],b2=rightptr[tem-d],b3=rightptr[tem-d+1];
                    a1 = (a1+a2)/2;
                    a3 = (a2+a3)/2;
                    b1 = (b1+b2)/2;
                    b3 = (b2+b3)/2;
                    s1temp[d] = s0temp[d] +
                            min(
                                min(
                                    min(
                                        min(abs(a1-b1),abs(a1-b2)),
                                        min(abs(a1-b3),abs(a2-b1))),
                                    min(
                                        min(abs(a2-b2),abs(a2-b3)),
                                        min(abs(a3-b1),abs(a3-b2)))),
                                abs(a3-b3));

                    a1=leftptr[tem2-1],a2=leftptr[tem2],a3=leftptr[tem2+1],
                            b1=rightptr[tem2-d-1],b2=rightptr[tem2-d],b3=rightptr[tem2-d+1];
                    a1 = (a1+a2)/2;
                    a3 = (a2+a3)/2;
                    b1 = (b1+b2)/2;
                    b3 = (b2+b3)/2;
                    s1temp[d] -=
                            min(
                                min(
                                    min(
                                        min(abs(a1-b1),abs(a1-b2)),
                                        min(abs(a1-b3),abs(a2-b1))),
                                    min(
                                        min(abs(a2-b2),abs(a2-b3)),
                                        min(abs(a3-b1),abs(a3-b2)))),
                                abs(a3-b3));
                }
            }
            //d=j
            if(j<maxdis){
                s1[i*maxdis+j] = 0;
                for(int j1=-winsize;j1<=winsize;j1++){
                    if(!BT)
                        s1[i*maxdis+j] += abs(leftptr[i*size2.width+j+j1]-rightptr[i*size2.width+j1]);
                    else{
                        int temp = i*size2.width+j1;
                        int a1 = leftptr[temp+j-1],a2=leftptr[temp+j],a3=leftptr[temp+j+1];
                        int b1 = rightptr[temp-1],b2 = rightptr[temp],b3 = rightptr[temp+1];
                        a1 = (a1+a2)/2;
                        a3 = (a2+a3)/2;
                        b1 = (b1+b2)/2;
                        b3 = (b2+b3)/2;
                        s1[i*maxdis+j] +=
                                min(
                                    min(
                                        min(
                                            min(abs(a1-b1),abs(a1-b2)),
                                            min(abs(a1-b3),abs(a2-b1))),
                                        min(
                                            min(abs(a2-b2),abs(a2-b3)),
                                            min(abs(a3-b1),abs(a3-b2)))),
                                    abs(a3-b3));
                    }

                }
            }
        }
        int *temp = s0;
        s0 = s1;
        s1 = temp;
        int *costj = costout+j*maxdis;
        for(int i = 0;i<size.height;i++){
            int mind;
            int minsad=1<<29;
            int tem = (i+winsize)*maxdis;
            int tem2 = (i-1-winsize)*maxdis;
            for(int d = 0;d<maxdis&&d<=j;d++){
                sad[d] = sad[d] + s0[tem+d] - s0[tem2+d];
                costj[d] = sad[d];
            }
            costj +=size.width*maxdis;
        }

    }
    delete []sad;
    delete []s0_;
    delete []s1_;
}


void stereo_MSGM_MY(Mat &left,Mat &right,Mat &dis,int maxdis,int P1,int P2,int iter,int winsize,int prefilter,bool BT){
    if(left.size()!=right.size())
        return;
    Size size = left.size();

    dis.create(size,CV_32F);
    float *disptr = (float *)dis.data;

    Mat leftgray,rightgray;
    cvtColor(left,leftgray,CV_BGR2GRAY);
    cvtColor(right,rightgray,CV_BGR2GRAY);

    unsigned char *leftptr = leftgray.data;
    unsigned char *rightptr = rightgray.data;

    //-1,size.width+1
    int LrWidth = size.width+2;
    int LrMax  = maxdis+2;

    int *cost = new int[size.width*size.height*maxdis];

    stereo_BMBox_BT(left,right,cost,maxdis,winsize,prefilter,BT);

    int *disint = new int[size.width*size.height];
    int *LrSum   = new int[size.width*size.height*maxdis];
    int *LrBuff_ = new int[LrWidth*LrMax*4*2];//4个方向，双缓冲,Lr(j,d+1)
    int *minLr_ = new int[LrWidth*4*2];//4个方向，双缓冲
    if(!LrSum||!LrBuff_||!minLr_){
        return ;
    }

    int *Lr0 = LrBuff_+LrMax;
    int *Lr01 = Lr0+4*LrWidth*LrMax;
    int *minLr0 = minLr_+1;
    int *minLr01 = minLr0+4*LrWidth;


    //initial Lr
    for(int i = -1;i<=size.width;i++){
        int *Lr1 = Lr0+LrWidth*LrMax;
        int *Lr2 = Lr0+2*LrWidth*LrMax;
        int *Lr3 = Lr0+3*LrWidth*LrMax;
        int *Lr11 = Lr01+LrWidth*LrMax;
        int *Lr21 = Lr01+2*LrWidth*LrMax;
        int *Lr31 = Lr01+3*LrWidth*LrMax;
        for(int d = -1;d<=maxdis;d++){
            int v = 0;
            if(d==-1||d==maxdis)
                v = 1<<29;
            Lr0[i*LrMax+d+1] = v;
            Lr1[i*LrMax+d+1] = v;
            Lr2[i*LrMax+d+1] = v;
            Lr3[i*LrMax+d+1] = v;
            Lr01[i*LrMax+d+1] = v;
            Lr11[i*LrMax+d+1] = v;
            Lr21[i*LrMax+d+1] = v;
            Lr31[i*LrMax+d+1] = v;
        }
    }

    //initial minLr
    for(int i = -1;i<=size.width;i++){
        int *minLr1 = minLr0+LrWidth;
        int *minLr2 = minLr0+2*LrWidth;
        int *minLr3 = minLr0+3*LrWidth;
        int *minLr11 = minLr01+LrWidth;
        int *minLr21 = minLr01+2*LrWidth;
        int *minLr31 = minLr01+3*LrWidth;
        minLr0[i] = 0;
        minLr1[i] = 0;
        minLr2[i] = 0;
        minLr3[i] = 0;
        minLr01[i] = 0;
        minLr11[i] = 0;
        minLr21[i] = 0;
        minLr31[i] = 0;
    }

    for(int pass = 0;pass<iter;pass++){
        //四个方向
        //
        //1 2  3
        // ↖↑↗
        //0←
        //
        for(int i = 0;i<size.height;i++){
        int *Lr1 = Lr0+LrWidth*LrMax;
        int *Lr2 = Lr0+2*LrWidth*LrMax;
        int *Lr3 = Lr0+3*LrWidth*LrMax;
        int *Lr11 = Lr01+LrWidth*LrMax;
        int *Lr21 = Lr01+2*LrWidth*LrMax;
        int *Lr31 = Lr01+3*LrWidth*LrMax;
        int *minLr1 = minLr0+LrWidth;
        int *minLr2 = minLr0+2*LrWidth;
        int *minLr3 = minLr0+3*LrWidth;
        int *minLr11 = minLr01+LrWidth;
        int *minLr21 = minLr01+2*LrWidth;
        int *minLr31 = minLr01+3*LrWidth;

        int *costi = cost+i*size.width*maxdis;
        int *LrSumi = LrSum + i*size.width*maxdis;
        for(int j = 0;j<size.width;j++){
            int *costij = costi+j*maxdis;
            int *LrSumij = LrSumi+j*maxdis;

            int *Lr0j_1 = Lr0+(j-1)*LrMax;
            int *minLr0j = minLr0+j;
            *minLr0j = 1<<29;
            int *Lr0j = Lr0+j*LrMax;

            int *Lr11j = Lr11+j*LrMax;
            int *Lr1j_1 = Lr1 + (j-1)*LrMax;
            int *minLr11j = minLr11+j;
            *minLr11j =  1<<29;

            int *Lr21j = Lr21+j*LrMax;
            int *Lr2j= Lr2+j*LrMax;
            int *minLr21j = minLr21+j;
            *minLr21j = 1<<29;

            int *Lr31j = Lr31+j*LrMax;
            int *Lr3j_1 = Lr3 + (j+1)*LrMax;
            int *minLr31j = minLr31 + j;
            *minLr31j = 1<<29;

            int minc = 1<<29;
            int mind=0;
            for(int d = 0;d<maxdis&&d<=j;d++){
                Lr0j[d+1] = costij[d]
                        + min(min(Lr0j_1[d+1],minLr0[j-1]+P2),min(Lr0j_1[d],Lr0j_1[d+2])+P1)- minLr0[j-1];
                *minLr0j = min(*minLr0j,Lr0j[d+1]);

                Lr11j[d+1] = costij[d]
                        + min(min(Lr1j_1[d+1],minLr1[j-1]+P2),min(Lr1j_1[d],Lr1j_1[d+2])+P1) - minLr1[j-1];
                *minLr11j = min(*minLr11j,Lr11j[d+1]);

                Lr21j[d+1] = costij[d]
                        +min(min(Lr2j[d+1],minLr2[j]+P2),min(Lr2j[d],Lr2j[d+2])+P1) - minLr2[j];
                *minLr21j = min(*minLr21j,Lr21j[d+1]);

                Lr31j[d+1] = costij[d]
                        +min(min(Lr3j_1[d+1],minLr3[j+1]+P2),min(Lr3j_1[d],Lr3j_1[d+2])+P1) - minLr3[j+1];
                *minLr31j = min(*minLr31j,Lr31j[d+1]);

                LrSumij[d] = 0
                        +Lr0j[d+1]
                        +Lr11j[d+1]
                        +Lr21j[d+1]
                        +Lr31j[d+1]
                        ;

            }
            disptr[i*size.width+j] = mind;
        }
        int *temp = Lr0;
        Lr0  = Lr01;
        Lr01 = temp;

        temp = minLr0;
        minLr0 = minLr01;
        minLr01  = temp;
    }
        //四个方向
        //
        //1 2  3
        // ↖↑↗
        //0←
        //
        for(int i = size.height-1;i>=0;i--){
        int *Lr1 = Lr0+LrWidth*LrMax;
        int *Lr2 = Lr0+2*LrWidth*LrMax;
        int *Lr3 = Lr0+3*LrWidth*LrMax;
        int *Lr11 = Lr01+LrWidth*LrMax;
        int *Lr21 = Lr01+2*LrWidth*LrMax;
        int *Lr31 = Lr01+3*LrWidth*LrMax;
        int *minLr1 = minLr0+LrWidth;
        int *minLr2 = minLr0+2*LrWidth;
        int *minLr3 = minLr0+3*LrWidth;
        int *minLr11 = minLr01+LrWidth;
        int *minLr21 = minLr01+2*LrWidth;
        int *minLr31 = minLr01+3*LrWidth;

        int *costi = cost+i*size.width*maxdis;
        int *LrSumi = LrSum + i*size.width*maxdis;
        for(int j = size.width-1;j>=0;j--){
            int *costij = costi+j*maxdis;
            int *LrSumij = LrSumi+j*maxdis;

            int *Lr0j_1 = Lr0+(j+1)*LrMax;
            int *minLr0j = minLr0+j;
            *minLr0j = 1<<29;
            int *Lr0j = Lr0+j*LrMax;

            int *Lr11j = Lr11+j*LrMax;
            int *Lr1j_1 = Lr1 + (j-1)*LrMax;
            int *minLr11j = minLr11+j;
            *minLr11j =  1<<29;

            int *Lr21j = Lr21+j*LrMax;
            int *Lr2j= Lr2+j*LrMax;
            int *minLr21j = minLr21+j;
            *minLr21j = 1<<29;

            int *Lr31j = Lr31+j*LrMax;
            int *Lr3j_1 = Lr3 + (j+1)*LrMax;
            int *minLr31j = minLr31 + j;
            *minLr31j = 1<<29;

            int minc = 1<<29;
            int mind=0;
            for(int d = 0;d<maxdis&&d<=j;d++){
                Lr0j[d+1] = costij[d]
                        + min(min(Lr0j_1[d+1],minLr0[j+1]+P2),min(Lr0j_1[d],Lr0j_1[d+2])+P1)- minLr0[j+1];
                *minLr0j = min(*minLr0j,Lr0j[d+1]);

                Lr11j[d+1] = costij[d]
                        + min(min(Lr1j_1[d+1],minLr1[j-1]+P2),min(Lr1j_1[d],Lr1j_1[d+2])+P1) - minLr1[j-1];
                *minLr11j = min(*minLr11j,Lr11j[d+1]);

                Lr21j[d+1] = costij[d]
                        +min(min(Lr2j[d+1],minLr2[j]+P2),min(Lr2j[d],Lr2j[d+2])+P1) - minLr2[j];
                *minLr21j = min(*minLr21j,Lr21j[d+1]);

                Lr31j[d+1] = costij[d]
                        +min(min(Lr3j_1[d+1],minLr3[j+1]+P2),min(Lr3j_1[d],Lr3j_1[d+2])+P1)- minLr3[j+1];
                *minLr31j = min(*minLr31j,Lr31j[d+1]);

                LrSumij[d] += 0
                        +Lr0j[d+1]
                        +Lr11j[d+1]
                        +Lr21j[d+1]
                        +Lr31j[d+1]
                        ;
                LrSumij[d] /= 8;
                if(LrSumij[d]<minc)
                    minc = LrSumij[d],mind = d;
            }
//            disptr[i*size.width+j] = mind;
            disint[i*size.width+j] = mind;
        }
        int *temp = Lr0;
        Lr0  = Lr01;
        Lr01 = temp;

        temp = minLr0;
        minLr0 = minLr01;
        minLr01  = temp;
    }
        int *temp = cost;
        cost = LrSum;
        LrSum = temp;
    }

//    LRC(cost,maxdis,size,2,disint);
    uniquenessC(cost,disint,maxdis,size);
    connetFilter(disint,size,2,200);

    for(int i = 0;i<size.width*size.height;i++)
        disptr[i] = disint[i];

    delete []cost;
    delete []LrSum;
    delete []LrBuff_;
    delete []minLr_;
    delete []disint;
}


void testMyWorld(){
//    QString leftfilename = "D:/works/qtwork5_5/build-3dreconstruction-Desktop_Qt_5_1_0_MinGW_32bit-Debug/images/opencv/tsukubaL.bmp";
//    QString rightfilename = "D:/works/qtwork5_5/build-3dreconstruction-Desktop_Qt_5_1_0_MinGW_32bit-Debug/images/opencv/tsukubaR.bmp";

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
            StereoForm *form = new StereoForm(leftmat,rightmat);
            form->showMaximized();

//            Mat dis,vdisp;


//            clock_t t = clock();
//            stereo_MSGM_MY(leftmat,rightmat,dis,16*7,8,32,5,0,63,true);

//            qDebug()<<"used time "<<clock()-t<<"ms"<<endl;


//            dis.convertTo(vdisp,CV_8U);
//            normalize(vdisp,vdisp,0,255,CV_MINMAX);

//            imshow("SGM P1=8 P2=32 ",vdisp);
        }
    }
}



///*
int main(int argc, char *argv[]){
    QApplication a(argc, argv);
    testMyWorld();
    return a.exec();
}
//*/
#endif
