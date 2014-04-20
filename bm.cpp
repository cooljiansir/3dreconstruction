#include <QApplication>
#include "cv.h"
#include "highgui.h"
#include <QFileDialog>
#include <algorithm>
#include <time.h>

#include <QDebug>


using namespace cv;
using namespace std;


void stereo_BM(Mat &left,Mat &right,Mat &dis,int winsize,int maxdis){
    if(left.size()!=right.size())
        return;
    Size size = left.size();
    dis.create(size,CV_32F);

    Mat disR;
    disR.create(left.size(),CV_32F);
    float *disRptr = (float*)disR.data;

    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;
    float *disptr = (float*)dis.data;

    double *aline = new double[size.width*maxdis];
    double *tempcost = new double[maxdis];//left disparity
    double *tempcost2 = new double[maxdis];//right disparity

    clock_t t1 = clock();
    //initial the border
    //up and bottom
    for(int j = 0;j<size.width;j++){
        for(int i = 0;i<winsize;i++)
            disptr[i*size.width+j] = -1;
        for(int i = size.height-winsize;i<size.height;i++)
            disptr[i*size.width+j] = -1;
    }
    //left and right
    for(int i = 0;i<size.height;i++){
        for(int j = 0;j<winsize;j++)
            disptr[i*size.width+j] = -1;
        for(int j = size.width-winsize;j<size.width;j++)
            disptr[i*size.width+j] = -1;
    }
    qDebug()<<"use time 1 "<<clock() - t1<<"ms"<<endl;
    t1 = clock();
    //initial aline
    for(int j = 0;j<size.width;j++){
        unsigned char *p1 = leftptr + j*3;
        for(int d = 0;d<maxdis&&d<=j;d++){
            double te = 0;
            unsigned char *p2 = rightptr + (j-d)*3;
            for(int i = 0;i<winsize*2+1;i++){
                unsigned char *p1_ = p1+3*i*size.width;
                unsigned char *p2_ = p2+3*i*size.width;
                te += fabs(p1_[0] - p2_[0]);
                te += fabs(p1_[1] - p2_[1]);
                te += fabs(p1_[2] - p2_[2]);
            }
            aline[j*maxdis+d] = te;
        }
    }
    qDebug()<<"use time 2 "<<clock() - t1<<"ms"<<endl;
    t1 = clock();
    for(int i = winsize;i+winsize<size.height;i++){
        //calculate right disparity
        for(int j = winsize;j+winsize<size.width;j++){
//            double *sub = aline+(j+d-winsize-1)*maxdis;
//            double *sub_ = aline+(j+d+winsize)*maxdis;
            int mmind = 0;
            for(int d = 0;d<maxdis&&d+j+winsize<=size.width;d++){
                if(j>winsize){//pre has valid value
                    tempcost2[d] = tempcost2[d] - aline[(j+d-winsize-1)*maxdis+d] + aline[(j+d+winsize)*maxdis+d];
//                    tempcost2[d] = tempcost2[d] - sub[d] + sub_[d];
                }else{
                    tempcost2[d] = 0;
                    for(int k = j+d-winsize;k<=j+d+winsize;k++)
                        tempcost2[d] += aline[k*maxdis+d];
                }
                if(tempcost2[d]<tempcost2[mmind])
                    mmind = d;
            }
            disRptr[i*size.width+j] = mmind;
        }
        //calculate left disparity
        for(int j = winsize;j+winsize<size.width;j++){
            int mmind = 0;
            double *sub = aline+(j-winsize-1)*maxdis;
            double *sub_ = aline+(j+winsize)*maxdis;
            for(int d = 0;d<maxdis&&d+winsize<=j;d++){
                if(j-d-winsize>0){//pre has valide value
                    //tempcost[d] = tempcost[d] - aline[(j-winsize-1)*maxdis+d] + aline[(j+winsize)*maxdis+d];
                    tempcost[d] = tempcost[d] - sub[d] + sub_[d];
                }else{//
                    tempcost[d] = 0;
                    for(int k = j - winsize;k<=j+winsize;k++)
                        tempcost[d] += aline[k*maxdis+d];
                }
                if(tempcost[d]<tempcost[mmind])
                    mmind = d;
            }

            double te = mmind;
            double ter = disRptr[i*size.width+j-mmind];
            if(fabs(te-ter)>1){//consistence
                disptr[i*size.width+j] = -1;
            }
            else{
                //interpolation
                //见 “三维重建中立体匹配算法的研究” P21
                if(mmind>0&&mmind<maxdis-1&&mmind+winsize<j){
                    double tem = 2*tempcost[mmind - 1]  + 2*tempcost[mmind+1] - 4*tempcost[mmind];
                    if(tem>0.001)te = te +(tempcost[mmind - 1]  - tempcost[mmind+1])/tem;
                }
                disptr[i*size.width+j] = te;
            }
        }

        if(i+winsize+1==size.height)
            break;
        int tempd = (2*winsize+1)*size.width*3;
        for(int j = 0;j<size.width;j++){
            for(int d = 0;d<maxdis&&d<=j;d++){
                unsigned char *p1 = leftptr + ((i-winsize) * size.width + j)*3;
                unsigned char *p2 = rightptr + ((i-winsize) * size.width + j- d)*3;
                double te = - fabs(p1[0]-p2[0]) - fabs(p1[1]-p2[1]) - fabs(p1[2]-p2[2]);
                p1 += tempd;
                p2 += tempd;
                te += fabs(p1[0]-p2[0]) + fabs(p1[1]-p2[1]) + fabs(p1[2]-p2[2]);
                aline[j*maxdis+d] += te;
            }
        }
    }
    qDebug()<<"use time 3 "<<clock() - t1<<"ms"<<endl;
    t1 = clock();
    delete []aline;
    delete []tempcost;
    delete []tempcost2;
}
void stereo_BM_segment(Mat &left,Mat &right,Mat &dis,int winsize,int maxdis){
    if(left.size()!=right.size())
        return;

    double lamda = 0.1;
    //segmentation
    int spatialRad = 10,colorRad = 10,maxPyrLevel = 1;
    Mat segmentmat;
    pyrMeanShiftFiltering(left,segmentmat, spatialRad, colorRad, maxPyrLevel );

    Mat leftgray,rightgray;
    leftgray.create(left.size(),CV_8UC1);
    rightgray.create(right.size(),CV_8UC1);
    cvtColor(left,leftgray,CV_BGR2GRAY);
    cvtColor(right,rightgray,CV_BGR2GRAY);

    Size size = left.size();
    dis.create(size,CV_32F);

    unsigned char *leftptr = leftgray.data;
    unsigned char *rightptr = rightgray.data;
    unsigned char *segmentmatptr = segmentmat.data;

    double *cost = new double[size.width];
    int *costindex = new int[size.width];
    double *cost2 = new double[size.width];
    int *costindex2 = new int[size.width];

    float *disptr = (float*)dis.data;

    for(int i = 0;i<size.height;i++){
        for(int j = 0;j<size.width;j++){
            disptr[i*size.width+j] = -1;
        }
    }
    for(int i = winsize;i+winsize<size.height;i++){
        for(int j = winsize;j+winsize<size.width;j++){
            unsigned char *se = segmentmatptr+(i*size.width+j)*3;
            costindex[j] = -1;
            for(int d = 0;d<maxdis&&d+winsize<=j;d++){
                double sadsum = 0;
                for(int i1 = -winsize;i1<=winsize;i1++){
                    for(int j1 = -winsize;j1<=winsize;j1++){
                        unsigned char * se1 = se + (i1*size.width+j1)*3;
                        double r=1;
                        if(se[0]==se1[0]&&se[1]==se1[1]&&se[2]==se1[2])
                            r = 1;
                        else r = lamda;
                        sadsum += r*abs(leftptr[(i+i1)*size.width+j+j1]-
                                rightptr[(i+i1)*size.width+j-d+j1]);
                    }
                }
                if(costindex[j]==-1||sadsum<cost[j])
                    costindex[j] = d,cost[j] = sadsum;
            }
        }
        for(int j = winsize;j+winsize<size.width;j++){
            costindex2[j] = -1;
        }
        for(int j = winsize;j+winsize<size.width;j++){
            int d = costindex[j];
            if(d != -1){
                if(costindex2[j-d]==-1||cost2[j-d]<cost[j])
                    cost2[j-d] = cost[j],costindex2[j-d] = d;
            }

        }
        for(int j = winsize;j+winsize<size.width;j++){
            int d1 = costindex[j];
            if(d1 !=-1){
                int d2 = costindex2[j-d1];
                if(abs(d1-d2)>1)
                    d1 = -1;
            }
            disptr[i*size.width+j] = d1;
        }
    }
    delete []cost;
    delete []cost2;
    delete []costindex;
    delete []costindex2;
}

void stereo_BM_AW(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize){
    if(left.size()!=right.size())
        return;
//    double yc=7,yg=36;
    double yc=10,yg=36;

    Size size = left.size();
    dis.create(size,CV_32F);

    float *disptr = (float*)dis.data;

    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;

    for(int i = 0;i<size.height;i++)
        for(int j = 0;j<size.width;j++)
            disptr[i*size.width+j] = -1;

    //double *mincost = new double[size.width];
    //double *
    double *w1buff = new double[(2*winsize+1)*(2*winsize+1)*size.width];
    double *w2buff = new double[(2*winsize+1)*(2*winsize+1)*size.width];


    for(int i = winsize;i+winsize<size.height;i++){
        for(int j = winsize;j+winsize<size.width;j++){
            int p = (i*size.width + j)*3;
            int windex = j*(2*winsize+1)*(2*winsize+1);
            for(int i1 = -winsize;i1<=winsize;i1++){
                for(int j1 = -winsize;j1<=winsize;j1++){
                    int q = ((i+i1)*size.width + j+j1)*3;
                    double w1 =
                            exp(-sqrt((leftptr[p] - leftptr[q])*(leftptr[p] - leftptr[q])+
                                      (leftptr[p+1] - leftptr[q+1])*(leftptr[p+1] - leftptr[q+1])+
                                      (leftptr[p+2] - leftptr[q+2])*(leftptr[p+1] - leftptr[q+2]))/yc
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
            double mincost;
            int mmindex=-1;
            for(int d = 0;d<maxdis&&d<=j;d++){
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
                        /*double w1 =
                                exp(-sqrt((leftptr[p] - leftptr[q])*(leftptr[p] - leftptr[q])+
                                          (leftptr[p+1] - leftptr[q+1])*(leftptr[p+1] - leftptr[q+1])+
                                          (leftptr[p+2] - leftptr[q+2])*(leftptr[p+1] - leftptr[q+2]))/yc
                                -sqrt(i1*i1+j1*j1)/yg);
                        double w2 =
                                exp(-sqrt((rightptr[p_] - rightptr[q_])*(rightptr[p_] - rightptr[q_])+
                                             (rightptr[p_+1] - rightptr[q_+1])*(rightptr[p_+1] - rightptr[q_+1])+
                                             (rightptr[p_+2] - rightptr[q_+2])*(rightptr[p_+2] - rightptr[q_+2]))/yc
                                -sqrt(i1*i1+j1*j1)/yg);
                        */
                        double e1 = fabs(leftptr[q]-rightptr[q_])+
                                fabs(leftptr[q+1]-rightptr[q_+1])+
                                fabs(leftptr[q+2]-rightptr[q_+2]);

                        double w1 = w1buff[w1index++];
                        double w2 = w2buff[w2index++];
                        sum += w1*w2*e1;
                        sumw += w1*w2;
                    }
                }
                if(mmindex==-1||sum/sumw<mincost)
                    mincost = sum/sumw,mmindex = d;
            }
            disptr[i*size.width+j] = mmindex;
        }
        qDebug()<<"finished "<<i*100/size.height<<"%\r";
    }
    delete []w1buff;
    delete []w2buff;
}

void stereo_BM_AW_LRC(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize){
    if(left.size()!=right.size())
        return;
//    double yc=7,yg=36;
    double yc=10,yg=36;

    Size size = left.size();
    dis.create(size,CV_32F);

    float *disptr = (float*)dis.data;

    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;

    for(int i = 0;i<size.height;i++)
        for(int j = 0;j<size.width;j++)
            disptr[i*size.width+j] = -1;

    //double *mincost = new double[size.width];
    //double *
    double *w1buff = new double[(2*winsize+1)*(2*winsize+1)*size.width];
    double *w2buff = new double[(2*winsize+1)*(2*winsize+1)*size.width];
    double *cost = new double[size.width*maxdis];
    int *mincost2 = new int[size.width];


    for(int i = winsize;i+winsize<size.height;i++){
        for(int j = winsize;j+winsize<size.width;j++){
            int p = (i*size.width + j)*3;
            int windex = j*(2*winsize+1)*(2*winsize+1);
            for(int i1 = -winsize;i1<=winsize;i1++){
                for(int j1 = -winsize;j1<=winsize;j1++){
                    int q = ((i+i1)*size.width + j+j1)*3;
                    double w1 =
                            exp(-sqrt((leftptr[p] - leftptr[q])*(leftptr[p] - leftptr[q])+
                                      (leftptr[p+1] - leftptr[q+1])*(leftptr[p+1] - leftptr[q+1])+
                                      (leftptr[p+2] - leftptr[q+2])*(leftptr[p+1] - leftptr[q+2]))/yc
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
            double mincost;
            int mmindex=-1;
            for(int d = 0;d<maxdis&&d+winsize<=j;d++){
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
                cost[j*maxdis+d] = sum/sumw;
            }
        }
        //right WTA
        for(int j = winsize;j+winsize<size.width;j++){
            double mmin;
            int mindex = -1;
            for(int d = 0;d<maxdis&&d+j+winsize<size.width;d++){
                if(mindex==-1||cost[(j+d)*maxdis+d]<mmin)
                    mmin = cost[(j+d)*maxdis+d],mindex = d;
            }
            mincost2[j] = mindex;
        }
        //left WTA and consistence check
        for(int j = winsize;j+winsize<size.width;j++){
            double mmin;
            int mindex = -1;
            for(int d = 0;d<maxdis&&d+winsize<=j;d++){
                if(mindex==-1||cost[j*maxdis+d]<mmin)
                    mmin = cost[j*maxdis+d],mindex = d;
            }
//            mincost1[j] = mindex;
            int d2 = mincost2[j-mindex];
            double *tempcost = cost + j*maxdis;
            if(abs(mindex-d2)<=1){
                double te = mindex;
                if(mindex>0&&mindex<maxdis-1&&mindex+winsize<j){
                    double tem = 2*tempcost[mindex - 1]  + 2*tempcost[mindex+1] - 4*tempcost[mindex];
                    if(tem>0.001)te = te +(tempcost[mindex - 1]  - tempcost[mindex+1])/tem;
                }
                disptr[i*size.width+j] = te;
            }
        }

        qDebug()<<"finished "<<i*100/size.height<<"%\r";
    }
    delete []w1buff;
    delete []w2buff;
    delete []cost;
    delete []mincost2;
}
void testMyBM(){
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
//            stereo_BM(leftmat,rightmat,dis,7,100);
//            stereo_BM_segment(leftmat,rightmat,dis,7,40);
            stereo_BM_AW(leftmat,rightmat,dis,120,16);

            qDebug()<<"use time"<<clock() - t<<"ms"<<endl;
            dis.convertTo(vdisp,CV_8U);
            normalize(vdisp,vdisp,0,255,CV_MINMAX);
            imshow("BM ",vdisp);
        }
    }
}

///*
int main(int argc, char *argv[]){
    QApplication a(argc, argv);
    testMyBM();

    return a.exec();
}
//*/
