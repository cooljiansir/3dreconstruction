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
    double yc=7,yg=36;
//    double yc=50,yg=36;

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

//换成Lab空间
//L = L*100/255
//a = a-128
//b = b-128
void stereo_BM_AW_Lab(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize){
    if(left.size()!=right.size())
        return;
    double yc=7,yg=36;
//    double yc=50,yg=36;

    Size size = left.size();
    dis.create(size,CV_32F);

    float *disptr = (float*)dis.data;

    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;

    Mat leftlab,rightlab;
    cvtColor(left,leftlab,CV_BGR2Lab);
    cvtColor(right,rightlab,CV_BGR2Lab);

    unsigned char *leftlabptr = leftlab.data;
    unsigned char *rightlabptr = rightlab.data;

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
                            exp(-sqrt((leftlabptr[p] - leftlabptr[q])*(leftlabptr[p] - leftlabptr[q])
                                      *100.0/255.0*100.0/255.0+
                                      (leftlabptr[p+1] - leftlabptr[q+1])*(leftlabptr[p+1] - leftlabptr[q+1])+
                                      (leftlabptr[p+2] - leftlabptr[q+2])*(leftlabptr[p+2] - leftlabptr[q+2]))/yc
                            -sqrt(i1*i1+j1*j1)/yg);
                    double w2 =
                            exp(-sqrt((rightlabptr[p] - rightlabptr[q])*(rightlabptr[p] - rightlabptr[q])+
                                         (rightlabptr[p+1] - rightlabptr[q+1])*(rightlabptr[p+1] - rightlabptr[q+1])+
                                         (rightlabptr[p+2] - rightlabptr[q+2])*(rightlabptr[p+2] - rightlabptr[q+2]))/yc
                            -sqrt(i1*i1+j1*j1)/yg);
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
                    /*if(isnan(w1)){
                        printf("i:%d j:%d i1:%d j1:%d w1=nan\n",i,j,i1,j1);
                        printf("color distance:%lf\n",sqrt((leftlabptr[p] - leftlabptr[q])*(leftlabptr[p] - leftlabptr[q])
                          *100.0/255.0*100.0/255.0+
                          (leftlabptr[p+1] - leftlabptr[q+1])*(leftlabptr[p+1] - leftlabptr[q+1])+
                          (leftlabptr[p+2] - leftlabptr[q+2])*(leftlabptr[p+1] - leftlabptr[q+2])));
                        printf("(%d,%d,%d)\n",leftlabptr[p],leftlabptr[p+1],leftlabptr[p+2]);
                        printf("(%d,%d,%d)\n",leftlabptr[q],leftlabptr[q+1],leftlabptr[q+2]);
                        printf("L2:%d\n",(leftlabptr[p] - leftlabptr[q])*(leftlabptr[p] - leftlabptr[q]));
                        printf("a2:%d\n",(leftlabptr[p+1] - leftlabptr[q+1])*(leftlabptr[p+1] - leftlabptr[q+1]));
                        printf("b:%d\n",(leftlabptr[p+2] - leftlabptr[q+2]));
                        printf("b:%d\n",(leftlabptr[p+1] - leftlabptr[q+2]));
                        printf("b2:%d\n",(leftlabptr[p+2] - leftlabptr[q+2])*(leftlabptr[p+1] - leftlabptr[q+2]));
                        return;
                    }
                    if(isnan(w2)){
                        printf("i:%d j:%d i1:%d j1:%d w2=nan\n",i,j,i1,j1);
                        return;
                    }*/
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
                        /*
                         double w1 =
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
//                        double w1 =1;
//                        double w2 = 1;
//                        printf("w1=%lf w2=%lf\n",w1,w2);
                        sum += w1*w2*e1;
                        sumw += w1*w2;
                    }
                }
                if(mmindex==-1||sum/sumw<mincost)
                    mincost = sum/sumw,mmindex = d;
//                printf("%lf\n",sumw);
            }
            disptr[i*size.width+j] = mmindex;
        }
        qDebug()<<"finished "<<i*100/size.height<<"%\r";
    }
    delete []w1buff;
    delete []w2buff;
}

//发现彩色图片只能得到非常稀疏（大部分在边缘）,给/3
void stereo_BM_AW_Color(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize){
    if(left.size()!=right.size())
        return;
//    double yc=7,yg=36;
    double yc=20,yg=36;

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
                    /*double w1 =
                            exp(-sqrt((leftptr[p]+leftptr[p+1] +leftptr[p+2]
                                - leftptr[q]-leftptr[q+1]-leftptr[q+2])*
                            (leftptr[p]+leftptr[p+1] + leftptr[p+2]
                            - leftptr[q]-leftptr[q+1]-leftptr[q+2]))/3/yc
                            -sqrt(i1*i1+j1*j1)/yg);
                    double w2 =
                            exp(-sqrt((rightptr[p]+rightptr[p+1] +rightptr[p+2]
                                - rightptr[q]-rightptr[q+1]-rightptr[q+2])*
                            (rightptr[p]+rightptr[p+1] + rightptr[p+2]
                            - rightptr[q]-rightptr[q+1]-rightptr[q+2]))/3/yc
                            -sqrt(i1*i1+j1*j1)/yg);
                            *//*
                    double w1 =
                            exp(-sqrt((leftptr[p] - leftptr[q])*(leftptr[p] - leftptr[q])+
                                      (leftptr[p+1] - leftptr[q+1])*(leftptr[p+1] - leftptr[q+1])+
                                      (leftptr[p+2] - leftptr[q+2])*(leftptr[p+1] - leftptr[q+2]))/3/yc
                            -sqrt(i1*i1+j1*j1)/yg);
                    double w2 =
                            exp(-sqrt((rightptr[p] - rightptr[q])*(rightptr[p] - rightptr[q])+
                                         (rightptr[p+1] - rightptr[q+1])*(rightptr[p+1] - rightptr[q+1])+
                                         (rightptr[p+2] - rightptr[q+2])*(rightptr[p+2] - rightptr[q+2]))/3/yc
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
                        double e1 = fabs(leftptr[q]-rightptr[q_])+
                                fabs(leftptr[q+1]-rightptr[q_+1])+
                                fabs(leftptr[q+2]-rightptr[q_+2]);


                        /*double e1 = fabs((leftptr[q]+leftptr[q+1]+leftptr[q+2])
                                -(rightptr[q_]+rightptr[q_+1]+rightptr[q_+2]))/3;
                                */
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
void stereo_BM_AW_gray(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize){
    if(left.size()!=right.size())
        return;
    double yc=7,yg=36;
//    double yc=10,yg=36;

    Size size = left.size();
    dis.create(size,CV_32F);

    float *disptr = (float*)dis.data;

    Mat leftgray,rightgray;
    leftgray.create(left.size(),CV_8UC1);
    rightgray.create(right.size(),CV_8UC1);
    cvtColor(left,leftgray,CV_BGR2GRAY);
    cvtColor(right,rightgray,CV_BGR2GRAY);

    unsigned char *leftptr = leftgray.data;
    unsigned char *rightptr = rightgray.data;

    for(int i = 0;i<size.height;i++)
        for(int j = 0;j<size.width;j++)
            disptr[i*size.width+j] = -1;

    //double *mincost = new double[size.width];
    //double *
    double *w1buff = new double[(2*winsize+1)*(2*winsize+1)*size.width];
    double *w2buff = new double[(2*winsize+1)*(2*winsize+1)*size.width];


    for(int i = winsize;i+winsize<size.height;i++){
        for(int j = winsize;j+winsize<size.width;j++){
            int p = i*size.width + j;
            int windex = j*(2*winsize+1)*(2*winsize+1);
            for(int i1 = -winsize;i1<=winsize;i1++){
                for(int j1 = -winsize;j1<=winsize;j1++){
                    int q = (i+i1)*size.width + j+j1;
                    double w1 =
                            exp(-sqrt((leftptr[p] - leftptr[q])*(leftptr[p] - leftptr[q]))/yc
                            -sqrt(i1*i1+j1*j1)/yg);
                    double w2 =
                            exp(-sqrt((rightptr[p] - rightptr[q])*(rightptr[p] - rightptr[q]))/yc
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
                int p = i*size.width + j;
                int p_ = p-d;
                int w1index = j*(2*winsize+1)*(2*winsize+1);
                int w2index = (j-d)*(2*winsize+1)*(2*winsize+1);
                for(int i1 = -winsize;i1<=winsize;i1++){
                    for(int j1 = -winsize;j1<=winsize;j1++){

                        int q = (i+i1)*size.width + j+j1;
                        int q_ = q-d;

                        double e1 = fabs(leftptr[q]-rightptr[q_]);

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

void stereo_BM_FBS(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize,int winsizesmall){
    if((2*winsize+1)%(2*winsizesmall+1)!=0){
        qDebug()<<"winsize 不合法1"<<endl;
        return;
    }
    if((2*winsize+1)/(2*winsizesmall+1)%2==0){
        qDebug()<<"winsize 不合法2"<<endl;
        return;
    }
    if(left.size()!=right.size())
        return;
    double yc=7,yg=36;
//    double yc=10,yg=36;

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

    int winsmallwidth = winsizesmall*2+1;
    int nwin = (2*winsize+1)/winsmallwidth;
    double *s1buff = new double[nwin*nwin*3];
    double *s2buff = new double[nwin*nwin*3];


    for(int i = winsize;i+winsize<size.height;i++){
        for(int j = winsize;j+winsize<size.width;j++){
            int p = (i*size.width + j)*3;

            for(int k1 = 0;k1<nwin*nwin*3;k1++)
                s1buff[k1] = s2buff[k1] = 0;
            int windex0 = 0;
            //计算每个小窗口中的平均值
            for(int i1 = -winsize;i1<=winsize;i1++){
                for(int j1 = -winsize;j1<=winsize;j1++){
                    int q = ((i+i1)*size.width + j+j1)*3;
                    int i_ = windex0 / (2*winsize+1);
                    int j_ = windex0 % (2*winsize+1);
                    i_ /= winsmallwidth;
                    j_ /= winsmallwidth;
                    s1buff[(i_*nwin+j_)*3] += leftptr[q];
                    s1buff[(i_*nwin+j_)*3+1] += leftptr[q+1];
                    s1buff[(i_*nwin+j_)*3+2] += leftptr[q+2];

                    s2buff[(i_*nwin+j_)*3] += rightptr[q];
                    s2buff[(i_*nwin+j_)*3+1] += rightptr[q+1];
                    s2buff[(i_*nwin+j_)*3+2] += rightptr[q+2];
                    windex0++;
                }
            }

            int windex = j*(2*winsize+1)*(2*winsize+1);
            windex0 = 0;
            for(int i1 = -winsize;i1<=winsize;i1++){
                for(int j1 = -winsize;j1<=winsize;j1++){
                    int q = ((i+i1)*size.width + j+j1)*3;
                    int i_ = windex0 / (2*winsize+1);
                    int j_ = windex0 % (2*winsize+1);
                    i_ /= winsmallwidth;
                    j_ /= winsmallwidth;
                    double lp0 = s1buff[(i_*nwin+j_)*3]/winsmallwidth/winsmallwidth;
                    double lp1 = s1buff[(i_*nwin+j_)*3+1]/winsmallwidth/winsmallwidth;
                    double lp2 = s1buff[(i_*nwin+j_)*3+2]/winsmallwidth/winsmallwidth;

                    double rp0 = s2buff[(i_*nwin+j_)*3]/winsmallwidth/winsmallwidth;
                    double rp1 = s2buff[(i_*nwin+j_)*3+1]/winsmallwidth/winsmallwidth;
                    double rp2 = s2buff[(i_*nwin+j_)*3+2]/winsmallwidth/winsmallwidth;

                    double dis = (i_*winsmallwidth + winsizesmall - winsize)*
                                 (i_*winsmallwidth + winsizesmall - winsize)+
                                 (j_*winsmallwidth + winsizesmall - winsize)*
                                 (j_*winsmallwidth + winsizesmall - winsize);
                    dis = sqrt(dis);

                    double w1 = exp(-sqrt((leftptr[p] - lp0)*(leftptr[p] - lp0)+
                                          (leftptr[p+1] - lp1)*(leftptr[p+1] - lp1)+
                                          (leftptr[p+2] - lp2)*(leftptr[p+2] - lp2))/yc
                            -dis/yg);
                    double w2 = exp(-sqrt((rightptr[p] - rp0)*(rightptr[p] - rp0)+
                                          (rightptr[p+1] - rp1)*(rightptr[p+1] - rp1)+
                                          (rightptr[p+2] - rp2)*(rightptr[p+2] - rp2))/yc
                            -dis/yg);
                    /*
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
                            */
                    w1buff[windex] = w1;
                    w2buff[windex] = w2;
                    windex++;
                    windex0 ++;
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
    delete []s1buff;
    delete []s2buff;
}

void stereo_BM_AW_LRC(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize){
    if(left.size()!=right.size())
        return;
    double yc=7,yg=36;
//    double yc=7,yg=10;

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
                    double w1 = 1.0/(1+1/yc*sqrt((leftptr[p] - leftptr[q])*(leftptr[p] - leftptr[q])+
                                                          (leftptr[p+1] - leftptr[q+1])*(leftptr[p+1] - leftptr[q+1])+
                                                          (leftptr[p+2] - leftptr[q+2])*(leftptr[p+1] - leftptr[q+2])))
                            /(1+1/yg*sqrt(i1*i1+j1*j1));
                    double w2 = 1.0/(1+1/yc*sqrt((rightptr[p] - rightptr[q])*(rightptr[p] - rightptr[q])+
                                         (rightptr[p+1] - rightptr[q+1])*(rightptr[p+1] - rightptr[q+1])+
                                         (rightptr[p+2] - rightptr[q+2])*(rightptr[p+2] - rightptr[q+2])))
                            /(1+1/yg*sqrt(i1*i1+j1*j1));
//                    w1buff[windex] = w1;
//                    w2buff[windex] = w2;
                    w1buff[windex] = 1;
                    w2buff[windex] = 1;
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


/*
 *从上往下，BlockMatching
 *
 *Sum(x,y,d) = Sum(x,y-1,d) + s(x,y+winsize,d) - s(x,y-winsize-1,d)
 *s(x,y,d)   = s(x-1,y,d) + I(x+winsize,y,d)-I(x-winsize-1,y,d)
 *I(x,y,d)   = fabs(c(x,y)-c(x-d,y))
 *
 *
 */

void stereo_BM2(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize){
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
            if(j<maxdis){
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
    qDebug()<<"释放sad"<<endl;
    delete []s0_;
    qDebug()<<"释放s0"<<endl;
    delete []s1_;
    qDebug()<<"释放s1"<<endl;
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
//            stereo_BM(leftmat,rightmat,dis,3,40);
//            stereo_BM_segment(leftmat,rightmat,dis,7,40);
//            stereo_BM_AW(leftmat,rightmat,dis,40,7);
//            freopen("disresult.txt","w",stdout);
//            stereo_BM_AW_Lab(leftmat,rightmat,dis,140,8);
//            cout<<dis<<endl;
//            stereo_BM_AW_gray(leftmat,rightmat,dis,130,7);
//            stereo_BM_AW_Color(leftmat,rightmat,dis,130,7);
//            stereo_BM_FBS(leftmat,rightmat,dis,20,16,1);
//            stereo_BM_AW_LRC(leftmat,rightmat,dis,120,5);
            stereo_BM2(leftmat,rightmat,dis,40,3);


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
