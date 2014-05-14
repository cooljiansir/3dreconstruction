#include "stereo.h"
#include <QDebug>
#include <highgui.h>
#include <iostream>

using namespace std;
using namespace cv;


//左右一致性校验
//dis为输出视差
void LRC(int *cost,int maxdis,Size size,int maxdiff,int *dis){
    int *buff = new int[size.width*2];
    int *lcost = buff;
    int *rcost = buff+size.width;
    for(int i = 0;i<size.height;i++){
        int *costi = cost+i*size.width*maxdis;
        //L
        for(int j = 0;j<size.width;j++){
            int *costij = costi+j*maxdis;
            int minc = 1<<29;
            int mind ;
            for(int d = 0;d<maxdis&&d<=j;d++){
                if(costij[d]<minc)
                    minc = costij[d],mind = d;
            }
            lcost[j] = mind;
//            dis[i*size.width+j] = mind;
        }
//        continue;
        //R
        for(int j = 0;j<size.width;j++){
            int minc = 1<<29;
            int mind;
            for(int d = 0;d<maxdis&&j+d<size.width;d++){
                int *costjd = costi+(j+d)*maxdis;
                if(costjd[d]<minc)
                    minc = costjd[d],mind = d;
            }
            rcost[j] = mind;
        }
        //check
        for(int j = 0;j<size.width;j++){
            if(abs(lcost[j]-rcost[j])>maxdiff)
                dis[i*size.width+j]=  -1;
            else{
                dis[i*size.width+j] = lcost[j];
            }
        }
    }
    delete []buff;
}

//唯一性约束
void uniquenessC(int *cost,int *dis,int maxdis,Size size){
    int *buff = new int[size.width*2];
    int *rdis = buff;
    int *rcost = buff+size.width;
    for(int i = 0;i<size.height;i++){
        int *disi = dis+i*size.width;
        int *costi = cost+i*size.width*maxdis;
        for(int j = 0;j<size.width;j++)
            rdis[j] = -1,rcost[j]=1<<29;
        for(int j = 0;j<size.width;j++){
            int *costij = costi+j*maxdis;
            if(disi[j]!=-1){
                int d = disi[j];
                int jd = j-d;
                rdis[jd] = d;
                if(rcost[jd]<costij[d])
                    rdis[jd] = d,rcost[jd] = costij[d];
            }
        }
        for(int j = 0;j<size.width;j++){
            if(disi[j]!=-1){
                int d = disi[j];
                int dr = rdis[j-d];
                if(d != dr)
                    disi[j] = -1;
            }
        }
    }

    delete []buff;
}

void connetFilter(int *dis,Size size,int diff,int minarea){
    int *buff = new int[size.width*size.height*4];
    int *label = buff;
    int *labelArea = buff+size.width*size.height;
    int *quex = buff+2*size.width*size.height;
    int *quey = buff+3*size.width*size.height;
    int queindex;
    int labelcount = 0;


    for(int i = 0;i<size.height;i++){
        for(int j = 0;j<size.width;j++){
            label[i*size.width+j] = -1;
        }
    }
    for(int i = 0;i<size.height;i++){
        int di[4]={0,0,1,-1};
        int dj[4]={1,-1,0,0};
        int *labeli = label+i*size.width;
        int *disi = dis+i*size.width;
        for(int j = 0;j<size.width;j++){
            if(labeli[j]!=-1){
                if(labelArea[labeli[j]]<minarea)
                    disi[j] = -1;
            }
            else if(disi[j]!=-1){
                queindex = 0;
                quex[queindex] = j;
                quey[queindex] = i;
                queindex++;
                int count = 0;
                while(queindex>0){
                    queindex--;
                    int dx = quex[queindex];
                    int dy = quey[queindex];
                    count++;
                    label[dy*size.width+dx] = labelcount;
                    for(int k = 0;k<4;k++){
                        int mx = dx+dj[k];
                        int my = dy+di[k];
                        if(mx>=0&&mx<size.width&&my>=0&&my<size.height
                                &&dis[my*size.width+mx]!=-1
                                &&label[my*size.width+mx]==-1
                                &&abs(dis[my*size.width+mx]-dis[dy*size.width+dx])<=diff){
                            quex[queindex] = mx;
                            quey[queindex] = my;
                            queindex++;
                        }
                    }
                }
                labelArea[labelcount++] = count;
                if(count<minarea)
                    disi[j] = -1;
            }
        }
    }
    delete []buff;
}

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

//换成Lab空间
//L = L*100/255
//a = a-128
//b = b-128
void stereo_BM_AW_Cost(Mat &left,Mat &right,int *costin,int maxdis,int winsize){
    if(left.size()!=right.size())
        return;
    double yc=7,yg=36;
//    double yc=50,yg=36;

    Size size = left.size();


    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;

    Mat leftlab,rightlab;
    cvtColor(left,leftlab,CV_BGR2Lab);
    cvtColor(right,rightlab,CV_BGR2Lab);

    unsigned char *leftlabptr = leftlab.data;
    unsigned char *rightlabptr = rightlab.data;

    int *cost = new int[size.width*size.height*maxdis];

    Mat dis;
    dis.create(size,CV_32F);
    float *disptr = (float*)dis.data;

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
                    w1buff[windex] = w1;
                    w2buff[windex] = w2;
                    windex++;
                }
            }
        }
        int *costini = costin+i*size.width*maxdis;
        int *costi = cost+i*size.width*maxdis;
        for(int j = winsize;j+winsize<size.width;j++){
            double mincost;
            int mmindex=-1;
            int *costij = costi+j*maxdis;
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
//                        double e1 = fabs(leftptr[q]-rightptr[q_])+
//                                fabs(leftptr[q+1]-rightptr[q_+1])+
//                                fabs(leftptr[q+2]-rightptr[q_+2]);
                        double e1 = costini[(i1*size.width+j+j1)*maxdis+d];

                        double w1 = w1buff[w1index++];
                        double w2 = w2buff[w2index++];
                        sum += w1*w2*e1;
                        sumw += w1*w2;
                    }
                }
                costij[d] = sum/sumw;
            }
        }
        qDebug()<<"finished "<<i*100/size.height<<"%\r";
    }

    for(int k = 0;k<4;k++){
        int i0[4]={0,size.height-winsize,0,0},
                in[4]={winsize,size.height,size.height,size.height},
                j0[4]={0,0,0,size.width-winsize},
                jn[4]={size.width,size.width,winsize,size.width};
        for(int i = i0[k];i<in[k];i++){
            for(int j = j0[k];j<jn[k];j++){
                for(int d = 0;d<maxdis&&d<=j;d++){
                    costin[(i*size.width+j)*maxdis+d] = 0;
                }
            }
        }
    }
    for(int i = winsize;i+winsize<size.height;i++){
        int *costini = costin+i*size.width*maxdis;
        int *costi = cost + i*size.width*maxdis;
        for(int j = winsize;j+winsize<size.width;j++){
            int *costinij = costini+j*maxdis;
            int *costij = costi + j*maxdis;
            for(int d = 0;d<maxdis&&d<=j;d++){
                costinij[d] = costij[d];
            }
        }
    }


    delete []w1buff;
    delete []w2buff;
    delete []cost;
}

/*
 *FBS立体匹配,计算匹配代价
 *自适应权值和Box加速算法
 *
 *从上往下，BlockMatching
 *
 *Sum(x,y,d) = Sum(x,y-1,d) + s(x,y+winsize,d) - s(x,y-winsize-1,d)
 *s(x,y,d)   = s(x-1,y,d) + I(x+winsize,y,d)-I(x-winsize-1,y,d)
 *I(x,y,d)   = fabs(c(x,y)-c(x-d,y))
 *
 *
 */


void stereo_BM_FBS_COST_COM(Mat &left,Mat &right,int *costin,int maxdis,int winsize,int bigwinsize){
    if(left.size()!=right.size())
        return;
    double yc=7,yg=36;


    Size size = left.size();



    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;



    int border = bigwinsize*(2*winsize+1)+winsize;
    double *w1buff=new double[size.width*(2*bigwinsize+1)*(2*bigwinsize+1)];
    double *w2buff=new double[size.width*(2*bigwinsize+1)*(2*bigwinsize+1)];


    int *cost = new int[size.width*size.height*maxdis];
    //每个像素的邻域均值
    //3通道
    int *leftsum = new int[size.width*size.height*3];
    int *rightsum = new int[size.width*size.height*3];
    int *lefts = new int[size.height*3];
    int *rights = new int[size.height*3];

    //initial s
    for(int i=0;i<size.height;i++){
        lefts[i*3] = lefts[i*3+1] = lefts[i*3+2] = 0;
        rights[i*3] = rights[i*3+1] = rights[i*3+2] = 0;
        for(int j = 0;j<2*winsize+1;j++){
            int temp = i*size.width*3+3*j;
            lefts[i*3] += leftptr[temp];
            lefts[i*3+1] += leftptr[temp+1];
            lefts[i*3+2] += leftptr[temp+2];
            rights[i*3] += rightptr[temp];
            rights[i*3+1] += rightptr[temp+1];
            rights[i*3+2] += rightptr[temp+2];
        }
    }
    {//initial sum
        int j = winsize;
        for(int i=winsize;i+winsize<size.height;i++){
            int temp = (i*size.width+j)*3;
            leftsum[temp] = leftsum[temp+1] = leftsum[temp+2] = 0;
            rightsum[temp] = rightsum[temp+1] = rightsum[temp+2] = 0;
            for(int k = -winsize;k<=winsize;k++){
                leftsum[temp] += lefts[(i+k)*3];
                leftsum[temp+1] += lefts[(i+k)*3+1];
                leftsum[temp+2] += lefts[(i+k)*3+2];
                rightsum[temp] += rights[(i+k)*3];
                rightsum[temp+1] += rights[(i+k)*3+1];
                rightsum[temp+2] += rights[(i+k)*3+2];
            }
        }
    }
    //real calculate sum
    for(int j = winsize+1;j+winsize<size.width;j++){
        for(int i = 0;i<size.height;i++){
            int temp = (i*size.width+j+winsize)*3;
            int temp2 = (i*size.width+j-winsize-1)*3;
            lefts[i*3] = lefts[i*3]+leftptr[temp] - leftptr[temp2];
            lefts[i*3+1] = lefts[i*3+1]+leftptr[temp+1] - leftptr[temp2+1];
            lefts[i*3+2] = lefts[i*3+2]+leftptr[temp+2] - leftptr[temp2+2];
            rights[i*3] = rights[i*3] + rightptr[temp] - rightptr[temp2];
            rights[i*3+1] = rights[i*3+1] + rightptr[temp+1] - rightptr[temp2+1];
            rights[i*3+2] = rights[i*3+2] + rightptr[temp+2] - rightptr[temp2+2];
        }

        //first row
        {
            int les[3]={0,0,0};
            int ris[3]={0,0,0};
            for(int k = 0;k<2*winsize+1;k++){
                les[0] += lefts[k*3];
                les[1] += lefts[k*3+1];
                les[2] += lefts[k*3+2];
                ris[0] += rights[k*3];
                ris[1] += rights[k*3+1];
                ris[2] += rights[k*3+2];
            }
            int temp = (winsize*size.width+j)*3;
            leftsum[temp] = les[0];
            leftsum[temp+1] = les[1];
            leftsum[temp+2] = les[2];
            rightsum[temp] = ris[0];
            rightsum[temp+1] = ris[1];
            rightsum[temp+2] = ris[2];
        }
        for(int i = winsize+1;i+winsize<size.height;i++){
            int temp = (i*size.width+j)*3;
            int temp_p = ((i-1)*size.width+j)*3;
            leftsum[temp] = leftsum[temp_p] + lefts[(i+winsize)*3] - lefts[(i-winsize-1)*3];
            leftsum[temp+1] = leftsum[temp_p+1] + lefts[(i+winsize)*3+1] - lefts[(i-winsize-1)*3+1];
            leftsum[temp+2] = leftsum[temp_p+2] + lefts[(i+winsize)*3+2] - lefts[(i-winsize-1)*3+2];
            rightsum[temp] = rightsum[temp_p] + rights[(i+winsize)*3] - rights[(i-winsize-1)*3];
            rightsum[temp+1] = rightsum[temp_p+1] + rights[(i+winsize)*3+1] - rights[(i-winsize-1)*3+1];
            rightsum[temp+2] = rightsum[temp_p+2] + rights[(i+winsize)*3+2] - rights[(i-winsize-1)*3+2];
        }
    }

    //get mean
    int squ = (2*winsize+1)*(2*winsize+1);
    for(int i = 0;i<size.width*size.height*3;i++)
        leftsum[i]/= squ,rightsum[i] /= squ;

    //计算需要用到的sad范围行为：距上边界winsize到下边界winsize
    //采用循环结构
    //
    //sad(i,j,d) = sad(i,j-1,d)+s(i,j+winsize,d)-s(i,j-1-winsize,d);
    //s(i,j,d) = s(i-1,j,d) - I(i-winsize-1,j,d) + I(i+winsize,j,d)

    int sadrow = (2*bigwinsize+1)*(2*winsize+1)-winsize*2;
    int sadcols = size.width*maxdis;
    double *sad_ = new double[sadrow*sadcols];
    double *s    = new double[size.width*maxdis];


    //initial  the first row
    //sad
    for(int j = winsize;j+winsize<size.width;j++){
        unsigned char *leftptrij = leftptr+winsize*size.width*3+j*3;
        unsigned char *rightptrij = rightptr+winsize*size.width*3+j*3;
        double *sadj = sad_+j*maxdis;
        for(int d = 0;d<maxdis&&d+winsize<=j;d++){
            double sa=0;
            for(int i1 = -winsize;i1<=winsize;i1++){
                for(int j1=-winsize;j1<=winsize;j1++){
                    int temp = i1*size.width*3+j1*3;
//                    sa = sa + fabs(leftptrij[temp] - rightptrij[temp-d*3])
//                            + fabs(leftptrij[temp+1] - rightptrij[temp-d*3+1])
//                            + fabs(leftptrij[temp+2] - rightptrij[temp-d*3+2]);
                    sa = sa + costin[((winsize+i1)*size.width+j+j1)*maxdis+d];
                }
            }
            sadj[d] = sa;
        }
    }
    //s
    for(int j=0;j<size.width;j++){
        for(int d = 0;d<maxdis&&d<=j;d++){
            double st=0;
            for(int i=0;i<2*winsize+1;i++){
                int temp = (i*size.width+j)*3;
//                st = st + fabs(leftptr[temp]-rightptr[temp-d*3])
//                        + fabs(leftptr[temp+1]-rightptr[temp-d*3+1])
//                        + fabs(leftptr[temp+2]-rightptr[temp-d*3+2]);
                st = st + costin[(i*size.width+j)*maxdis+d];
            }
            s[j*maxdis+d] = st;
        }
    }
    //calculate until the first big window is ok
    for(int i=winsize+1;i+winsize<(2*bigwinsize+1)*(2*winsize+1);i++){
        double *sad = sad_ + (i-winsize)*sadcols;
        //calculate s
        for(int j=0;j<size.width;j++){
            int temp1 = ((i+winsize)*size.width+j)*3;
            int temp2 = ((i-winsize-1)*size.width+j)*3;
            int te1 = ((i+winsize)*size.width+j)*maxdis;
            int te2 = ((i-winsize-1)*size.width+j)*maxdis;
            for(int d = 0;d<maxdis&&d<=j;d++){
                double st;
//                st = fabs(leftptr[temp1]-rightptr[temp1-d*3])
//                        +fabs(leftptr[temp1+1]-rightptr[temp1-d*3+1])
//                        +fabs(leftptr[temp1+2]-rightptr[temp1-d*3+2]);
//                st = st - fabs(leftptr[temp2]-rightptr[temp2-3*d])
//                        - fabs(leftptr[temp2+1]-rightptr[temp2-3*d+1])
//                        - fabs(leftptr[temp2+2]-rightptr[temp2-3*d+2]);
                st = costin[te1+d] - costin[te2+d];
                s[j*maxdis+d] += st;
            }
        }
        //calculate sad
        for(int j=winsize;j+winsize<size.width;j++){
            for(int d = 0;d<maxdis&&d+winsize<=j;d++){
                if(d+winsize==j){
                    double sa = 0;
                    for(int j1=-winsize;j1<=winsize;j1++){
                        sa = sa + s[(j+j1)*maxdis+d];
                    }
                    sad[j*maxdis+d] = sa;
                }else{
                    sad[j*maxdis+d] = sad[(j-1)*maxdis+d] - s[(j-1-winsize)*maxdis+d]
                            +s[(j+winsize)*maxdis+d];
                }

            }
        }
    }

    for(int i = border;i+border<size.height;i++){
        unsigned char *leftptri = leftptr+i*size.width*3;
        unsigned char *rightptri = rightptr +i*size.width*3;
        //calculate w1 w2 first
        for(int j = border;j+border<size.width;j++){
            unsigned char *leftptrij = leftptri + j*3;
            unsigned char *rightptrij = rightptri + j*3;
            int fastindexin = 0;
            int windex=j*(2*bigwinsize+1)*(2*bigwinsize+1);
            for(int i1=-bigwinsize;i1<=bigwinsize;i1++){
                for(int j1=-bigwinsize;j1<=bigwinsize;j1++){

                    int r = i+i1*(2*winsize+1);
                    int c = j+j1*(2*winsize+1);
                    int temp = (r*size.width+c)*3;
                    double w1 =
                            exp(-sqrt((leftptrij[0] - leftsum[temp])*(leftptrij[0] - leftsum[temp])+
                                      (leftptrij[1] - leftsum[temp+1])*(leftptrij[1] - leftsum[temp+1])+
                                      (leftptrij[2] - leftsum[temp+2])*(leftptrij[2] - leftsum[temp+2]))/yc
                            -sqrt(i1*i1+j1*j1)*(2*winsize+1)/yg);
                    double w2 =
                            exp(-sqrt((rightptrij[0] - rightsum[temp])*(rightptrij[0] - rightsum[temp])+
                                         (rightptrij[1] - rightsum[temp+1])*(rightptrij[1] - rightsum[temp+1])+
                                         (rightptrij[2] - rightsum[temp+1])*(rightptrij[2] - rightsum[temp+1]))/yc
                            -sqrt(i1*i1+j1*j1)*(2*winsize+1)/yg);
                    w1buff[windex] = w1;
                    w2buff[windex] = w2;
                    windex++;
                }
            }
        }

        //calculate sad
        if(i>border){
            int in = i+bigwinsize*(2*winsize+1);
            double *sad = sad_ + (in-winsize)*sadcols%(sadrow*sadcols);
            //calculate s
            for(int j=0;j<size.width;j++){
                int temp1 = ((in+winsize)*size.width+j)*3;
                int temp2 = ((in-winsize-1)*size.width+j)*3;
                int te1 = ((in+winsize)*size.width+j)*maxdis;
                int te2 = ((in-winsize-1)*size.width+j)*maxdis;
                for(int d = 0;d<maxdis&&d<=j;d++){
                    double st;
//                    st = fabs(leftptr[temp1]-rightptr[temp1-d*3])
//                            +fabs(leftptr[temp1+1]-rightptr[temp1-d*3+1])
//                            +fabs(leftptr[temp1+2]-rightptr[temp1-d*3+2]);
//                    st = st - fabs(leftptr[temp2]-rightptr[temp2-3*d])
//                            - fabs(leftptr[temp2+1]-rightptr[temp2-3*d+1])
//                            - fabs(leftptr[temp2+2]-rightptr[temp2-3*d+2]);
                    st = costin[te1+d] - costin[te2+d];
                    s[j*maxdis+d] += st;
                }
            }
            //calculate sad
            for(int j=winsize;j+winsize<size.width;j++){
                for(int d = 0;d<maxdis&&d+winsize<=j;d++){
                    if(d+winsize==j){
                        double sa = 0;
                        for(int j1=-winsize;j1<=winsize;j1++){
                            sa = sa + s[(j+j1)*maxdis+d];
                        }
                        sad[j*maxdis+d] = sa;
                    }else{
                        sad[j*maxdis+d] = sad[(j-1)*maxdis+d] - s[(j-1-winsize)*maxdis+d]
                                +s[(j+winsize)*maxdis+d];
                    }
                }
            }
        }

        int *costi = cost+i*size.width*maxdis;
        //real dealing
        for(int j = border;j+border<size.width;j++){
            unsigned char *leftptrij = leftptri+j*3;
            unsigned char *rightptrij = rightptri+j*3;
            double mmin=1<<29;
            int mindex;
            int *costij = costi+j*maxdis;
            for(int d=0;d<maxdis&&d+border<=j;d++){
                double sum=0;
                double sumw=0;
                int windex=j*(2*bigwinsize+1)*(2*bigwinsize+1);
                int fastindexin = 0;
                for(int i1=-bigwinsize;i1<=bigwinsize;i1++){
                    for(int j1=-bigwinsize;j1<=bigwinsize;j1++){
                        double w1 = w1buff[windex];
                        double w2 = w1buff[windex];
                        windex++;
                        int r = i+i1*(2*winsize+1);
                        int c = j+j1*(2*winsize+1);
                        double *sadn = sad_ + (r-winsize)*sadcols%(sadcols*sadrow);
                        sum += w1*w2*sadn[c*maxdis+d];
                        sumw += w1*w2;
                    }
                }
                costij[d] = sum/sumw/9;
//                if(sum/sumw<mmin)
//                    mmin=sum/sumw,mindex = d;
            }
//            disptr[i*size.width+j] = mindex;
        }
    }

    for(int k = 0;k<4;k++){
        int i0[4]={0,size.height-border,0,0},
                in[4]={border,size.height,size.height,size.height},
                j0[4]={0,0,0,size.width-border},
                jn[4]={size.width,size.width,border,size.width};
        for(int i = i0[k];i<in[k];i++){
            for(int j = j0[k];j<jn[k];j++){
                for(int d = 0;d<maxdis&&d<=j;d++){
                    costin[(i*size.width+j)*maxdis+d] = 0;
                }
            }
        }
    }
    for(int i = border;i+border<size.height;i++){
        int *costini = costin+i*size.width*maxdis;
        int *costi = cost + i*size.width*maxdis;
        for(int j = border;j+border<size.width;j++){
            int *costinij = costini+j*maxdis;
            int *costij = costi + j*maxdis;
            for(int d = 0;d<maxdis&&d<=j;d++){
                costinij[d] = costij[d];
            }
        }
    }


    delete []w1buff;
    delete []w2buff;
    delete []sad_;
    delete []s;
    delete []lefts;
    delete []leftsum;
    delete []rights;
    delete []rightsum;
    delete []cost;
}

/*
 *抛物线拟合
 *d = d + (C(-1)-C(1))/(2C(-1)-4C(0)+2C(1))
 *
 */

void subpixel(int *cost,Size size,int *disint,int maxdis,float *dis){
    for(int i = 0;i<size.height;i++){
        int *costi = cost+i*size.width*maxdis;
        int *disinti = disint+i*size.width;
        float *disi = dis+i*size.width;
        for(int j = 0;j<size.width;j++){
            int *costij = costi + j*maxdis;
            int d = disinti[j];
            if(d>0&&d<maxdis){
                int c_1=costij[d-1],c0 = costij[d],c1 = costij[d+1];
                int bo = c_1-2*c0+c1;
                if(bo!=0)
                    disi[j] = d+(c_1-c1)/(bo*2.0);
                else
                    disi[j] = d;
            }else
                disi[j] = d;
        }
    }
}

void stereo_MSGM_MY(Mat &left,Mat &right,Mat &dis,int maxdis,int P1,int P2,int iter,
                    int winsize,int prefilter,bool BT,bool lrc,bool uniq,bool filt,bool subpix,int AW_FBS){
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


    if(AW_FBS==1){
        stereo_BMBox_BT(left,right,cost,maxdis,0,prefilter,BT);
        stereo_BM_AW_Cost(left,right,cost,maxdis,winsize);
    }else if(AW_FBS==2){
        stereo_BMBox_BT(left,right,cost,maxdis,0,prefilter,BT);
        stereo_BM_FBS_COST_COM(left,right,cost,maxdis,1,winsize);
    }
    else{
        stereo_BMBox_BT(left,right,cost,maxdis,winsize,prefilter,BT);
    }

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


    if(lrc)
        LRC(cost,maxdis,size,2,disint);
    if(uniq)
        uniquenessC(cost,disint,maxdis,size);
    if(filt)
        connetFilter(disint,size,2,200);

    if(subpix)
        subpixel(cost,size,disint,maxdis,disptr);
    else
        for(int i = 0;i<size.width*size.height;i++)
            disptr[i] = disint[i];

    delete []cost;
    delete []LrSum;
    delete []LrBuff_;
    delete []minLr_;
    delete []disint;
}

//DP算法
void stereoDP_(Mat &left,Mat &right,Mat &dis,int maxdis,double P,int winsize,int prefilter,bool BT,int AW_FBS,
               bool lrc,bool filt,bool uniq,bool subpix){
    if(left.size()!=right.size())
        return;
    Size size = left.size();

    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;

    dis.create(size,CV_32F);
    float *disptr = (float*)dis.data;

    //[-1,width)
    //[-1,width)二位数组
    //边界值
    int costwidth = size.width+1;
    double *cost_ = new double[costwidth*costwidth];
    double *cost = cost_+costwidth+1;
    //用于保存路径
    char   *path = new char[size.width*size.width];

    int *costfr = new int[size.width*size.height*maxdis];
    int *disint = new int[size.width*size.height];

    if(AW_FBS==1){
        stereo_BMBox_BT(left,right,costfr,maxdis,0,prefilter,BT);
        stereo_BM_AW_Cost(left,right,costfr,maxdis,winsize);
    }else if(AW_FBS==2){
        stereo_BMBox_BT(left,right,costfr,maxdis,0,prefilter,BT);
        stereo_BM_FBS_COST_COM(left,right,costfr,maxdis,1,winsize);
    }
    else{
        stereo_BMBox_BT(left,right,costfr,maxdis,winsize,prefilter,BT);
    }


    //边界值初始化
    //至始至终都没有改变，所以，初始化一次就行了
    for(int i = 0;i<costwidth;i++)
        cost_[i] = cost_[i*costwidth] = 0;

    for(int i=0;i<size.width*size.height;i++)
        disptr[i] = -1;

    for(int i = 0;i<size.height;i++){
        unsigned char *leftptri = leftptr+i*size.width*3;
        unsigned char *rightptri = rightptr+i*size.width*3;
        int *costfri  = costfr+i*size.width*maxdis;
        for(int a = 0;a<size.width;a++){
            double *costa = cost+a*costwidth;
            double *costa_1 = costa-costwidth;
            char *patha = path+a*size.width;
            int *costfria = costfri+a*maxdis;
            for(int b = 0;b<size.width;b++){
                //A (a,b) is matched
                double A;
                if(a-b<0||a-b>=maxdis)
                    A=1<<29;
                else {
                    A = costa_1[b-1]
//                                + fabs(leftptri[a*3]-rightptri[b*3])
//                                + fabs(leftptri[a*3+1]-rightptri[b*3+1])
//                                + fabs(leftptri[a*3+2]-rightptri[b*3+2]);
                            +costfria[a-b];

                }
                //a is blocked
                double B = costa_1[b] + P;
                //b is blocked
                double C = costa[b-1] + P;
                if(A<B){
                    if(A<C){//A is min
                        costa[b] = A;
                        patha[b] = 1;
                    }else{//C is min
                        costa[b] = C;
                        patha[b] = 3;
                    }
                }else{
                    if(B<C){//B is min
                        costa[b] = B;
                        patha[b] = 2;
                    }else{//C is min
                        costa[b] = C;
                        patha[b] = 3;
                    }
                }
            }
        }
        int indexa = size.width-1;
        int indexb = size.width-1;
        float *disptri = disptr+i*size.width;
        //qDebug()<<"MAX" <<cost[indexa*costwidth+indexb];
        while(indexa>=0&&indexb>=0){
            if(path[indexa*size.width+indexb]==1){
//                disptri[indexa] = indexa-indexb;
                disint[i*size.width+indexa] = indexa-indexb;
                indexa--;
                indexb--;
            }else if(path[indexa*size.width+indexb]==2){
                indexa--;
            }else if(path[indexa*size.width+indexb]==3){
                indexb--;
            }
        }

    }
    if(lrc)
        LRC(costfr,maxdis,size,2,disint);
    if(uniq)
        uniquenessC(costfr,disint,maxdis,size);
    if(filt)
        connetFilter(disint,size,2,200);

    if(subpix)
        subpixel(costfr,size,disint,maxdis,disptr);
    else
        for(int i = 0;i<size.width*size.height;i++)
            disptr[i] = disint[i];
    delete []cost_;
    delete []path;
    delete []costfr;
    delete []disint;
}

//WTA BM 算法
void WTA_(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize,bool BT,int prefilter,
          bool lrc,bool uniq,bool filt,bool subpix,int AW_FBS){
    if(left.size()!=right.size())
        return;
    Size size = left.size();
    int *cost = new int[size.width*size.height*maxdis];
    int *disint = new int[size.width*size.height];

    if(AW_FBS==1){
        stereo_BMBox_BT(left,right,cost,maxdis,0,prefilter,BT);
        stereo_BM_AW_Cost(left,right,cost,maxdis,winsize);
    }else if(AW_FBS==2){
        stereo_BMBox_BT(left,right,cost,maxdis,0,prefilter,BT);
        stereo_BM_FBS_COST_COM(left,right,cost,maxdis,1,winsize);
    }
    else{
        stereo_BMBox_BT(left,right,cost,maxdis,winsize,prefilter,BT);
    }
    for(int i = 0;i<size.height;i++){
        int *costi = cost+i*size.width*maxdis;
        for(int j = 0;j<size.width;j++){
            int *costij = costi + j*maxdis;
            int mincost = 1<<29;
            int mind ;
            for(int d = 0;d<maxdis&&d<=j;d++){
                if(costij[d]<mincost)
                    mincost = costij[d],mind = d;
            }
            disint[i*size.width+j] = mind;
        }
    }

    if(lrc)
        LRC(cost,maxdis,size,2,disint);
    if(uniq)
        uniquenessC(cost,disint,maxdis,size);
    if(filt)
        connetFilter(disint,size,2,200);

    dis.create(size,CV_32F);
    float *disptr = (float*)dis.data;
    if(subpix){
        subpixel(cost,size,disint,maxdis,disptr);
    }
    else{
        for(int i = 0;i<size.width*size.height;i++)
            disptr[i] = disint[i];
    }
    delete []cost;
    delete []disint;
}

Stereo::Stereo()
{
    //初始化默认值
    method = METHOD_CVSGBM;
    maxdis = 16*8;
    costCalulate_BT = true;
    costCalulate_SOBEL = true;
    costAggregation = COST_AGGREGATION_FW;
    winsize = 7;
    computeDisparity = COMPUTE_DISPARITY_ITER_SGM;
    P1 = 8;
    P2 = 32;
    iterTimes = 5;
    disRefineLRC = false;
    disRefineUnique = true;
    disRefineFilter = true;
    disRefineSubPixel = true;
}

void Stereo::stereoMatch(Mat &leftmat, Mat &rightmat, Mat &dismat){
    if(this->method==METHOD_CVSGBM){
        int SADWindowSize = this->winsize;
        int numberOfDisparities = maxdis;

        StereoSGBM sgbm;
        sgbm.preFilterCap = 63;
        sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

        int cn = 1;

        sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.minDisparity = 0;
        sgbm.numberOfDisparities = numberOfDisparities;
        sgbm.uniquenessRatio = 10;
        sgbm.speckleWindowSize = 100;
        sgbm.speckleRange = 32;
        sgbm.disp12MaxDiff = 1;
        sgbm.fullDP = true;

        Mat sgbmmat;
        Mat leftmatgray,rightmatgray;
        cvtColor(leftmat,leftmatgray,CV_BGR2GRAY);
        cvtColor(rightmat,rightmatgray,CV_BGR2GRAY);

        sgbm(leftmatgray,rightmatgray,sgbmmat);
        dismat.create(leftmat.size(),CV_32F);
        Size size = leftmat.size();
        for(int i = 0;i<size.height;i++)
            for(int j = 0;j<size.width;j++)
                *dismat.ptr<float>(i,j) = (*sgbmmat.ptr<short int>(i,j))/16.0;
    }
    else if(this->method==METHOD_CVBM){
        StereoBM bm;
        bm.state->preFilterCap = 31;
        bm.state->SADWindowSize = this->winsize;
        bm.state->minDisparity = 0;
        bm.state->numberOfDisparities = this->maxdis;
        bm.state->textureThreshold = 10;
        bm.state->uniquenessRatio = 15;
        bm.state->speckleWindowSize = 100;
        bm.state->speckleRange = 32;
        bm.state->disp12MaxDiff = 1;

        Mat leftmatgray,rightmatgray;
        cvtColor(leftmat,leftmatgray,CV_BGR2GRAY);
        cvtColor(rightmat,rightmatgray,CV_BGR2GRAY);

        bm(leftmatgray,rightmatgray,dismat,CV_32F);
    }
    else if(this->method==METHOD_CUSTOM){
        int AW_FBS = 0;
        if(this->costAggregation==Stereo::COST_AGGREGATION_AW)
            AW_FBS = 1;
        else if(this->costAggregation==Stereo::COST_AGGREGATION_FBS)
            AW_FBS = 2;
        if(this->computeDisparity==Stereo::COMPUTE_DISPARITY_ITER_SGM){
            stereo_MSGM_MY(leftmat,rightmat,dismat,
                       maxdis,P1,P2,this->iterTimes,this->winsize/2,
                       this->costCalulate_SOBEL?64:-1,this->costCalulate_BT,
                       this->disRefineLRC,this->disRefineUnique,this->disRefineFilter,
                           this->disRefineSubPixel,AW_FBS                           );
        }else if(this->computeDisparity==Stereo::COMPUTE_DISPARITY_WTA){
            WTA_(leftmat,rightmat,dismat,maxdis,winsize/2,this->costCalulate_BT,
                 this->costCalulate_SOBEL?64:-1,this->disRefineLRC
                 ,this->disRefineUnique,this->disRefineFilter,this->disRefineSubPixel,AW_FBS
                 );
        }else if(this->computeDisparity==Stereo::COMPUTE_DISPARITY_DP){
            stereoDP_(leftmat,rightmat,dismat,maxdis,P1,winsize/2,this->costCalulate_SOBEL?64:-1,
                      this->costCalulate_BT,AW_FBS,this->disRefineLRC,this->disRefineFilter,this->disRefineUnique
                      ,this->disRefineSubPixel);
        }
    }
}

void Stereo::save(Mat &left, Mat &right, Mat &dis, const char path[]){
    freopen(path,"w",stdout);
    //参数
    cout<<method<<endl;
    cout<<maxdis<<endl;
    cout<<costCalulate_BT<<endl;
    cout<<costCalulate_SOBEL<<endl;
    cout<<costAggregation<<endl;
    cout<<winsize<<endl;
    cout<<computeDisparity<<endl;
    cout<<P1<<endl;//DP算法的参数//Iter-SGM的参数
    cout<<P2<<endl;//Iter-SGM的参数
    cout<<iterTimes<<endl;//Iter-SGM的参数
    cout<<disRefineLRC<<endl;
    cout<<disRefineUnique<<endl;
    cout<<disRefineFilter<<endl;
    cout<<disRefineSubPixel<<endl;

//    cout<<left<<endl;
//    cout<<right<<endl;
//    cout<<dis<<endl;
    saveMat(left);
    saveMat(right);
    saveMat(dis);
    fclose(stdout);
}

void Stereo::read(Mat &left, Mat &right, Mat &dis, const char path[]){
    freopen(path,"r",stdin);
    //参数
    cin>>method;
    cin>>maxdis;
    cin>>costCalulate_BT;
    cin>>costCalulate_SOBEL;
    cin>>costAggregation;
    cin>>winsize;
    cin>>computeDisparity;
    cin>>P1;//DP算法的参数//Iter-SGM的参数
    cin>>P2;//Iter-SGM的参数
    cin>>iterTimes;//Iter-SGM的参数
    cin>>disRefineLRC;
    cin>>disRefineUnique;
    cin>>disRefineFilter;
    cin>>disRefineSubPixel;



    readMat(left);
    readMat(right);
    readMat(dis);
    fclose(stdin);
//    cin>>left;
//    cin>>right;
//    cin>>dis;
}

void Stereo::readMat(Mat &mat){
    Size size;
    int chann;
    int temp;
    cin>>size.width>>size.height>>chann;
    if(chann==3){
        mat.create(size,CV_8UC3);
        unsigned char  *matptr = mat.data;
        for(int i = 0;i<size.height;i++){
            for(int j = 0;j<size.width;j++){
                for(int k = 0;k<chann;k++){
                    cin>>temp;
                    *matptr = temp;
                    matptr++;
                }
            }
        }
    }else{
        mat.create(size,CV_32F);
        float *matptr = (float *)mat.data;
        for(int i = 0;i<size.height;i++){
            for(int j = 0;j<size.width;j++){
                for(int k = 0;k<chann;k++){
                    cin>>*matptr;
                    matptr++;
                }
            }
        }
    }


}


void Stereo::saveMat(Mat &mat){
    Size size = mat.size();
    int chann = mat.channels();
    int temp;
    cout<<size.width<<" "<<size.height<<" "<<chann<<endl;
    if(chann==3){
        unsigned char  *matptr = mat.data;
        for(int i = 0;i<size.height;i++){
            for(int j = 0;j<size.width;j++){
                for(int k = 0;k<chann;k++){
                    temp = *matptr;
                    cout<<temp<<" ";
                    matptr++;
                }
            }
            cout<<endl;
        }
    }else{
        float *matptr = (float *)mat.data;
        for(int i = 0;i<size.height;i++){
            for(int j = 0;j<size.width;j++){
                for(int k = 0;k<chann;k++){
                    cout<<*matptr<<" ";
                    matptr++;
                }
            }
            cout<<endl;
        }
    }
}
