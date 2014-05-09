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
    qDebug()<<"释放完sad"<<endl;
    delete []s0_;
    qDebug()<<"释放完s0"<<endl;
    delete []s1_;
    qDebug()<<"释放完s1"<<endl;

}


//换成Lab空间
//L = L*100/255
//a = a-128
//b = b-128
void stereo_BM_AW_Lab_Pro(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize){
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



/*
 *FBS立体匹配
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


void stereo_BM_FBS2Pro(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize,int bigwinsize){
    if(left.size()!=right.size())
        return;
    double yc=7,yg=36;


    Size size = left.size();

    dis.create(size,CV_32F);



    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;

    float *disptr = (float*)dis.data;

    for(int i = 0;i<size.height;i++)
        for(int j = 0;j<size.width;j++)
            disptr[i*size.width+j] = -1;

    int border = bigwinsize*(2*winsize+1)+winsize;
    double *w1buff=new double[size.width*(2*bigwinsize+1)*(2*bigwinsize+1)];
    double *w2buff=new double[size.width*(2*bigwinsize+1)*(2*bigwinsize+1)];


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
                    sa = sa + fabs(leftptrij[temp] - rightptrij[temp-d*3])
                            + fabs(leftptrij[temp+1] - rightptrij[temp-d*3+1])
                            + fabs(leftptrij[temp+2] - rightptrij[temp-d*3+2]);
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
                st = st + fabs(leftptr[temp]-rightptr[temp-d*3])
                        + fabs(leftptr[temp+1]-rightptr[temp-d*3+1])
                        + fabs(leftptr[temp+2]-rightptr[temp-d*3+2]);
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
            for(int d = 0;d<maxdis&&d<=j;d++){
                double st;
                st = fabs(leftptr[temp1]-rightptr[temp1-d*3])
                        +fabs(leftptr[temp1+1]-rightptr[temp1-d*3+1])
                        +fabs(leftptr[temp1+2]-rightptr[temp1-d*3+2]);
                st = st - fabs(leftptr[temp2]-rightptr[temp2-3*d])
                        - fabs(leftptr[temp2+1]-rightptr[temp2-3*d+1])
                        - fabs(leftptr[temp2+2]-rightptr[temp2-3*d+2]);
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
                for(int d = 0;d<maxdis&&d<=j;d++){
                    double st;
                    st = fabs(leftptr[temp1]-rightptr[temp1-d*3])
                            +fabs(leftptr[temp1+1]-rightptr[temp1-d*3+1])
                            +fabs(leftptr[temp1+2]-rightptr[temp1-d*3+2]);
                    st = st - fabs(leftptr[temp2]-rightptr[temp2-3*d])
                            - fabs(leftptr[temp2+1]-rightptr[temp2-3*d+1])
                            - fabs(leftptr[temp2+2]-rightptr[temp2-3*d+2]);
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

        //real dealing
        for(int j = border;j+border<size.width;j++){
            unsigned char *leftptrij = leftptri+j*3;
            unsigned char *rightptrij = rightptri+j*3;
            double mmin=1<<29;
            int mindex;
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
                if(sum/sumw<mmin)
                    mmin=sum/sumw,mindex = d;
            }
            disptr[i*size.width+j] = mindex;
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
}

void stereoDP2(Mat &left,Mat &right,Mat &dis,int maxdis,double P){
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

    //边界值初始化
    //至始至终都没有改变，所以，初始化一次就行了
    for(int i = 0;i<costwidth;i++)
        cost_[i] = cost_[i*costwidth] = 0;

    for(int i=0;i<size.width*size.height;i++)
        disptr[i] = -1;

    for(int i = 0;i<size.height;i++){
        unsigned char *leftptri = leftptr+i*size.width*3;
        unsigned char *rightptri = rightptr+i*size.width*3;
        for(int a = 0;a<size.width;a++){
            double *costa = cost+a*costwidth;
            double *costa_1 = costa-costwidth;
            char *patha = path+a*size.width;
            for(int b = 0;b<size.width;b++){
                //A (a,b) is matched
                double A;
                if(a-b<0||a-b>=maxdis)
                    A=1<<29;
                else A = costa_1[b-1]
                                + fabs(leftptri[a*3]-rightptri[b*3])
                                + fabs(leftptri[a*3+1]-rightptri[b*3+1])
                                + fabs(leftptri[a*3+2]-rightptri[b*3+2]);

                /*double A = costa_1[b-1]
                                + (leftptri[a*3]-rightptri[b*3])*(leftptri[a*3]-rightptri[b*3])
                                + (leftptri[a*3+1]-rightptri[b*3+1])*(leftptri[a*3+1]-rightptri[b*3+1])
                                + (leftptri[a*3+2]-rightptri[b*3+2])*(leftptri[a*3+2]-rightptri[b*3+2]);
                                */
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
                disptri[indexa] = indexa-indexb;
                indexa--;
                indexb--;
            }else if(path[indexa*size.width+indexb]==2){
                indexa--;
            }else if(path[indexa*size.width+indexb]==3){
                indexb--;
            }
        }

    }
    delete []cost_;
    delete []path;
}

/*
 *AW 算法+DP算法
 *采用LAB空间的自适应权值算法计算得到匹配代价
 *再通过DP算法进行匹配
 *
 *
 *
 *
 *
 */
void stereo_BM_AW_DP(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize,double P){
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
    double *cost = new double[size.width*size.width];
    int DPwidth = size.width+1;
    double *DP_ = new double[DPwidth*DPwidth];
    double *DP = DP_+DPwidth+1;
    char *path = new char[size.width*size.width];

    //initial DP
    for(int i = 0;i<DPwidth;i++)
        DP_[i] = DP_[i*DPwidth] = 0;

    for(int i = winsize;i+winsize<size.height;i++){
        unsigned char *leftptri = leftptr+i*size.width*3;
        unsigned char *rightptri = rightptr+i*size.width*3;

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
                cost[j*size.width+(j-d)] = sum/sumw;

            }

        }

        for(int a = 0;a<size.width;a++){
            double *DPa = DP+a*DPwidth;
            double *DPa_1 = DPa - DPwidth;
            char *patha = path+a*size.width;
            for(int b = 0;b<size.width;b++){
                double A;
                if(a<winsize||b<winsize||a-b>=maxdis||a-b<0||a+winsize>=size.width)
                    A=1<<29;
//                    A = DPa_1[b-1]
//                                + fabs(leftptri[a*3]-rightptri[b*3])
//                                + fabs(leftptri[a*3+1]-rightptri[b*3+1])
//                                + fabs(leftptri[a*3+2]-rightptri[b*3+2]);
                else {
                    A = DPa_1[b-1]+cost[a*size.width+b];
                }

                double B = DPa_1[b] + P;
                double C = DPa[b-1] + P;
                if(A<B){
                    if(A<C){//A is min
                        DPa[b] = A;
                        patha[b] = 1;
                    }else{//C is min
                        DPa[b] = C;
                        patha[b] = 3;
                    }
                }else{
                    if(B<C){//B is min
                        DPa[b] = B;
                        patha[b] = 2;
                    }else{//C is min
                        DPa[b] = C;
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
                disptri[indexa] = indexa-indexb;
                indexa--;
                indexb--;
            }else if(path[indexa*size.width+indexb]==2){
                indexa--;
            }else if(path[indexa*size.width+indexb]==3){
                indexb--;
            }
        }
        qDebug()<<"finished "<<i*100/size.height<<"%\r";
    }
    delete []w1buff;
    delete []w2buff;
    delete []DP_;
    delete []path;
    delete []cost;
}


/*
 *FBS+DP
 *采用FBS算法计算像素匹配代价，采用DP求得视察值
 *
 *FBS立体匹配
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


void stereo_BM_FBS_DP(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize,int bigwinsize,double P){
    if(left.size()!=right.size())
        return;
    double yc=7,yg=36;


    Size size = left.size();

    dis.create(size,CV_32F);



    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;

    float *disptr = (float*)dis.data;

    for(int i = 0;i<size.height;i++)
        for(int j = 0;j<size.width;j++)
            disptr[i*size.width+j] = -1;

    int border = bigwinsize*(2*winsize+1)+winsize;
    double *w1buff=new double[size.width*(2*bigwinsize+1)*(2*bigwinsize+1)];
    double *w2buff=new double[size.width*(2*bigwinsize+1)*(2*bigwinsize+1)];


    //每个像素的邻域均值
    //3通道
    int *leftsum = new int[size.width*size.height*3];
    int *rightsum = new int[size.width*size.height*3];
    int *lefts = new int[size.height*3];
    int *rights = new int[size.height*3];

    double *cost = new double[size.width*size.width];
    int DPwidth = size.width+1;
    double *DP_ = new double[DPwidth*DPwidth];
    double *DP = DP_+DPwidth+1;
    char *path = new char[size.width*size.width];

    //initial DP
    for(int i = 0;i<DPwidth;i++)
        DP_[i] = DP_[i*DPwidth] = 0;

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
                    sa = sa + fabs(leftptrij[temp] - rightptrij[temp-d*3])
                            + fabs(leftptrij[temp+1] - rightptrij[temp-d*3+1])
                            + fabs(leftptrij[temp+2] - rightptrij[temp-d*3+2]);
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
                st = st + fabs(leftptr[temp]-rightptr[temp-d*3])
                        + fabs(leftptr[temp+1]-rightptr[temp-d*3+1])
                        + fabs(leftptr[temp+2]-rightptr[temp-d*3+2]);
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
            for(int d = 0;d<maxdis&&d<=j;d++){
                double st;
                st = fabs(leftptr[temp1]-rightptr[temp1-d*3])
                        +fabs(leftptr[temp1+1]-rightptr[temp1-d*3+1])
                        +fabs(leftptr[temp1+2]-rightptr[temp1-d*3+2]);
                st = st - fabs(leftptr[temp2]-rightptr[temp2-3*d])
                        - fabs(leftptr[temp2+1]-rightptr[temp2-3*d+1])
                        - fabs(leftptr[temp2+2]-rightptr[temp2-3*d+2]);
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
                for(int d = 0;d<maxdis&&d<=j;d++){
                    double st;
                    st = fabs(leftptr[temp1]-rightptr[temp1-d*3])
                            +fabs(leftptr[temp1+1]-rightptr[temp1-d*3+1])
                            +fabs(leftptr[temp1+2]-rightptr[temp1-d*3+2]);
                    st = st - fabs(leftptr[temp2]-rightptr[temp2-3*d])
                            - fabs(leftptr[temp2+1]-rightptr[temp2-3*d+1])
                            - fabs(leftptr[temp2+2]-rightptr[temp2-3*d+2]);
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

        //real dealing
        for(int j = border;j+border<size.width;j++){
            unsigned char *leftptrij = leftptri+j*3;
            unsigned char *rightptrij = rightptri+j*3;
            double mmin=1<<29;
            int mindex;
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
                //if(sum/sumw<mmin)
                  //  mmin=sum/sumw,mindex = d;
                cost[j*size.width+j-d] = sum/sumw;
            }
            //disptr[i*size.width+j] = mindex;
        }

        //DP
        for(int a = 0;a<size.width;a++){
            double *DPa = DP+a*DPwidth;
            double *DPa_1 = DPa - DPwidth;
            char *patha = path+a*size.width;
            for(int b = 0;b<size.width;b++){
                double A;
                if(a<border||b<border||a-b>=maxdis||a-b<0||a+border>=size.width)
                    A = 1<<29;
//                    A = DPa_1[b-1]
//                                + fabs(leftptri[a*3]-rightptri[b*3])
//                                + fabs(leftptri[a*3+1]-rightptri[b*3+1])
//                                + fabs(leftptri[a*3+2]-rightptri[b*3+2]);
                else {
                    A = DPa_1[b-1]+cost[a*size.width+b];
                }

                double B = DPa_1[b] + P;
                double C = DPa[b-1] + P;
                if(A<B){
                    if(A<C){//A is min
                        DPa[b] = A;
                        patha[b] = 1;
                    }else{//C is min
                        DPa[b] = C;
                        patha[b] = 3;
                    }
                }else{
                    if(B<C){//B is min
                        DPa[b] = B;
                        patha[b] = 2;
                    }else{//C is min
                        DPa[b] = C;
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
                disptri[indexa] = indexa-indexb;
                indexa--;
                indexb--;
            }else if(path[indexa*size.width+indexb]==2){
                indexa--;
            }else if(path[indexa*size.width+indexb]==3){
                indexb--;
            }
        }
        qDebug()<<"finished "<<i*100/size.height<<"%\r";
    }
    delete []w1buff;
    delete []w2buff;
    delete []sad_;
    delete []s;
    delete []lefts;
    delete []leftsum;
    delete []rights;
    delete []rightsum;
    delete []DP_;
    delete []path;
    delete []cost;
}


//单个方向扫描
void stereo_SGM(Mat &left,Mat &right,Mat &dis,int maxdis,int dir,double P1,double P2){
    if(left.size()!=right.size())
        return;

    Size size = left.size();
    dis.create(size,CV_32F);

    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;
    float *disptr = (float*)dis.data;

    //多两个个作为边界
    int Ewidth = size.width+2;
    int Emax = maxdis+2;//当d=-1和d=maxdis时的边界
    double *E_ = new double[Ewidth*Emax*7];//其实只有3个方向需要，但是为了程序统一，4个方向都存起来
    double *minE_ = new double[Ewidth*4];//所有d的最小值
    double *E0 = E_ + 1*Emax;
    double *E11 = E_ + Ewidth*Emax + 1*Emax;;
    double *E12 = E_ + 2*Ewidth*Emax + 1*Emax;;
    double *E21 = E_ + 3*Ewidth*Emax + 1*Emax;;
    double *E22 = E_ + 4*Ewidth*Emax + 1*Emax;;
    double *E31 = E_ + 5*Ewidth*Emax + 1*Emax;;
    double *E32 = E_ + 6*Ewidth*Emax + 1*Emax;;

    double *minE0 = minE_ + 1;
    double *minE1 = minE_ +   Ewidth+1;
    double *minE2 = minE_ + 2*Ewidth+1;
    double *minE3 = minE_ + 3*Ewidth+1;

    //initial and borders
    for(int i=-1;i<=size.width;i++){
        for(int d = 0;d<maxdis;d++){
            E0[i*Emax+d+1] = 0;
            E11[i*Emax+d+1] = 0;
            E12[i*Emax+d+1] = 0;
            E21[i*Emax+d+1] = 0;
            E22[i*Emax+d+1] = 0;
            E31[i*Emax+d+1] = 0;
            E32[i*Emax+d+1] = 0;
        }
        int d = -1;
        E0[i*Emax+d+1] = 1<<29;
        E11[i*Emax+d+1] = 1<<29;
        E12[i*Emax+d+1] = 1<<29;
        E21[i*Emax+d+1] = 1<<29;
        E22[i*Emax+d+1] = 1<<29;
        E31[i*Emax+d+1] = 1<<29;
        E32[i*Emax+d+1] = 1<<29;
        d = maxdis;
        E0[i*Emax+d+1] = 1<<29;
        E11[i*Emax+d+1] = 1<<29;
        E12[i*Emax+d+1] = 1<<29;
        E21[i*Emax+d+1] = 1<<29;
        E22[i*Emax+d+1] = 1<<29;
        E31[i*Emax+d+1] = 1<<29;
        E32[i*Emax+d+1] = 1<<29;
    }
    for(int i = -1;i<=size.width;i++){
        minE0[i] = 0;
        minE1[i] = 0;
        minE2[i] = 0;
        minE3[i] = 0;
    }

    //四个方向
    //
    //1 2  3
    // ↖↑↗
    //0←
    //
    int dx[4]={-1,-1,0,1};

    //0号方向
    //E(i,j,d) = Cost(i,j,d) + min(A,B,C)
    //A = E(i,j-1,d)
    //B = min(E(i,j-1,d-1),E(i,j-1,d+1))+P1
    //C = minE(i,j-1)+P2
    for(int i = 0;i<size.height;i++){
        unsigned char *leftptri = leftptr+i*size.width*3;
        unsigned char *rightptri = rightptr+i*size.width*3;
        for(int j = 0;j<size.width;j++){
            unsigned char *leftptrij = leftptri+j*3;
            unsigned char *rightptrij = rightptri+j*3;
            double *E0j_1 = E0+(j-1)*Emax;
            double minc0 = 1<<29;
            double *E1j_1 = E11+(j-1)*Emax;
            double minc1 = 1<<29;
            double *E2j = E21+j*Emax;
            double minc2 = 1<<29;
            double *E3j_1 = E31+(j+1)*Emax;
            double minc3 = 1<<29;
            int mind;
            for(int d = 0;d<maxdis&&d<=j;d++){
                unsigned char *rightptrijd = rightptrij-3*d;
                double Cost = fabs(leftptrij[0]  - rightptrijd[0])
                        + fabs(leftptrij[1]  - rightptrijd[1])
                        + fabs(leftptrij[2]  - rightptrijd[2]);
                if(dir==0){
                    double *Et0 = E0 + j*Emax+d+1;
                    {
                    double A = E0j_1[d+1];
                    double B1 = E0j_1[d];
                    double B2 = E0j_1[d+2];
                    double C = minE0[j-1]+P2;
                    *Et0 = Cost + min(min(A,C),min(B1,B2)+P1);
                    if(*Et0<minc0)
                        minc0 = *Et0,mind=d;
                    }
                }
                else if(dir==1){
                    double *Et1 = E12 + j*Emax+d+1;
                    {
                    double A = E1j_1[d+1];
                    double B1 = E1j_1[d];
                    double B2 = E1j_1[d+2];
                    double C = minE1[j-1]+P2;
                    *Et1 = Cost + min(min(A,C),min(B1,B2)+P1);
                    if(*Et1<minc1)
                        minc1 = *Et1,mind=d;
                    }
                }
                else if(dir==2){
                    double *Et2 = E22 + j*Emax+d+1;
                    {
                    double A = E2j[d+1];
                    double B1 = E2j[d];
                    double B2 = E2j[d+2];
                    double C = minE2[j]+P2;
                    *Et2 = Cost + min(min(A,C),min(B1,B2)+P1);
                    if(*Et2<minc2)
                        minc2 = *Et2,mind=d;
                    }
                }
                else if(dir==3){
                    double *Et3 = E32 + j*Emax+d+1;
                    {
                    double A = E3j_1[d+1];
                    double B1 = E3j_1[d];
                    double B2 = E3j_1[d+2];
                    double C = minE3[j+1]+P2;
                    *Et3 = Cost + min(min(A,C),min(B1,B2)+P1);
                    if(*Et3<minc3)
                        minc3 = *Et3,mind=d;
                    }
                }
            }
            minE0[j] = minc0;
            minE1[j] = minc1;
            minE2[j] = minc2;
            minE3[j] = minc3;
            disptr[i*size.width+j] = mind;
        }
        double *temp = E11;
        E11 = E12;
        E12 = temp;
        temp = E21;
        E21 = E22;
        E22 = temp;
        temp = E31;
        E31 = E32;
        E32 = temp;
    }
    delete []E_;
    delete []minE_;
}

void stereo_Itera(Size size,int *cost,Mat &dis,int maxdis,int P1,int P2,int iter){
    int *disint = new int[size.width*size.height];
    int *disint2 = new int[size.width*size.height];
    if(dis.size().height<=0)
        return;
    float *disptr = (float*)dis.data;
    //initalborder
    for(int i = 0;i<size.height;i++)
        for(int j = 0;j<size.width;j++)
            disint[i*size.width+j] = disptr[i*size.width+j];

    if(1){
    int T = -1;
    int Tmax = iter;
    while(++T<Tmax){
        for(int i = 1;i+1<size.height;i++){
            int *disinti = disint+i*size.width;
            int *disinti_p = disinti-size.width;
            int *disinti_n = disinti+size.width;
            int *disint2i = disint2+i*size.width;
            int *costi = cost + i*size.width*maxdis;
            for(int j = 1;j<size.width;j++){
                int mmin=1<<29;
                int mmindex;
                int *costij = costi+j*maxdis;
                //八邻域
                int near[8]={disinti_p[j-1],disinti_p[j],disinti_p[j+1],
                             disinti[j-1],disinti[j+1],
                             disinti_n[j-1],disinti_n[j],disinti_n[j+1]
                            };
                for(int d = 0;d<maxdis&&d<=j;d++){
                    //int cost = (leftptri[j] - rightptri[j-d])*(leftptri[j] - rightptri[j-d]);
                    int co = costij[d];
                    for(int ni = 0;ni<8;ni++){
                        if(abs(d-near[ni])==1){
                            co += P1;
                        }else if(abs(d-near[ni])>1){
                            co += P2;
                        }
                    }
                    if(co<mmin){
                        mmin = co;
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

    for(int i = 0;i<size.height;i++){
        for(int j = 0;j<size.width;j++){
            disptr[i*size.width+j] = disint[i*size.width+j];
        }
    }
    delete []disint;
    delete []disint2;

}

void stereo_MSGM(Mat &left,Mat &right,Mat &dis,int maxdis,int P1,int P2){
    if(left.size()!=right.size())
        return;
    Size size = left.size();

    dis.create(size,CV_32F);
    float *disptr = (float *)dis.data;

    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;

    //-1,size.width+1
    int LrWidth = size.width+2;
    int LrMax  = maxdis+2;

    int *cost = new int[size.width*size.height*maxdis];
    if(!cost){
        return ;
    }


    //get cost first
    //other cost function should be here
    for(int i = 0;i<size.height;i++){
        unsigned char *leftptri = leftptr+i*size.width*3;
        unsigned char *rightptri = rightptr+i*size.width*3;
        int *costi = cost+i*size.width*maxdis;
        for(int j = 0;j<size.width;j++){
            unsigned char *leftptrij = leftptri+j*3;
            unsigned char *rightptrij = rightptri+j*3;
            int *costij = costi+j*maxdis;
            for(int d = 0;d<maxdis&&d<=j;d++){
                unsigned char *rightptrijd = rightptrij - 3*d;
                /*costij[d] = (leftptrij[0]-rightptrijd[0])*(leftptrij[0]-rightptrijd[0])
                            + (leftptrij[1]-rightptrijd[1])*(leftptrij[1]-rightptrijd[1])
                            + (leftptrij[2]-rightptrijd[2])*(leftptrij[2]-rightptrijd[2]);
                            */


                costij[d] = abs(leftptrij[0]-rightptrijd[0])
                            + abs(leftptrij[1]-rightptrijd[1])
                            + abs(leftptrij[2]-rightptrijd[2]);

//                costij[d] = abs(leftptrij[0]-rightptrijd[0]+leftptrij[1]-rightptrijd[1]+leftptrij[2]-rightptrijd[2]);
            }
        }
    }


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
                        +min(min(Lr3j_1[d+1],minLr3[j+1]+P2),min(Lr3j_1[d],Lr3j_1[d+2])+P1) - minLr3[j+1];
                *minLr31j = min(*minLr31j,Lr31j[d+1]);

//                if(*minLr0j+*minLr11j+*minLr21j+*minLr31j<minc)
//                    minc = *minLr0j+*minLr11j+*minLr21j+*minLr31j,mind = d;
                LrSumij[d] += 0
                        +Lr0j[d+1]
                        +Lr11j[d+1]
                        +Lr21j[d+1]
                        +Lr31j[d+1];
                if(LrSumij[d]<minc)
                    minc = LrSumij[d],mind = d;
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

    stereo_Itera(size,cost,dis,maxdis,P1,P2,20);
    delete []cost;
    delete []LrSum;
    delete []LrBuff_;
    delete []minLr_;
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

void stereo_BMBox_Cost(Mat &left,Mat &right,int *costout,int maxdis,int winsize){
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
        int *costoutj = costout+j*maxdis;
        for(int i = 0;i<size.height;i++){
            int mind;
            int minsad=1<<29;
            int tem = (i+winsize)*maxdis;
            int tem2 = (i-1-winsize)*maxdis;
            for(int d = 0;d<maxdis&&d<=j;d++){
                sad[d] = sad[d] + s0[tem+d] - s0[tem2+d];
                costoutj[d] = sad[d];
            }
            costoutj += size.width*maxdis;
        }

    }
    delete []sad;
    qDebug()<<"释放完sad"<<endl;
    delete []s0_;
    qDebug()<<"释放完s0"<<endl;
    delete []s1_;
    qDebug()<<"释放完s1"<<endl;

}



//换成Lab空间
//L = L*100/255
//a = a-128
//b = b-128
void stereo_BM_AW_Lab_Cost(Mat &left,Mat &right,int *costout,int maxdis,int winsize){
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


    //double *mincost = new double[size.width];
    //double *
    double *w1buff = new double[(2*winsize+1)*(2*winsize+1)*size.width];
    double *w2buff = new double[(2*winsize+1)*(2*winsize+1)*size.width];

    for(int k = 0;k<4;k++){
        int mini_[4]={0,size.height-winsize-1,0,0};
        int maxi_[4]={winsize,size.height-1,size.height,size.height};
        int minj_[4]={0,0,0,size.width-1-winsize};
        int maxj_[4]={size.width,size.width,winsize,size.width};
        for(int i = mini_[k];i<maxi_[k];i++){
            int *costi = costout+i*size.width*maxdis;
            unsigned char *leftptri = leftptr+i*size.width*3;
            unsigned char *rightptri = rightptr+i*size.width*3;
            for(int j = minj_[k];j<maxj_[k];j++){
                int *costij = costi+j*maxdis;
                unsigned char *leftptrij = leftptri+j*3;
                unsigned char *rightptrij = rightptri+j*3;
                for(int d = 0;d<maxdis&&d<=j;d++){
                    unsigned char *rightptrijd = rightptrij-3*d;
                    costij[d] = abs(leftptrij[0]-rightptrijd[0])
                                +abs(leftptrij[1]-rightptrijd[1])
                                +abs(leftptrij[2]-rightptrijd[2]);
                    costij[d]*=100;
                }
            }
        }
    }


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
        int *costi = costout+i*size.width*maxdis;
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
                        double e1 = fabs(leftptr[q]-rightptr[q_])+
                                fabs(leftptr[q+1]-rightptr[q_+1])+
                                fabs(leftptr[q+2]-rightptr[q_+2]);

                        double w1 = w1buff[w1index++];
                        double w2 = w2buff[w2index++];
                        sum += w1*w2*e1;
                        sumw += w1*w2;
                    }
                }
                costij[d] = sum*100.0/sumw;
            }

        }
        qDebug()<<"finished "<<i*100/size.height<<"%\r";
    }
    delete []w1buff;
    delete []w2buff;
}


void stereo_MSGM_FW_AW(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize,int P1,int P2,int method,int iter){
    if(left.size()!=right.size())
        return;
    Size size = left.size();

    dis.create(size,CV_32F);
    float *disptr = (float *)dis.data;

    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;

    //-1,size.width+1
    int LrWidth = size.width+2;
    int LrMax  = maxdis+2;

    int *cost = new int[size.width*size.height*maxdis];
    if(!cost){
        return ;
    }


    if(winsize==0){
        //get cost first
        //other cost function should be here
        for(int i = 0;i<size.height;i++){
            unsigned char *leftptri = leftptr+i*size.width*3;
            unsigned char *rightptri = rightptr+i*size.width*3;
            int *costi = cost+i*size.width*maxdis;
            for(int j = 0;j<size.width;j++){
                unsigned char *leftptrij = leftptri+j*3;
                unsigned char *rightptrij = rightptri+j*3;
                int *costij = costi+j*maxdis;
                for(int d = 0;d<maxdis&&d<=j;d++){
                    unsigned char *rightptrijd = rightptrij - 3*d;
                    costij[d] = abs(leftptrij[0]-rightptrijd[0])
                                + abs(leftptrij[1]-rightptrijd[1])
                                + abs(leftptrij[2]-rightptrijd[2]);
                }
            }
        }
    }
    else {
        if(method==0)stereo_BMBox_Cost(left,right,cost,maxdis,winsize);
        else if(method==1)stereo_BM_AW_Lab_Cost(left,right,cost,maxdis,winsize);
    }


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
                        +min(min(Lr3j_1[d+1],minLr3[j+1]+P2),min(Lr3j_1[d],Lr3j_1[d+2])+P1) - minLr3[j+1];
                *minLr31j = min(*minLr31j,Lr31j[d+1]);

//                if(*minLr0j+*minLr11j+*minLr21j+*minLr31j<minc)
//                    minc = *minLr0j+*minLr11j+*minLr21j+*minLr31j,mind = d;
                LrSumij[d] += 0
                        +Lr0j[d+1]
                        +Lr11j[d+1]
                        +Lr21j[d+1]
                        +Lr31j[d+1]
                        ;
                if(LrSumij[d]<minc)
                    minc = LrSumij[d],mind = d;
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


    if(iter)
        stereo_Itera(size,cost,dis,maxdis,P1,P2,iter);

    delete []cost;
    delete []LrSum;
    delete []LrBuff_;
    delete []minLr_;
}


void testAll(){
    QString leftfilename = "D:/works/qtwork5_5/build-3dreconstruction-Desktop_Qt_5_1_0_MinGW_32bit-Debug/images/opencv/tsukubaL.bmp";
    QString rightfilename = "D:/works/qtwork5_5/build-3dreconstruction-Desktop_Qt_5_1_0_MinGW_32bit-Debug/images/opencv/tsukubaR.bmp";

    /*QString leftfilename = QFileDialog::getOpenFileName(
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
        */
            Mat leftmat = imread(leftfilename.toUtf8().data());
            Mat rightmat = imread(rightfilename.toUtf8().data());

            Mat dis,vdisp;


            clock_t t = clock();
//            stereoBmProto(leftmat,rightmat,dis,8,20);
//            stereo_BMBox(leftmat,rightmat,dis,20,5);
//            stereo_BM_FBS2Pro(leftmat,rightmat,dis,20,1,3);
//            stereoDP2(leftmat,rightmat,dis,20,20);
//            stereo_BM_AW_DP(leftmat,rightmat,dis,20,7,20);
//            stereo_BM_FBS_DP(leftmat,rightmat,dis,20,1,5,20*9);
//            stereo_SGM(leftmat,rightmat,dis,20,1,10,40);
//            stereo_MSGM(leftmat,rightmat,dis,20,4,20);
            stereo_MSGM_FW_AW(leftmat,rightmat,dis,20,3,8*7*7,32*7*7,0,0);

//            stereo_BM_AW_Lab_Pro(leftmat,rightmat,dis,20,10);
            qDebug()<<"used time "<<clock()-t<<"ms"<<endl;


            dis.convertTo(vdisp,CV_8U);
            normalize(vdisp,vdisp,0,255,CV_MINMAX);

            imshow("Single Scanline ",vdisp);
//        }
//    }
}




///*
int main(int argc, char *argv[]){
    QApplication a(argc, argv);
    testAll();
    return a.exec();
}
//*/
