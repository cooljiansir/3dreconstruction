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

void stereoDP2(Mat &left,Mat &right,Mat &dis,double P){
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
                double A = costa_1[b-1]
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
//            stereo_BMBox(leftmat,rightmat,dis,20,5);
//            stereo_BM_FBS2Pro(leftmat,rightmat,dis,20,1,3);
//            stereoDP2(leftmat,rightmat,dis,200);
            stereo_BM_AW_DP(leftmat,rightmat,dis,20,7,30);
//            stereo_BM_FBS_DP(leftmat,rightmat,dis,20,1,4,30*9);

//            stereo_BM_AW_Lab_Pro(leftmat,rightmat,dis,20,10);
            qDebug()<<"used time "<<clock()-t<<"ms"<<endl;


            dis.convertTo(vdisp,CV_8U);
            normalize(vdisp,vdisp,0,255,CV_MINMAX);

            imshow("DP P=200",vdisp);
        }
    }
}




///*
int main(int argc, char *argv[]){
    QApplication a(argc, argv);
    testAll();
    return a.exec();
}
//*/
