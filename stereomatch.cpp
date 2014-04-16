#include "stereomatch.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "cv.h"
#include <QDebug>

#define MAXS 100//最大视差

using namespace cv;

void connectedArea(Mat &dis,float maxDiff,int maxSize);

StereoMatchOpencvSGBM::StereoMatchOpencvSGBM(){
    this->paramCount = 3;
    this->param[0] = 11;
    this->param[1] = 6*16;
    this->param[2] = 0;
}

string StereoMatchOpencvSGBM::getKindName(){
    return "cvSGBM";
}
int StereoMatchOpencvSGBM::getParamCount(){
    return this->paramCount;
}
string StereoMatchOpencvSGBM::getParamName(int index){
    char name[3][100] = {"SADwinSize","ndisparities","mindisparity"};
    return name[index];
}
int StereoMatchOpencvSGBM::getParamValue(int index){
    return this->param[index];
}
int StereoMatchOpencvSGBM::getParamMax(int index){
    int mmax[3]={41,16*25,40};
    return mmax[index];
}
int StereoMatchOpencvSGBM::getParamMin(int index){
    int mmin[3]={3,16,0};
    return mmin[index];
}

void StereoMatchOpencvSGBM::setParamValue(int index, int value){
    if(index>=this->paramCount)return;
    if(index==0){//参数矫正
        if((value&1)==0)value+=1;//若是偶数变为奇数
    }else if(index==1){
        if(value%16!=0)
            value = (value/16)*16;
    }else if(index==2){

    }
    this->param[index] = value;
}
void StereoMatchOpencvSGBM::stereoMatch(Mat &left, Mat &right, Mat &dis){
    Mat leftgray,rightgray;
    leftgray.create(left.size(),CV_8UC1);
    rightgray.create(right.size(),CV_8UC1);
    cvtColor(left,leftgray,CV_BGR2GRAY);
    cvtColor(right,rightgray,CV_BGR2GRAY);

    int SADWindowSize = this->param[0];
    int numberOfDisparities = this->param[1];
    int minDisparity = this->param[2];

    StereoSGBM sgbm;
    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

    int cn = 1;

    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = minDisparity;
    sgbm.numberOfDisparities = numberOfDisparities;
    sgbm.uniquenessRatio = 10;
    sgbm.speckleWindowSize = 100;
    sgbm.speckleRange = 32;
    sgbm.disp12MaxDiff = 1;
    sgbm.fullDP = true;

    Mat dis1;
    Size size = leftgray.size();
    sgbm(leftgray,rightgray,dis1);
    dis.create(size,CV_32F);
    float *disptr = (float*)dis.data;
    for(int i = 0;i<size.height;i++)
        for(int j = 0;j<size.width;j++){
            *(disptr+i*size.width+j) = *dis1.ptr<short int>(i,j)/16.0;
        }
}


StereoMatchOpencvBM::StereoMatchOpencvBM(){
    this->paramCount = 3;
    this->param[0] = 11;
    this->param[1] = 6*16;
    this->param[2] = 0;
}
string StereoMatchOpencvBM::getKindName(){
    return "cvBM";
}
int StereoMatchOpencvBM::getParamCount(){
    return this->paramCount;
}
string StereoMatchOpencvBM::getParamName(int index){
    char name[3][100] = {"SADwinSize","ndisparities","mindisparity"};
    return name[index];
}
int StereoMatchOpencvBM::getParamValue(int index){
    return this->param[index];
}
int StereoMatchOpencvBM::getParamMax(int index){
    int mmax[3]={41,16*25,40};
    return mmax[index];
}
int StereoMatchOpencvBM::getParamMin(int index){
    int mmin[3]={7,16,0};
    return mmin[index];
}
void StereoMatchOpencvBM::setParamValue(int index, int value){
    if(index>=this->paramCount)return;
    if(index==0){//参数矫正
        if((value&1)==0)value+=1;//若是偶数变为奇数
    }else if(index==1){
        if(value%16!=0)
            value = (value/16)*16;
    }else if(index==2){

    }
    this->param[index] = value;
}
void StereoMatchOpencvBM::stereoMatch(Mat &left, Mat &right, Mat &dis){
    Mat leftgray,rightgray;
    leftgray.create(left.size(),CV_8UC1);
    rightgray.create(right.size(),CV_8UC1);
    cvtColor(left,leftgray,CV_BGR2GRAY);
    cvtColor(right,rightgray,CV_BGR2GRAY);

    StereoBM bm;
    int SADWindowSize = this->param[0];
    int numberOfDisparities = this->param[1];
    int minDisparity = this->param[2];

    bm.state->preFilterCap = 31;
    bm.state->SADWindowSize = SADWindowSize;
    bm.state->minDisparity = minDisparity;
    bm.state->numberOfDisparities = numberOfDisparities;
    bm.state->textureThreshold = 10;
    bm.state->uniquenessRatio = 15;
    bm.state->speckleWindowSize = 100;
    bm.state->speckleRange = 32;
    bm.state->disp12MaxDiff = 1;
    bm(leftgray,rightgray,dis,CV_32F);
}

StereoMatchOpencvVar::StereoMatchOpencvVar(){
    this->paramCount = 1;
    this->param[0] = 5*16;
    qDebug()<<"设置初始值为"<<this->param[0]<<endl;
}

string StereoMatchOpencvVar::getKindName(){
    return "cvVar";
}
int StereoMatchOpencvVar::getParamCount(){
    return this->paramCount;
}
string StereoMatchOpencvVar::getParamName(int index){
    char name[1][100] = {"ndisparities"};
    return name[index];
}
int StereoMatchOpencvVar::getParamValue(int index){
    qDebug()<<"返回param"<<index<<":"<<this->param[index]<<endl;
    return this->param[index];
}
int StereoMatchOpencvVar::getParamMax(int index){
    int mmax[1]={16*25};
    return mmax[index];
}
int StereoMatchOpencvVar::getParamMin(int index){
    int mmin[1]={16};
    return mmin[index];
}
void StereoMatchOpencvVar::setParamValue(int index, int value){
    if(index>=this->paramCount)return;
    if(index==0){
        if(value%16!=0)
            value = (value/16)*16;
    }
    this->param[index] = value;
}
void StereoMatchOpencvVar::stereoMatch(Mat &left, Mat &right, Mat &dis){
//    Mat leftgray,rightgray;
//    leftgray.create(left.size(),CV_8UC1);
//    rightgray.create(right.size(),CV_8UC1);
//    cvtColor(left,leftgray,CV_BGR2GRAY);
//    cvtColor(right,rightgray,CV_BGR2GRAY);

    int numberOfDisparities = this->param[0];

    StereoVar var;
    var.levels = 3;                                 // ignored with USE_AUTO_PARAMS
    var.pyrScale = 0.5;                             // ignored with USE_AUTO_PARAMS
    var.nIt = 25;
    var.minDisp = -numberOfDisparities;
    var.maxDisp = 0;
    var.poly_n = 3;
    var.poly_sigma = 0.0;
    var.fi = 15.0f;
    var.lambda = 0.03f;
    var.penalization = var.PENALIZATION_TICHONOV;   // ignored with USE_AUTO_PARAMS
    var.cycle = var.CYCLE_V;                        // ignored with USE_AUTO_PARAMS
    var.flags = var.USE_SMART_ID | var.USE_AUTO_PARAMS | var.USE_INITIAL_DISPARITY | var.USE_MEDIAN_FILTERING ;
    var(left,right,dis);
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 */
StereoMatchDynamic::StereoMatchDynamic(){
    //没有可调参数
    this->paramCount = 0;
}
string StereoMatchDynamic::getKindName(){
    return "dynamic";
}
int StereoMatchDynamic::getParamCount(){
    return this->paramCount;
}
string StereoMatchDynamic::getParamName(int index){
    return "";
}
int StereoMatchDynamic::getParamMax(int index){
    return 0;
}
int StereoMatchDynamic::getParamMin(int index){
    return 0;
}
int StereoMatchDynamic::getParamValue(int index){
    return 0;
}
void StereoMatchDynamic::setParamValue(int index, int value){

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
void StereoMatchDynamic::stereoMatch(Mat &left, Mat &right, Mat &dis){
    Mat disL,disR;
    dynamicPro(left,right,disL,4,20);
    dynamicProR(left,right,disR,4,20);

    constant(disL,disR,dis);
}

StereoMatchBM::StereoMatchBM(){
    this->paramCount = 2;
    this->param[0] = 3;
    this->param[1] = 6*16;
}
string StereoMatchBM::getKindName(){
    return "BM";
}
int StereoMatchBM::getParamCount(){
    return this->paramCount;
}
string StereoMatchBM::getParamName(int index){
    char name[2][100] = {"SADwinSize","ndisparities"};
    return name[index];
}
int StereoMatchBM::getParamValue(int index){
    if(index<this->paramCount)
        return this->param[index];
    return 0;
}
int StereoMatchBM::getParamMax(int index){
    int mmax[2]={41,16*25};
    return mmax[index];
}

int StereoMatchBM::getParamMin(int index){
    int mmin[2] = {3,16};
    return mmin[index];
}
void StereoMatchBM::setParamValue(int index, int value){
    if(index>=this->paramCount)
        return;
    if(index==0){
        if((value&1)==0)value+=1;//若是偶数变为奇数
    }else if(index==1){
        if(value%16!=0)
            value = (value/16)*16;
    }
    this->param[index] = value;
}

void swap_d(double *&a,double *&b){
    double *t = a;
    a = b;
    b = t;
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

///*
void StereoMatchBM::stereoMatch(Mat &left, Mat &right, Mat &dis){
    stereoSemi4(left,right,dis,2,8*5*5,32*5*5,128);
    connectedArea(dis,32,100);
    return;
    if(left.size()!=right.size())
        return;
    int winsize = this->param[0]/2;
    int maxdis = this->param[1];
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
    connectedArea(dis,32,100);
    qDebug()<<"use time 3 "<<clock() - t1<<"ms"<<endl;
    t1 = clock();
    delete []aline;
    delete []tempcost;
    delete []tempcost2;
}
//*/
void connectedArea(Mat &dis,float maxDiff,int maxSize){
    Size size = dis.size();
    float *disptr = (float*)dis.data;

    int *stackx = new int[size.height*size.width];
    int *stacky = new int[size.height*size.width];
    int *label  = new int[size.height*size.width];
    int *labelArea =  new int[size.height*size.width];

    int labelc = 1;
    int stackindex;

    for(int i = 0;i<size.height;i++){
        int *label_ = label+i*size.width;
        int *labelArea_ = labelArea+i*size.width;
        for(int j = 0;j<size.width;j++){
            label_[j] = labelArea_[j] = -1;
        }
    }

    for(int i = 0;i<size.height;i++){
        float *disptr_ = disptr + i*size.width;
        int *label_ = label + i*size.width;
        for(int j = 0;j<size.width;j++){
            if(disptr_[j]>=0){
                if(label_[j]!=-1){
                    if(labelArea[label_[j]]!=-1&&labelArea[label_[j]]<maxSize)
                        disptr_[j] = -1;
                }else{
                    stackindex = 1;
                    stackx[0] = j;
                    stacky[0] = i;
                    int counta = 0;
                    while(stackindex>0){
                        stackindex--;
                        counta++;
                        int x = stackx[stackindex];
                        int y = stacky[stackindex];
                        label[y*size.width+x] = labelc;
                        int dx[]={-1,1,0,0};
                        int dy[]={0,0,-1,1};
                        for(int k =0;k<4;k++){
                            int x_ = x+dx[k];
                            int y_ = y+dy[k];
                            int tempd = y_*size.width+x_;
                            if(x_>=0&&x_<size.width&&y_>=0&&y_<size.height
                                    &&label[tempd]==-1
                                    &&disptr[tempd]>=0
                                    &&fabs(disptr[tempd]-disptr[y*size.width+x])<=maxDiff){
                                stackx[stackindex] = x_;
                                stacky[stackindex] = y_;
                                stackindex++;
                            }
                        }
                    }
                    labelArea[labelc++] = counta;
                    if(counta<maxSize){
                        disptr_[j] =  -1;
                    }
                }
            }
        }
    }
    delete []stackx;
    delete []stacky;
    delete []label;
    delete []labelArea;
}
/*
void StereoMatchBM::stereoMatch(Mat &left, Mat &right, Mat &dis){
    if(left.size()!=right.size())
        return;
    int winsize = this->param[0]/2;
    int maxdis = this->param[1];
    Size size = left.size();
    dis.create(size,CV_32F);


    //left and right consistence needs
    int *cost2 = new int[size.width]; //mmin cost of a line
    int *disint = new int[size.width];       //dis of a line integer
    int *disint2 = new int[size.width];      //dis2 of a line integer

    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;
    float *disptr = (float*)dis.data;

    double *aline = new double[size.width*maxdis];
    double *tempcost = new double[maxdis];//left disparity

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
            disptr[i*size.width+    j] = -1;
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

        for(int j = winsize;j+winsize<size.width;j++)
            cost2[j] = -1;

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

            disint[j] = mmind;
            if(cost2[j-mmind]==-1||tempcost[mmind]<cost2[j-mmind]){
                disint2[j-mmind] = mmind;
                cost2[j-mmind] = tempcost[mmind];
            }

            //interpolation
            //见 “三维重建中立体匹配算法的研究” P21
            if(mmind>0&&mmind<maxdis-1&&mmind+winsize<j){
                double tem = 2*tempcost[mmind - 1]  + 2*tempcost[mmind+1] - 4*tempcost[mmind];
                if(tem>0.001)te = te +(tempcost[mmind - 1]  - tempcost[mmind+1])/tem;
            }
            disptr[i*size.width+j] = te;
        }

        //left and right consistence
        for(int j = winsize;j+winsize<size.width;j++){
            int d = disint[j];
            if(fabs(disint2[j-d]-d)>1)
                disptr[i*size.width+j] = -1;
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
    connectedArea(dis,32,200);
    qDebug()<<"use time 3 "<<clock() - t1<<"ms"<<endl;
    t1 = clock();

    delete []aline;
    delete []tempcost;
    delete []cost2;
    delete []disint;
    delete []disint2;
}
*/

StereoMatchSGBM_DP::StereoMatchSGBM_DP(){
    this->paramCount = 5;
    this->param[0] = 11;
    this->param[1] = 6*16;
    this->param[2] = 0;
    this->param[3] = 5;
    this->param[4] = 20;
}

string StereoMatchSGBM_DP::getKindName(){
    return "SGBM_DP";
}
int StereoMatchSGBM_DP::getParamCount(){
    return this->paramCount;
}
string StereoMatchSGBM_DP::getParamName(int index){
    char name[5][100] = {"SADwinSize","ndisparities","mindisparity","q","c0"};
    return name[index];
}
int StereoMatchSGBM_DP::getParamMax(int index){
    int mmax[5]={41,16*25,40,10,50};
    return mmax[index];
}
int StereoMatchSGBM_DP::getParamMin(int index){
    int mmin[5]={3,16,0,1,1};
    return mmin[index];
}
int StereoMatchSGBM_DP::getParamValue(int index){
    return this->param[index];
}
void StereoMatchSGBM_DP::setParamValue(int index, int value){
    if(index>=this->paramCount)return;
    if(index==0){//参数矫正
        if((value&1)==0)value+=1;//若是偶数变为奇数
    }else if(index==1){
        if(value%16!=0)
            value = (value/16)*16;
    }else if(index==2){

    }
    this->param[index] = value;
}
void StereoMatchSGBM_DP::stereoMatch(Mat &leftmat, Mat &rightmat, Mat &dis){

    if(leftmat.size()!=rightmat.size())
        return ;
    double q = this->param[3];
    double c0 = this->param[4];

    Size size = leftmat.size();
    //use sgbm algorithm for inital
    Mat leftgray,rightgray;
    leftgray.create(leftmat.size(),CV_8UC1);
    rightgray.create(rightmat.size(),CV_8UC1);
    cvtColor(leftmat,leftgray,CV_BGR2GRAY);
    cvtColor(rightmat,rightgray,CV_BGR2GRAY);

    int SADWindowSize = this->param[0];
    int numberOfDisparities = this->param[1];
    int minDisparity = this->param[2];
    StereoSGBM sgbm;
    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

    int cn = 1;//leftgray.channels();

    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = minDisparity;
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
    connectedArea(dis,32,200);
    delete []penalize;
    delete []steps;
}
