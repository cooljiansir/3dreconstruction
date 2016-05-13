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
//    sgbm.fullDP = false;

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
//    bm.state->preFilterCap = 1;
    bm.state->SADWindowSize = SADWindowSize;
    bm.state->minDisparity = minDisparity;
    bm.state->numberOfDisparities = numberOfDisparities;
    //bm.state->textureThreshold = 10;
    //bm.state->uniquenessRatio = 15;

    bm.state->textureThreshold = 0;
    bm.state->uniquenessRatio = 0;

//      bm.state->uniquenessRatio = 0;
    bm.state->speckleWindowSize = 100;
//      bm.state->speckleWindowSize = 1;
    bm.state->speckleRange = 32;
    bm.state->disp12MaxDiff = 1;
    bm(leftgray,rightgray,dis,CV_32F);
//    connectedArea(dis,2,100);
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
//    connectedArea(dis,2,100);
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

//    for(int i = 0;i<size.width*size.height;i++)
//        disptr[i] = disint[i];
    subpixel(cost,size,disint,maxdis,disptr);

    delete []cost;
    delete []LrSum;
    delete []LrBuff_;
    delete []minLr_;
    delete []disint;
}



void StereoMatchSGBM_DP::stereoMatch(Mat &leftmat, Mat &rightmat, Mat &dis){

    stereo_MSGM_MY(leftmat,rightmat,dis,140,8,32,5,0,64,true);
    medianBlur(dis,dis,3);

    return;

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

StereoMatchAW::StereoMatchAW(){
    this->paramCount = 0;
}
string StereoMatchAW::getKindName(){
    return "AW";
}
int StereoMatchAW::getParamCount(){
    return this->paramCount;
}
string StereoMatchAW::getParamName(int index){
    return "";
}
int StereoMatchAW::getParamMax(int index){
    return 0;
}
int StereoMatchAW::getParamMin(int index){
    return 0;
}
int StereoMatchAW::getParamValue(int index){
    return 0;
}
void StereoMatchAW::setParamValue(int index, int value){

}
void StereoMatchAW::stereoMatch(Mat &left, Mat &right, Mat &dis){
    if(left.size()!=right.size())
        return;
    double yc=7,yg=36;
    int winsize = 7;
    int maxdis = 200;

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
