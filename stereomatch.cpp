#include "stereomatch.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/contrib/contrib.hpp"
#include "cv.h"
#include <QDebug>

#define MAXS 100//最大视差

using namespace cv;

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

    sgbm(leftgray,rightgray,dis);
    dis.convertTo(dis,CV_32F);
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
    Mat leftgray,rightgray;
    leftgray.create(left.size(),CV_8UC1);
    rightgray.create(right.size(),CV_8UC1);
    cvtColor(left,leftgray,CV_BGR2GRAY);
    cvtColor(right,rightgray,CV_BGR2GRAY);

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
    var(leftgray,rightgray,dis);
}
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
