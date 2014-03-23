#include "stereomatch.h"

StereoMatchOpencvSBM::StereoMatchOpencvSBM(){
    this->paramCount = 3;
    this->param[0] = 11;
    this->param[1] = 6*16;
    this->param[2] = 0;
}

string StereoMatchOpencvSBM::getKindName(){
    return "cv_SBM";
}
int StereoMatchOpencvSBM::getParamCount(){
    return this->paramCount;
}
string StereoMatchOpencvSBM::getParamName(int index){
    char name[3][100] = {"SADwinSize","ndisparities","mindisparity"};
    return name[index];
}
int StereoMatchOpencvSBM::getParamValue(int index){
    return this->param[index];
}
int StereoMatchOpencvSBM::getParamMax(int index){
    int mmax[3]={41,16*25,40};
    return mmax[index];
}
int StereoMatchOpencvSBM::getParamMin(int index){
    int mmin[3]={3,16,0};
    return mmin[index];
}

void StereoMatchOpencvSBM::setParamValue(int index, int value){
    if(index==0){//参数矫正
        if((value&1)==0)value+=1;//若是偶数变为奇数
    }else if(index==1){
        if(value%16!=0)
            value = (value/16)*16;
    }else if(index==2){

    }
    this->param[index] = value;
}
void StereoMatchOpencvSBM::stereoMatch(Mat &left, Mat &right, Mat &dis){
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
    bm(leftgray,rightgray,dis);
}
