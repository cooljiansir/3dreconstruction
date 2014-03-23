#ifndef STEREOMATCH_H
#define STEREOMATCH_H

#include <string>
#include "cv.h"

using namespace cv;
using namespace std;

//匹配作为3d重建的核心算法
//单独抽象出一个类来研究研究

class StereoMatch
{
public:
    virtual string getKindName()=0;//获取分类的方法名称
    virtual int getParamCount()=0;//获取算法可调参数个数
    virtual string getParamName(int index)=0;//获取可调参数名称
    virtual int getParamValue(int index)=0;//获取可调参数的值
    virtual int getParamMax(int index)=0;//获取可调参数的最大值
    virtual int getParamMin(int index)=0;//获取可调参数的最小值
    virtual void setParamValue(int index,int value)=0;//设置可调参数的值
    virtual void stereoMatch(Mat &left,Mat &right,Mat &dis)=0;//匹配函数
};

//opencv 的SBM匹配函数
class StereoMatchOpencvSBM:public StereoMatch{
public:
    StereoMatchOpencvSBM();
    string getKindName();//获取分类的方法名称
    int getParamCount();//获取算法可调参数个数
    string getParamName(int index);//获取可调参数名称
    int getParamValue(int index);//获取可调参数的值
    int getParamMax(int index);//获取可调参数的最大值
    int getParamMin(int index);//获取可调参数的最小值
    void setParamValue(int index,int value);//设置可调参数的值
    void stereoMatch(Mat &left,Mat &right,Mat &dis);//匹配函数

private:
    int paramCount;
    int param[3];//可调参数
};

//opencv的BM匹配函数
class StereoMatchOpencvBM:public StereoMatch{
public:
    StereoMatchOpencvBM();
    string getKindName();//获取分类的方法名称
    int getParamCount();//获取算法可调参数个数
    string getParamName(int index);//获取可调参数名称
    int getParamValue(int index);//获取可调参数的值
    int getParamMax(int index);//获取可调参数的最大值
    int getParamMin(int index);//获取可调参数的最小值
    void setParamValue(int index,int value);//设置可调参数的值
    void stereoMatch(Mat &left,Mat &right,Mat &dis);//匹配函数

private:
    int paramCount;
    int param[3];//可调参数
};

#endif // STEREOMATCH_H
