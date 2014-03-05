#ifndef RTFDOCUMENT_H
#define RTFDOCUMENT_H

#include <stdio.h>
#include <string>
#include <vector>
#include "tinyxml2.h"

using namespace std;
using namespace tinyxml2;

/*
 * 业务模型
 *
 *
 */

struct RPoint{
    double x;
    double y;
};

class RTFDocument
{
    //数据
private:
    bool fileopen;

    //图像坐标
    vector<vector<RPoint> > image_point;
    //实际坐标
    vector<vector<RPoint> > object_point;

    //左摄像机内参
    bool l_insok;       //是否有效
    double l_intrinsic[3][3];
    //左摄像机畸变参数
    bool l_disok;       //是否有效
    double l_distortion[5];
    //左摄像机内参
    bool r_insok;       //是否有效
    double r_intrinsic[3][3];
    //右摄像机畸变参数
    bool r_disok;       //是否有效
    double r_distortion[5];

    //双目标定参数
    //.....



public:
    RTFDocument();
    ~RTFDocument();
    //从文件读
    bool read(string filename);
    //是否打开文件
    bool opened();
    //保存
    bool save();
    //新建文件保存
    bool write(string filename);
};

#endif // RTFDOCUMENT_H
