#ifndef RTFDOCUMENT_H
#define RTFDOCUMENT_H

#include <stdio.h>
#include <string>
#include <vector>
#include "tinyxml2.h"
#include <cv.h>

using namespace cv;
using namespace std;
using namespace tinyxml2;

/*
 * 业务模型
 *
 *
 */

#define SERACH_RANGE  7//7个像素搜索范围

//之前从MFC迁移过来的代码，有CPoint
struct CPoint{
    double x,y;
    CPoint(double x,double y){
        this->x = x;
        this->y = y;
    }
};

class RTFDocument
{
    //数据
private:
    bool fileopen;

    //图像坐标
    vector<vector<Point2f> > image_point;
    //实际坐标
    vector<vector<Point2f> > object_point;

    //描点所用的
    //打开的图片
    Mat selectimg;
    //监测到图片上所有角点
    vector<Point2f> harriscorners;

public:
    //开始
    bool selectBegin(string filename);
    //该点附近是否有角点
    bool havepoint(int x,int y);

    void getCornerByHand(vector<CPoint> &rec_4, vector<vector<Point2f> > &corner);

    //添加到表格数据
    void addCorner(vector<vector<Point2f> > &corner,double width);
private:

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
