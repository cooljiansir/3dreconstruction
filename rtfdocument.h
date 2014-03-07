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
    vector<vector<Point2f> > image_point_l;
    //实际坐标
    vector<vector<Point3f> > object_point_l;

    //图像坐标
    vector<vector<Point2f> > image_point_r;
    //实际坐标
    vector<vector<Point3f> > object_point_r;

    //当前是哪个摄像头，0左，1右
    int isright;

    //一下数据是打开一个手动标定窗口需要的
    //描点所用的
    //打开的图片
    Mat selectimg;
    //监测到图片上所有角点
    vector<Point2f> harriscorners;

public:
    //开始
    bool selectBegin(int isr, string filename);
    //该点附近是否有角点
    bool havepoint(int x,int y);

    void getCornerByHand(vector<CPoint> &rec_4, vector<vector<Point2f> > &corner);

    //添加到表格数据
    void addCorner(vector<vector<Point2f> > selectcorner, double width);

    //获取表格数据
    void getCorner(int isright,vector<vector<Point2f> > &image_point,vector<vector<Point3f> > &object_point);

    //获取各种参数
    void calParams();
private:

    //左摄像机内参
    bool l_insok;       //是否有效
    Mat l_intrinsic;
    //左摄像机畸变参数
    bool l_disok;       //是否有效
    Mat l_distortion;
    //左摄像机内参
    bool r_insok;       //是否有效
    Mat r_intrinsic;
    //右摄像机畸变参数
    bool r_disok;       //是否有效
    Mat r_distortion;

public:
    bool getLintrisic(Mat &intrinsic);
    bool getLdistortion(Mat &distortion);
    bool getRintrinsic(Mat &intrinsic);
    bool getRdistortion(Mat &distortion);


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
