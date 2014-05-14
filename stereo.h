#ifndef STEREO_H
#define STEREO_H

#include <cv.h>

using namespace cv;

class Stereo
{
public:
    Stereo();
    void stereoMatch(Mat &leftmat,Mat &rightmat,Mat &dismat);

public:
    //参数
    int method;
    int maxdis;
    bool costCalulate_BT;
    bool costCalulate_SOBEL;
    int costAggregation;
    int winsize;
    int computeDisparity;
    int P1;//DP算法的参数//Iter-SGM的参数
    int P2;//Iter-SGM的参数
    int iterTimes;//Iter-SGM的参数
    bool disRefineLRC;
    bool disRefineUnique;
    bool disRefineFilter;
    bool disRefineSubPixel;

public:
    static const int METHOD_CVBM = 1;//
    static const int METHOD_CVSGBM = 2;
    static const int METHOD_CUSTOM = 3;

    static const int COST_AGGREGATION_FW = 1;
    static const int COST_AGGREGATION_AW = 2;
    static const int COST_AGGREGATION_FBS = 3;

    static const int COMPUTE_DISPARITY_WTA = 1;
    static const int COMPUTE_DISPARITY_DP = 2;
    static const int COMPUTE_DISPARITY_ITER_SGM = 3;

private:
    void saveMat(Mat &mat);
    void readMat(Mat &mat);
public:
    void save(Mat &left,Mat &right,Mat &dis,const char path[]);
    void read(Mat &left, Mat &right, Mat &dis, const char path[]);

};

#endif // STEREO_H
