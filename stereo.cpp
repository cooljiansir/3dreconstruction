#include "stereo.h"

Stereo::Stereo()
{
    //初始化默认值
    method = METHOD_CVSGBM;
    maxdis = 16*8;
    costCalulate_BT = true;
    costCalulate_SOBEL = true;
    costAggregation = COST_AGGREGATION_FW;
    winsize = 1;
    computeDisparity = COMPUTE_DISPARITY_ITER_SGM;
    P1 = 8;
    P2 = 8;
    iterTimes = 5;
    disRefineLRC = false;
    disRefineUnique = true;
    disRefineFilter = true;
    disRefineSubPixel = true;
}

void Stereo::stereoMatch(Mat &leftmat, Mat &rightmat, Mat &dismat){
    if(this->method==METHOD_CVSGBM){
        int SADWindowSize = 7;
        int numberOfDisparities = maxdis;

        StereoSGBM sgbm;
        sgbm.preFilterCap = 63;
        sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

        int cn = 1;

        sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.minDisparity = 0;
        sgbm.numberOfDisparities = numberOfDisparities;
        sgbm.uniquenessRatio = 10;
        sgbm.speckleWindowSize = 100;
        sgbm.speckleRange = 32;
        sgbm.disp12MaxDiff = 1;
        sgbm.fullDP = true;

        Mat sgbmmat;
        Mat leftmatgray,rightmatgray;
        cvtColor(leftmat,leftmatgray,CV_BGR2GRAY);
        cvtColor(rightmat,rightmatgray,CV_BGR2GRAY);

        sgbm(leftmatgray,rightmatgray,sgbmmat);
        dismat.create(leftmat.size(),CV_32F);
        Size size = leftmat.size();
        for(int i = 0;i<size.height;i++)
            for(int j = 0;j<size.width;j++)
                *dismat.ptr<float>(i,j) = (*sgbmmat.ptr<short int>(i,j))/16.0;
    }
}
