#ifndef CORNER_H
#define CORNER_H

#include <QApplication>
#include "cv.h"
#include "highgui.h"
#include <QFileDialog>
#include <algorithm>
#include "MyMat.h"
#include <vector>

using namespace std;
using namespace cv;


class Corner{
public:
    Corner();

    void setPixMethod(int method);
    void setSubPixMethod(int method);

    void setThreshold(double threshold);
    double getThreshold(int medthod);

    void setK(double k);
    double getK();

    void setWinsize(int w);
    int getWinsize();

private:
    Mat img;
    double thres[4];
    int winsize;
    double k;

    int pixmethod;
    int subpixel;
public:
    const static int PIX_MORAVEC    = 0;
    const static int PIX_HARRIS     = 1;
    const static int PIX_NOBEL      = 2;
    const static int PIX_SHI_TOMASI = 3;

    const static int SUBPIX_FITTING = 0;
    const static int SUBPIX_VECTOR = 1;
public:
    void getCorner(Mat &img, vector<Point2f> &corners);
};




















#endif // CORNER_H

