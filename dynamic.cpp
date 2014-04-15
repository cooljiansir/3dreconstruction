#include <QApplication>
#include "cv.h"
#include "highgui.h"
#include <QFileDialog>
#include <algorithm>

#include <QDebug>

using namespace cv;
using namespace std;

void stereo_semi(Mat &left,Mat &right,Mat &dis){
    if(left.size()!=right.size())
        return 0;
    Size size = left.size();
    dis.create(size,CV_32F);


}



void test_semi(){
    QString leftfilename = QFileDialog::getOpenFileName(
       0,
       "Binocular Calibration - Open Left Image",
       NULL,
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!leftfilename.isNull()) { //用户选择了左图文件
        QString rightfilename = QFileDialog::getOpenFileName(
           0,
           "Binocular Calibration - Open Right Image",
           NULL,
           "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
        if (!rightfilename.isNull()) { //用户选择了左图文件
            Mat leftmat = imread(leftfilename.toUtf8().data());
            Mat rightmat = imread(rightfilename.toUtf8().data());

            normalize(vdisp,vdisp,0,255,CV_MINMAX);
            imshow("dynamic ",vdisp);
        }
    }
}


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    test_Dynamic();
    return a.exec();
