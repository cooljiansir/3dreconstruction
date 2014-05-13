#include "stereoform.h"
#include "ui_stereoform.h"
#include "uti.h"
#include <QPicture>
#include "stereo.h"
#include <highgui.h>

StereoForm::StereoForm(Mat &leftmat, Mat &rightmat, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::StereoForm)
{
    ui->setupUi(this);
    this->leftmat = leftmat.clone();
    this->rightmat = rightmat.clone();
    this->stereo.stereoMatch(leftmat,rightmat,dismat);
}

StereoForm::~StereoForm()
{
    delete ui;
}

void StereoForm::resizeEvent(QResizeEvent *){
    Mat leftshow,rightshow;
    resizeMat(leftmat,leftshow,ui->leftBorder->width(),ui->leftBorder->height());
    resizeMat(rightmat,rightshow,ui->rightBorder->width(),ui->rightBorder->height());
    ui->leftLabel->setPixmap(QPixmap::fromImage(Mat2QImage(leftshow)));
    ui->rightLabel->setPixmap(QPixmap::fromImage(Mat2QImage(rightshow)));

    if(this->dismat.size().width>0){
        Mat vdisp,disshow;
        this->dismat.convertTo(vdisp,CV_8U);
        normalize(vdisp,vdisp,0,255,CV_MINMAX);
//        imshow("test",vdisp);
//        return;
        resizeMat(vdisp,disshow,ui->disBorder->width(),ui->disBorder->height());
        disshow.convertTo(disshow,CV_8UC3);
        cvtColor(disshow,disshow,CV_GRAY2BGR);
//        imshow("test",disshow);
        ui->disLabel->setPixmap(QPixmap::fromImage(Mat2QImage(disshow)));
    }
}

void StereoForm::resizeMat(Mat &src, Mat &res, int width, int height){
    Size size = src.size();
    int w=width,h = width*size.height/size.width;
    if(h>height){
        h = height;
        w = height*size.width/size.height;
    }
    cv::resize(src,res,Size(w,h));
}
