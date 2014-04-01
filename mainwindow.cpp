#include "mainwindow.h"
#include <QFileDialog>
#include "ui_mainwindow.h"

#include "highgui.h"
#include "clicklabel.h"
#include "uti.h"

using namespace cv;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QString filename_left = QFileDialog::getOpenFileName(
       0,
       "Binocular Calibration - Open Left Image",
       NULL,
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if(!filename_left.isNull()){
        QString filename_right = QFileDialog::getOpenFileName(
           0,
           "Binocular Calibration - Open Left Image",
           NULL,
           "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
        if(!filename_right.isNull()){
            matLeft = imread(filename_left.toStdString().c_str());
            matRight = imread(filename_right.toStdString().c_str());
            uninMat;
            uninMat.create(matLeft.rows,matLeft.cols+matRight.cols,CV_8UC3);
            Mat uninMatL = uninMat(Rect(0,0,matLeft.cols,matLeft.rows));
            Mat uninMatR = uninMat(Rect(matLeft.cols,0,matRight.cols,matRight.rows));
            matLeft.copyTo(uninMatL);
            matRight.copyTo(uninMatR);
            ui->clickLabel->setPixmap(QPixmap::fromImage(Mat2QImage(uninMat)));
        }
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
void MainWindow::on_pressed(int x, int y){
//    int thres = 500;
//    Mat temmat;
//    this->imgmat.copyTo(temmat);
//    Vec3b v1 = this->imgmat.at<Vec3b>(y,x);
//    for(int i = 0;i<this->imgmat.rows;i++){
//        for(int j = 0;j<this->imgmat.cols;j++){
//            Vec3b *t2 = temmat.ptr<Vec3b>(i,j);
//            if(((*t2)[0]-v1[0])*((*t2)[0]-v1[0])
//                    +((*t2)[1]-v1[1])*((*t2)[1]-v1[1])
//                    +((*t2)[2]-v1[2])*((*t2)[2]-v1[2])<thres){
//                (*t2)[0] = (*t2)[1] = (*t2)[2] = 0;
//            }
//        }
//    }
//    imshow("area",temmat);
    Mat cpy;
    this->uninMat.copyTo(cpy);
    cv::line(cpy,Point(0,y),Point(cpy.cols,y),Scalar(0,0,255));
    ui->clickLabel->setPixmap(QPixmap::fromImage(Mat2QImage(cpy)));
    Mat showmat = Mat::zeros(255,cpy.cols,CV_8UC3);
    for(int i = 0;i<cpy.cols;i++){
        Vec3b *v = this->uninMat.ptr<Vec3b>(y,i);
        int va = ((*v)[0] + (*v)[1] + (*v)[2])/3;
        Vec3b *v2 = showmat.ptr<Vec3b>(va,i);
        (*v2)[0] = (*v2)[1] = (*v2)[2] = 255;

    }
    imshow("graph",showmat);
    //ui->label->setPixmap(QPixmap::fromImage(Mat2QImage(showmat)));
}
