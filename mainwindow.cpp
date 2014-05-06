#include "mainwindow.h"
#include <QFileDialog>
#include "ui_mainwindow.h"

#include "highgui.h"
#include "clicklabel.h"
#include "uti.h"
#include <QDebug>

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
            cvtColor(matLeft,matLeftLab,CV_BGR2Lab);
            uninMat;
            uninMat.create(matLeft.rows,matLeft.cols+matRight.cols,CV_8UC3);
            Mat uninMatL = uninMat(Rect(0,0,matLeft.cols,matLeft.rows));
            Mat uninMatR = uninMat(Rect(matLeft.cols,0,matRight.cols,matRight.rows));
            matLeft.copyTo(uninMatL);
            matLeft.copyTo(matLeft2);
            matRight.copyTo(uninMatR);
            ui->clickLabel->setPixmap(QPixmap::fromImage(Mat2QImage(uninMat)));
            imshow("src",matLeft);
        }
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}
//AW
//显示窗口内的权值
void showWeight(Mat &imgmat,Mat &matLab,int i,int j,int winsize){
//    Mat img = imgmat.clone();
    Mat img = imgmat;
    Size size = img.size();
    if(i>=winsize&&i+winsize<size.height
            &&j>=winsize&&j+winsize<size.width){
        double yc=7,yg=36;
//        double yc=20,yg=36;

        Mat inters = img(Rect(j-winsize,i-winsize,2*winsize+1,2*winsize+1));
        Mat disp;
        unsigned char *leftptr = img.data;
        unsigned char *leftptrlab = matLab.data;
        disp.create(2*winsize+1,2*winsize+1,CV_64F);
        double *disptr = (double *)disp.data;

        int p = (i*size.width + j)*3;
        int windex = 0;
        for(int i1 = -winsize;i1<=winsize;i1++){
            for(int j1 = -winsize;j1<=winsize;j1++){
                int q = ((i+i1)*size.width + j+j1)*3;
                /*double w1 =
                        exp(-sqrt((leftptr[p]+leftptr[p+1] +leftptr[p+2]
                            - leftptr[q]-leftptr[q+1]-leftptr[q+2])*
                        (leftptr[p]+leftptr[p+1] + leftptr[p+2]
                        - leftptr[q]-leftptr[q+1]-leftptr[q+2]))/3/yc
                        -sqrt(i1*i1+j1*j1)/yg);
                */

                double w1 =
                        exp(-sqrt((leftptrlab[p] - leftptrlab[q])*(leftptrlab[p] - leftptrlab[q])*100/255*100/255+
                                  (leftptrlab[p+1] - leftptrlab[q+1])*(leftptrlab[p+1] - leftptrlab[q+1])+
                                  (leftptrlab[p+2] - leftptrlab[q+2])*(leftptrlab[p+2] - leftptrlab[q+2]))/yc
                        -sqrt(i1*i1+j1*j1)/yg);

                qDebug()<<"w: "<<w1<<endl;
                disptr[windex++] = w1;
            }
        }
        Mat vdisp;
        normalize(disp,disp,0,255,CV_MINMAX);
        disp.convertTo(vdisp,CV_8U);
        for(int i = 0;i<2*winsize+1;i++){
            for(int j = 0;j<2*winsize+1;j++){
                unsigned char *p0 = inters.ptr<unsigned char>(i,j);
                p0[0] = p0[1] = p0[2] = vdisp.at<unsigned char>(i,j);

            }
        }
        imshow("AW ",img);
//        imshow("Weight ",vdisp);
        //imshow("original ",inters);
    }
}

//FBS
//显示窗口内的权值
void showWeight2(Mat &imgmat,Mat &matLab,int i,int j,int winsize,int bigwinsize){
//    Mat img = imgmat.clone();
    Mat img = imgmat;
    Size size = img.size();
    int border = bigwinsize*(2*winsize+1)+winsize;
    if(i>=border&&i+border<size.height
            &&j>=border&&j+border<size.width){
        double yc=7,yg=36;
//        double yc=20,yg=36;


        Mat disp;
        unsigned char *leftptr = img.data;
        disp.create((2*winsize+1)*(2*bigwinsize+1),(2*winsize+1)*(2*bigwinsize+1),CV_64F);
        Size dissize = disp.size();

        Mat inters = img(Rect(j-border,i-border,dissize.width,dissize.height));

        double *disptr = (double *)disp.data;


        for(int i1=-bigwinsize;i1<=bigwinsize;i1++){
            for(int j1=-bigwinsize;j1<=bigwinsize;j1++){
                double sum[3]={0,0,0};
                for(int i2=-winsize;i2<=winsize;i2++){
                    for(int j2=-winsize;j2<=winsize;j2++){
                        int temp = (i+i1*(2*winsize+1)+i2)*size.width+j+j1*(2*winsize+1)+j2;
                        temp *=3;
                        sum[0] += leftptr[temp];
                        sum[1] += leftptr[temp+1];
                        sum[2] += leftptr[temp+2];
                    }
                }
                sum[0] = sum[0]/(2*winsize+1)/(2*winsize+1);
                sum[1] = sum[1]/(2*winsize+1)/(2*winsize+1);
                sum[2] = sum[2]/(2*winsize+1)/(2*winsize+1);
                double w =
                        exp(-sqrt((leftptr[i*size.width*3+j*3] - sum[0])*(leftptr[i*size.width*3+j*3] - sum[0])+
                                  (leftptr[i*size.width*3+j*3+1] - sum[1])*(leftptr[i*size.width*3+j*3+1] - sum[1])+
                                  (leftptr[i*size.width*3+j*3+2] - sum[2])*(leftptr[i*size.width*3+j*3+2] - sum[2]))/yc
                        -sqrt(i1*i1+j1*j1)*(2*winsize+1)/yg);
                //qDebug()<<"w "<<w<<endl;
                for(int i2=-winsize;i2<=winsize;i2++){
                    for(int j2=-winsize;j2<=winsize;j2++){
                        disptr[(border+i1*(2*winsize+1)+i2)*dissize.width+border+j1*(2*winsize+1)+j2] = w;
                    }
                }
            }
        }

        Mat vdisp;
        //disp.convertTo(vdisp,CV_8U);
        normalize(disp,disp,0,255,CV_MINMAX);
        disp.convertTo(vdisp,CV_8U);
        for(int i = 0;i<dissize.height;i++){
            for(int j = 0;j<dissize.width;j++){
                unsigned char *p0 = inters.ptr<unsigned char>(i,j);
                p0[0] = p0[1] = p0[2] = vdisp.at<unsigned char>(i,j);
            }
        }
        imshow("FBS ",img);
//        imshow("Weight ",vdisp);
        //imshow("original ",inters);
    }
}

//查看权重值
void MainWindow::on_pressed(int x, int y){
    showWeight(this->matLeft,this->matLeftLab,y,x,22);
    showWeight2(this->matLeft2,this->matLeftLab,y,x,1,7);
}

/*以前用来研究一行显示的波形图的
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
*/
