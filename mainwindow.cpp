#include "mainwindow.h"
#include <QFileDialog>
#include "ui_mainwindow.h"

#include "highgui.h"
#include "clicklabel.h"

using namespace cv;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    QString filename = QFileDialog::getOpenFileName(
       0,
       "Binocular Calibration - Open Left Image",
       NULL,
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if(!filename.isNull()){
        this->imgmat = imread(filename.toStdString().c_str());
        ui->clickLabel->setPixmap(QPixmap(filename));
    }
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::on_pressed(int x, int y){
    int thres = 500;
    Mat temmat;
    this->imgmat.copyTo(temmat);
    Vec3b v1 = this->imgmat.at<Vec3b>(y,x);
    for(int i = 0;i<this->imgmat.rows;i++){
        for(int j = 0;j<this->imgmat.cols;j++){
            Vec3b *t2 = temmat.ptr<Vec3b>(i,j);
            if(((*t2)[0]-v1[0])*((*t2)[0]-v1[0])
                    +((*t2)[1]-v1[1])*((*t2)[1]-v1[1])
                    +((*t2)[2]-v1[2])*((*t2)[2]-v1[2])<thres){
                (*t2)[0] = (*t2)[1] = (*t2)[2] = 0;
            }
        }
    }
    imshow("area",temmat);
}
