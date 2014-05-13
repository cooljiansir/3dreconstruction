#include "stereoform.h"
#include "ui_stereoform.h"
#include "uti.h"
#include <QPicture>
#include "stereo.h"
#include <QDebug>
#include <highgui.h>
#include <QFileDialog>

StereoForm::StereoForm(Mat &leftmat, Mat &rightmat, QWidget *parent) :
    QWidget(parent),
    ui(new Ui::StereoForm)
{
    ui->setupUi(this);
    this->leftmat = leftmat.clone();
    this->rightmat = rightmat.clone();
    this->stereo.stereoMatch(leftmat,rightmat,dismat);
    initialForm();
    connect(ui->radioAW,SIGNAL(clicked()),this,SLOT(checkForm()));
    connect(ui->radioCustom,SIGNAL(clicked()),this,SLOT(checkForm()));
    connect(ui->radioDP,SIGNAL(clicked()),this,SLOT(checkForm()));
    connect(ui->radioFBS,SIGNAL(clicked()),this,SLOT(checkForm()));
    connect(ui->radioFW,SIGNAL(clicked()),this,SLOT(checkForm()));
    connect(ui->radioIterSGM,SIGNAL(clicked()),this,SLOT(checkForm()));
    connect(ui->radioOpencvBM,SIGNAL(clicked()),this,SLOT(checkForm()));
    connect(ui->radioOpencvSGBM,SIGNAL(clicked()),this,SLOT(checkForm()));
    connect(ui->radioWTA,SIGNAL(clicked()),this,SLOT(checkForm()));
    connect(ui->checkBoxBT,SIGNAL(clicked()),this,SLOT(checkForm()));
    connect(ui->checkBoxSobel,SIGNAL(clicked()),this,SLOT(checkForm()));
    connect(ui->checkBox_Filter,SIGNAL(clicked()),this,SLOT(checkForm()));
    connect(ui->checkBox_LRC,SIGNAL(clicked()),this,SLOT(checkForm()));
    connect(ui->checkBox_Subpixel,SIGNAL(clicked()),this,SLOT(checkForm()));
    connect(ui->checkBox_Unique,SIGNAL(clicked()),this,SLOT(checkForm()));
    this->checkForm();

}

StereoForm::~StereoForm()
{
    delete ui;
}

void StereoForm::resizeEvent(QResizeEvent *){
    this->updateImage();
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

void StereoForm::initialForm(){
    ui->radioAW->setChecked(this->stereo.costAggregation==Stereo::COST_AGGREGATION_AW);
    ui->radioCustom->setChecked(this->stereo.method==Stereo::METHOD_CUSTOM);
    ui->radioDP->setChecked(this->stereo.computeDisparity==Stereo::COMPUTE_DISPARITY_DP);
    ui->radioFBS->setChecked(this->stereo.costAggregation==Stereo::COST_AGGREGATION_FBS);
    ui->radioFW->setChecked(this->stereo.costAggregation==Stereo::COST_AGGREGATION_FW);
    ui->radioIterSGM->setChecked(this->stereo.computeDisparity==Stereo::COMPUTE_DISPARITY_ITER_SGM);
    ui->radioOpencvBM->setChecked(this->stereo.method==Stereo::METHOD_CVBM);
    ui->radioOpencvSGBM->setChecked(this->stereo.method==Stereo::METHOD_CVSGBM);
    ui->radioWTA->setChecked(this->stereo.computeDisparity==Stereo::COMPUTE_DISPARITY_WTA);
    ui->checkBoxBT->setChecked(this->stereo.costCalulate_BT);
    ui->checkBoxSobel->setChecked(this->stereo.costCalulate_SOBEL);
    ui->checkBox_Filter->setChecked(this->stereo.disRefineFilter);
    ui->checkBox_LRC->setChecked(this->stereo.disRefineLRC);
    ui->checkBox_Subpixel->setChecked(this->stereo.disRefineSubPixel);
    ui->checkBox_Unique->setChecked(this->stereo.disRefineUnique);
}

void StereoForm::checkForm(){
    //deal dynamic UI
    if(ui->radioOpencvBM->isChecked()||ui->radioOpencvSGBM->isChecked()){
        ui->label_maxdis_1->setVisible(true);
        ui->label_winsize_1->setVisible(true);
        ui->lineEdit_winsize_1->setVisible(true);
        ui->lineEdit_maxdis_1->setVisible(true);
        ui->groupBox2->setVisible(false);
        ui->groupBox3->setVisible(false);
        ui->groupBox4->setVisible(false);
        ui->groupBox5->setVisible(false);
    }else{
        ui->label_maxdis_1->setVisible(false);
        ui->label_winsize_1->setVisible(false);
        ui->lineEdit_winsize_1->setVisible(false);
        ui->lineEdit_maxdis_1->setVisible(false);

        ui->groupBox2->setVisible(true);
        ui->groupBox3->setVisible(true);
        ui->groupBox4->setVisible(true);
        ui->groupBox5->setVisible(true);
    }
    if(ui->radioWTA->isChecked()){
        ui->label_P1->setVisible(false);
        ui->label_P2->setVisible(false);
        ui->label_IterTimes->setVisible(false);
        ui->lineEdit_P1->setVisible(false);
        ui->lineEdit_P2->setVisible(false);
        ui->lineEdit_IterTimes->setVisible(false);
    }else if(ui->radioDP->isChecked()){
        ui->label_P1->setVisible(true);
        ui->label_P2->setVisible(false);
        ui->label_IterTimes->setVisible(false);
        ui->lineEdit_P1->setVisible(true);
        ui->lineEdit_P2->setVisible(false);
        ui->lineEdit_IterTimes->setVisible(false);
    }else if(ui->radioIterSGM->isChecked()){
        ui->label_P1->setVisible(true);
        ui->label_P2->setVisible(true);
        ui->label_IterTimes->setVisible(true);
        ui->lineEdit_P1->setVisible(true);
        ui->lineEdit_P2->setVisible(true);
        ui->lineEdit_IterTimes->setVisible(true);
    }

    //set UI value
    ui->lineEdit_IterTimes->setText(QString::number(this->stereo.iterTimes));
    ui->lineEdit_maxdis_1->setText(QString::number(this->stereo.maxdis));
    ui->lineEdit_maxdis_2->setText(QString::number(this->stereo.maxdis));
    ui->lineEdit_P1->setText(QString::number(this->stereo.P1));
    ui->lineEdit_P2->setText(QString::number(this->stereo.P2));
    ui->lineEdit_winsize_1->setText(QString::number(this->stereo.winsize));
    ui->lineEdit_winsize_2->setText(QString::number(this->stereo.winsize));

}

void StereoForm::on_pushButton_refresh_clicked()
{
    mutex.lock();
    //set params
    if(ui->radioAW->isChecked())
        this->stereo.costAggregation=Stereo::COST_AGGREGATION_AW;
    if(ui->radioCustom->isChecked())
        this->stereo.method=Stereo::METHOD_CUSTOM;
    if(ui->radioDP->isChecked())
        this->stereo.computeDisparity=Stereo::COMPUTE_DISPARITY_DP;
    if(ui->radioFBS->isChecked())
        this->stereo.costAggregation=Stereo::COST_AGGREGATION_FBS;
    if(ui->radioFW->isChecked())
        this->stereo.costAggregation = Stereo::COST_AGGREGATION_FW;
    if(ui->radioIterSGM->isChecked())
        this->stereo.computeDisparity=Stereo::COMPUTE_DISPARITY_ITER_SGM;
    if(ui->radioOpencvBM->isChecked())
        this->stereo.method=Stereo::METHOD_CVBM;
    if(ui->radioOpencvSGBM->isChecked())
        this->stereo.method=Stereo::METHOD_CVSGBM;
    if(ui->radioWTA->isChecked())
        this->stereo.computeDisparity=Stereo::COMPUTE_DISPARITY_WTA;

    this->stereo.costCalulate_BT = ui->checkBoxBT->isChecked();
    this->stereo.costCalulate_SOBEL = ui->checkBoxSobel->isChecked();
    this->stereo.disRefineFilter = ui->checkBox_Filter->isChecked();
    this->stereo.disRefineLRC = ui->checkBox_LRC->isChecked();
    this->stereo.disRefineSubPixel = ui->checkBox_Subpixel->isChecked();
    this->stereo.disRefineUnique = ui->checkBox_Unique->isChecked();

    this->stereo.iterTimes = ui->lineEdit_IterTimes->text().toInt();
    if(ui->radioCustom->isChecked()){
        this->stereo.maxdis = ui->lineEdit_maxdis_2->text().toInt();
        this->stereo.winsize = ui->lineEdit_winsize_2->text().toInt();
    }else{
        this->stereo.maxdis = ui->lineEdit_maxdis_1->text().toInt();
        this->stereo.winsize = ui->lineEdit_winsize_1->text().toInt();
    }
    this->stereo.P1 = ui->lineEdit_P1->text().toInt();
    this->stereo.P2 = ui->lineEdit_P2->text().toInt();


    this->stereo.stereoMatch(leftmat,rightmat,dismat);
    this->updateImage();
    mutex.unlock();
}
void StereoForm::updateImage(){
    Mat leftshow,rightshow;
    resizeMat(leftmat,leftshow,ui->leftBorder->width(),ui->leftBorder->height());
    resizeMat(rightmat,rightshow,ui->rightBorder->width(),ui->rightBorder->height());
    ui->leftLabel->setPixmap(QPixmap::fromImage(Mat2QImage(leftshow)));
    ui->rightLabel->setPixmap(QPixmap::fromImage(Mat2QImage(rightshow)));

    if(this->dismat.size().width>0){
        Mat vdisp,disshow;
        this->dismat.convertTo(vdisp,CV_8U);
        normalize(vdisp,vdisp,0,255,CV_MINMAX);
        resizeMat(vdisp,disshow,ui->disBorder->width(),ui->disBorder->height());
        disshow.convertTo(disshow,CV_8UC3);
        cvtColor(disshow,disshow,CV_GRAY2BGR);
        ui->disLabel->setPixmap(QPixmap::fromImage(Mat2QImage(disshow)));
    }
}



void testMyWorld(){
//    QString leftfilename = "D:/works/qtwork5_5/build-3dreconstruction-Desktop_Qt_5_1_0_MinGW_32bit-Debug/images/opencv/tsukubaL.bmp";
//    QString rightfilename = "D:/works/qtwork5_5/build-3dreconstruction-Desktop_Qt_5_1_0_MinGW_32bit-Debug/images/opencv/tsukubaR.bmp";

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
            StereoForm *form = new StereoForm(leftmat,rightmat);
            form->showMaximized();

        }
    }
}
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    testMyWorld();

    return app.exec();
}


