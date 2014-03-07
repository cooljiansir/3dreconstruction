#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QStandardItemModel>
#include <QFileDialog>
#include <QDir>
#include <QMessageBox>
#include "sigaldialog.h"
#include "binoculardialog.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->doc = new RTFDocument();
    ui->actionSave->setEnabled(false);
    ui->actionNew->setEnabled(true);
    ui->actionOpen_file->setEnabled(true);
    ui->actionDual->setEnabled(false);
    ui->actionTake_Photoes->setEnabled(false);
    ui->actionSigalCamera->setEnabled(false);
    ui->action3D_reconstruction->setEnabled(false);
    ui->actionTake_Photoes_2->setEnabled(false);
    ui->actionPolar_Correction->setEnabled(false);
    this->ui->stackedWidget->setCurrentIndex(0);//welcome!
}

MainWindow::~MainWindow()
{
    delete ui;
    delete this->doc;
}

//单摄像机标定界面初始化
void MainWindow::setupCalibUI(){

    //显示列表
    singal_model = new QStandardItemModel();
    singal_model->setColumnCount(6);
    this->ui->tableView->setModel(singal_model);
    singal_model->setHeaderData(0,Qt::Horizontal,QString::fromLocal8Bit("Image"));
    singal_model->setHeaderData(1,Qt::Horizontal,QString::fromLocal8Bit("u"));
    singal_model->setHeaderData(2,Qt::Horizontal,QString::fromLocal8Bit("v"));
    singal_model->setHeaderData(3,Qt::Horizontal,QString::fromLocal8Bit("x"));
    singal_model->setHeaderData(4,Qt::Horizontal,QString::fromLocal8Bit("y"));
    singal_model->setHeaderData(5,Qt::Horizontal,QString::fromLocal8Bit("z"));
    singal_model_r = new QStandardItemModel();
    singal_model_r->setColumnCount(6);
    this->ui->tableView_2->setModel(singal_model_r);
    singal_model_r->setHeaderData(0,Qt::Horizontal,QString::fromLocal8Bit("Image"));
    singal_model_r->setHeaderData(1,Qt::Horizontal,QString::fromLocal8Bit("u"));
    singal_model_r->setHeaderData(2,Qt::Horizontal,QString::fromLocal8Bit("v"));
    singal_model_r->setHeaderData(3,Qt::Horizontal,QString::fromLocal8Bit("x"));
    singal_model_r->setHeaderData(4,Qt::Horizontal,QString::fromLocal8Bit("y"));
    singal_model_r->setHeaderData(5,Qt::Horizontal,QString::fromLocal8Bit("z"));
    this->loadCalidUI();
}
void MainWindow::loadCalidUI(){
    vector<vector<Point2f> > image_point;
    vector<vector<Point3f> > object_point;
    this->doc->getCorner(0,image_point,object_point);
    int count = 0;
    for(int i = 0;i<image_point.size();i++){
        for(int j = 0;j<image_point[i].size();j++){
            singal_model->setItem(count,0,new QStandardItem(QString::number(i)));
            singal_model->setItem(count,1,new QStandardItem(QString::number(image_point[i][j].x)));
            singal_model->setItem(count,2,new QStandardItem(QString::number(image_point[i][j].y)));
            singal_model->setItem(count,3,new QStandardItem(QString::number(object_point[i][j].x)));
            singal_model->setItem(count,4,new QStandardItem(QString::number(object_point[i][j].y)));
            singal_model->setItem(count,5,new QStandardItem(QString::number(object_point[i][j].z)));
            count++;
        }
    }
    this->doc->getCorner(1,image_point,object_point);
    count = 0;
    for(int i = 0;i<image_point.size();i++){
        for(int j = 0;j<image_point[i].size();j++){
            singal_model_r->setItem(count,0,new QStandardItem(QString::number(i)));
            singal_model_r->setItem(count,1,new QStandardItem(QString::number(image_point[i][j].x)));
            singal_model_r->setItem(count,2,new QStandardItem(QString::number(image_point[i][j].y)));
            singal_model_r->setItem(count,3,new QStandardItem(QString::number(object_point[i][j].x)));
            singal_model_r->setItem(count,4,new QStandardItem(QString::number(object_point[i][j].y)));
            singal_model_r->setItem(count,5,new QStandardItem(QString::number(object_point[i][j].z)));
            count++;
        }
    }
    Mat l_intrinsic;
    Mat l_distortion;
    Mat r_intrinsic;
    Mat r_distortion;
    if(this->doc->getLintrisic(l_intrinsic)){
        QString str="";
        for(int i = 0;i<l_intrinsic.rows;i++) {
            for(int j = 0;j<l_intrinsic.cols;j++){
                QString s = QString::number(l_intrinsic.at<double>(i,j));
                str += s;
                str += QString(20-s.length(),' ');
                str += '\r';
            }
            str +='\n';
        }
        this->ui->l_IntrinLabel->setText(str);
    }
    if(this->doc->getLdistortion(l_distortion)){
        QString str="";
        for(int i = 0;i<l_distortion.rows;i++) {
            for(int j = 0;j<l_distortion.cols;j++){
                QString s = QString::number(l_distortion.at<double>(i,j));
                str += s;
                str += QString(20-s.length(),' ');
                str += '\r';
            }
            str +='\n';
        }
        this->ui->l_DistorLabel->setText(str);
    }
    if(this->doc->getRintrinsic(r_intrinsic)){
        QString str="";
        for(int i = 0;i<3;i++) {
            for(int j = 0;j<3;j++){
                QString s = QString::number(r_intrinsic.at<double>(i,j));
                str += s;
                str += QString(20-s.length(),' ');
                str += '\r';
            }
            str +='\n';
        }
        this->ui->r_IntrinLabel->setText(str);
    }
    if(this->doc->getRdistortion(r_distortion)){
        QString str="";
        for(int i = 0;i<r_distortion.rows;i++) {
            for(int j = 0;j<r_distortion.cols;j++){
                QString s = QString::number(r_distortion.at<double>(i,j));
                str += s;
                str += QString(20-s.length(),' ');
                str += '\r';
            }
            str +='\n';
        }
        this->ui->r_DistorLabel->setText(str);
    }
}

//截屏
void MainWindow::on_actionTake_Photoes_triggered()
{
    PhotoDialog  dig(this);
    dig.exec();
}
//打开文件
void MainWindow::on_actionOpen_file_triggered()
{
    QString filename = QFileDialog::getOpenFileName(
       this,
       "Open Document",
       QDir::currentPath(),
       "Document files (*.rtf);;All files(*.*)");
    if (!filename.isNull()) { //用户选择了文件
          this->setWindowTitle(filename.mid(filename.lastIndexOf("/")+1));
          ui->actionNew->setEnabled(false);
          ui->actionSave->setEnabled(true);

          ui->action3D_reconstruction->setEnabled(true);
          ui->actionDual->setEnabled(true);
          ui->actionPolar_Correction->setEnabled(true);
          ui->actionSigalCamera->setEnabled(true);
          ui->actionTake_Photoes->setEnabled(true);
          ui->actionTake_Photoes_2->setEnabled(true);

          ui->stackedWidget->setCurrentIndex(1);
    } else{ // 用户取消选择

    }
}
//保存文件
void MainWindow::on_actionSave_triggered()
{
    if(this->doc->opened()){
        this->doc->save();
    }else{
        QString fileName = QFileDialog::getSaveFileName(
                    this,
                    tr("Save File"),
                    "untitled.rtf",
                    tr("rtf files (*.rtf)"));
        if(!fileName.isNull()){
            this->setWindowTitle(fileName.mid(fileName.lastIndexOf("/")+1));
        }
    }
}
//
void MainWindow::on_actionSigalCamera_triggered()
{
    ui->stackedWidget->setCurrentIndex(1);
    setupCalibUI();
}
//新建文档
void MainWindow::on_actionNew_triggered()
{
    ui->actionOpen_file->setEnabled(false);
    ui->actionSave->setEnabled(true);
    ui->actionNew->setEnabled(false);

    ui->action3D_reconstruction->setEnabled(true);
    ui->actionDual->setEnabled(true);
    ui->actionPolar_Correction->setEnabled(true);
    ui->actionSigalCamera->setEnabled(true);
    ui->actionTake_Photoes->setEnabled(true);
    ui->actionTake_Photoes_2->setEnabled(true);

    this->setWindowTitle("untitiled.rtf");
    ui->stackedWidget->setCurrentIndex(1);
}

void MainWindow::on_adddataBut_clicked()
{
    QString filename = QFileDialog::getOpenFileName(
       this,
       "Open Document",
       QDir::currentPath(),
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!filename.isNull()) { //用户选择了文件
          this->doc->selectBegin(0,filename.toStdString());
          SigalDialog dig(filename,this->doc,this);
          dig.setWindowTitle(filename.mid(filename.lastIndexOf("/")+1));
          dig.exec();
          this->loadCalidUI();
    }
}

void MainWindow::on_pushButton_clicked()
{

}

void MainWindow::on_rightAddData_clicked()
{
    QString filename = QFileDialog::getOpenFileName(
       this,
       "Open Document",
       QDir::currentPath(),
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!filename.isNull()) { //用户选择了文件
          this->doc->selectBegin(1,filename.toStdString());
          SigalDialog dig(filename,this->doc,this);
          dig.setWindowTitle(filename.mid(filename.lastIndexOf("/")+1));
          dig.exec();
          this->loadCalidUI();
    }
}

void MainWindow::on_actionDual_triggered()
{
    ui->stackedWidget->setCurrentIndex(2);
}

void MainWindow::on_bincularAddBut_clicked()
{
    QString leftfilename = QFileDialog::getOpenFileName(
       this,
       "Binocular Calibration - Open Left Image",
       QDir::currentPath(),
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!leftfilename.isNull()) { //用户选择了左图文件
        QString rightfilename = QFileDialog::getOpenFileName(
           this,
           "Binocular Calibration - Open Left Image",
           QDir::currentPath(),
           "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
        if (!rightfilename.isNull()) { //用户选择了右图文件



        }
    }
}
