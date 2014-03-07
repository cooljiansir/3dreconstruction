#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QStandardItemModel>
#include <QFileDialog>
#include <QDir>
#include <QMessageBox>
#include "sigaldialog.h"

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
    this->ui->stackedWidget->setCurrentIndex(1);//welcome!
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
    singal_model->setColumnCount(5);
    this->ui->tableView->setModel(singal_model);
    singal_model->setHeaderData(0,Qt::Horizontal,QString::fromLocal8Bit("Image"));
    singal_model->setHeaderData(1,Qt::Horizontal,QString::fromLocal8Bit("u"));
    singal_model->setHeaderData(2,Qt::Horizontal,QString::fromLocal8Bit("v"));
    singal_model->setHeaderData(3,Qt::Horizontal,QString::fromLocal8Bit("x"));
    singal_model->setHeaderData(4,Qt::Horizontal,QString::fromLocal8Bit("y"));
    this->loadCalidUI();
}
void MainWindow::loadCalidUI(){
    vector<vector<Point2f> > image_point;
    vector<vector<Point2f> > object_point;
    this->doc->getCorner(image_point,object_point);
    int count = 0;
    for(int i = 0;i<image_point.size();i++){
        for(int j = 0;j<image_point[i].size();j++){
            singal_model->setItem(count,0,new QStandardItem(QString::number(i)));
            singal_model->setItem(count,1,new QStandardItem(QString::number(image_point[i][j].x)));
            singal_model->setItem(count,2,new QStandardItem(QString::number(image_point[i][j].y)));
            singal_model->setItem(count,3,new QStandardItem(QString::number(object_point[i][j].x)));
            singal_model->setItem(count,4,new QStandardItem(QString::number(object_point[i][j].y)));
            count++;
        }
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
    ui->stackedWidget->setCurrentIndex(0);
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
}

void MainWindow::on_adddataBut_clicked()
{
    QString filename = QFileDialog::getOpenFileName(
       this,
       "Open Document",
       QDir::currentPath(),
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!filename.isNull()) { //用户选择了文件
        this->doc->selectBegin(filename.toStdString());
          //this->doc->selectBegin(filename.toStdString());
          SigalDialog dig(filename,this->doc,this);
          dig.setWindowTitle(filename.mid(filename.lastIndexOf("/")+1));
          dig.exec();
          this->loadCalidUI();
    }
}
