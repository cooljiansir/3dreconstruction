#include "photodialog.h"
#include "ui_photodialog.h"
#include "uti.h"
#include <QDebug>
#include <QKeyEvent>

#include "highgui.h"

PhotoDialog::PhotoDialog(QString filename, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PhotoDialog)
{
    ui->setupUi(this);
    //this->img = filename.clone();
    //this->corners = corners;
    this->filename = filename;
    this->ui->radioButHarris->setChecked(true);
    this->ui->radioButFitting->setChecked(true);
    this->loadMat();
    this->smallx = this->smally = 0;
    this->bigrate = 5;
    this->ui->lineEdit->setText(QString::number(this->bigrate));
    this->setWindowFlags(Qt::Dialog
                         |Qt::WindowCloseButtonHint
                         |Qt::WindowMaximizeButtonHint
                         |Qt::WindowMinimizeButtonHint);
    this->setWindowTitle("Observe Corners");
    this->showMaximized();
    this->setFocus();
    this->on_radioButHarris_clicked();
}

PhotoDialog::~PhotoDialog()
{
    delete ui;
}

void PhotoDialog::resizeEvent(QResizeEvent *){
    showSmall();
    showBig();
}

void PhotoDialog::onScaleLabelClicked(int x, int y){
    this->smallx = x*this->smallrate;
    this->smally = y*this->smallrate;
    this->showBig();
    this->showSmall();
}

void PhotoDialog::scaledTo(Mat &src, Mat &res, int w, int h){
    int w1 = w;
    int h1 = w*src.rows/src.cols;
    if(h1>h){
        h1 = h;
        w1 = h*src.cols/src.rows;
    }
    cv::resize(src,res,Size(w1,h1));
}

void PhotoDialog::showBig(){

}
void PhotoDialog::showSmall(){
    this->smallwinwidth = ui->bigBorder->width()/bigrate;
    this->smallwinheight = ui->bigBorder->height()/bigrate;


    if(this->smallwinwidth>img.cols)this->smallwinwidth = img.cols;
    if(this->smallwinheight>img.rows)this->smallwinheight = img.rows;


    int x = this->smallx - this->smallwinwidth/2;
    int y = this->smally - this->smallwinheight/2;
    if(x<0)x = 0;
    if(y<0)y = 0;
    if(x+this->smallwinwidth>=img.cols)x = img.cols - this->smallwinwidth;
    if(y+this->smallwinheight>=img.rows)y = img.rows - this->smallwinheight;

    double realsx = ui->bigBorder->width()*1.0/this->smallwinwidth;
    double realsy = ui->bigBorder->height()*1.0/this->smallwinheight;
    Mat showmat = img.clone();

    cv::rectangle(showmat,Rect(x,y,smallwinwidth,smallwinheight),Scalar(0,0,255),4);
    Mat sMat = img(Rect(x,y,smallwinwidth,smallwinheight));
    Mat showbigmat;


    Size dsize(ui->bigBorder->width(),ui->bigBorder->height());
    cv::resize(sMat,showbigmat,dsize,0,0,INTER_NEAREST);

    for(int i = 0;i<this->corners.size();i++){
        if(corners[i].x>=x&&corners[i].y>=y
                &&corners[i].x<smallwinwidth+x
                &&corners[i].y<smallwinheight+y){
            Point p;
            p.x = (corners[i].x - x)*realsx;
            p.y = (corners[i].y - y)*realsy;
            //circle(showbigmat,p,6,Scalar(0,0,255));
            int li = 10;
            if(p.x-li>=0&&p.y-li>=0&&p.x+li<showbigmat.cols&&p.y+li<showbigmat.rows){
                line(showbigmat,Point(p.x-li,p.y),Point(p.x+li,p.y),Scalar(0,0,255));
                line(showbigmat,Point(p.x,p.y-li),Point(p.x,p.y+li),Scalar(0,0,255));
            }
        }
    }
//    this->scaledTo(showbigmat,showbigmat,ui->bigBorder->width(),ui->bigBorder->height());
    this->ui->bigLabel->setPixmap(QPixmap::fromImage(Mat2QImage(showbigmat)));

    for(int i = 0;i<this->corners.size();i++){
        Point p;
        p.x = corners[i].x;
        p.y = corners[i].y;
        circle(showmat,p,6,Scalar(0,0,255));
    }
    this->scaledTo(showmat,showmat,ui->scaleBorder->width(),ui->scaleBorder->height());
    this->ui->scaleLabel->setPixmap(QPixmap::fromImage(Mat2QImage(showmat)));
    this->smallrate = (img.cols*1.0/showmat.cols + img.rows*1.0/showmat.rows)/2;
}
void PhotoDialog::keyPressEvent(QKeyEvent *e){
    bool changed = false;
    if(e->key()==Qt::Key_Left){
        this->smallx --;
        if(this->smallx<this->smallwinwidth/2)
            this->smallx = this->smallwinwidth/2;
        if(this->smallx+this->smallwinwidth/2>=img.cols)
            this->smallx = this->img.cols-1-this->smallwinwidth/2;
        changed = true;
    }
    else if(e->key()==Qt::Key_Right){
        this->smallx ++;
        if(this->smallx<this->smallwinwidth/2)
            this->smallx = this->smallwinwidth/2;
        if(this->smallx+this->smallwinwidth/2>=img.cols)
            this->smallx = this->img.cols-1-this->smallwinwidth/2;
        changed = true;
    }else if(e->key()==Qt::Key_Up){
        this->smally--;
        if(this->smally<this->smallwinheight/2)
            this->smally=this->smallwinheight/2;
        if(this->smally+this->smallwinheight/2>=img.rows)
            this->smally = img.rows-1-this->smallwinheight/2;
        changed = true;
    }else if(e->key()==Qt::Key_Down){
        this->smally++;

        if(this->smally<this->smallwinheight/2)
            this->smally=this->smallwinheight/2;
        if(this->smally+this->smallwinheight/2>=img.rows)
            this->smally = img.rows-1-this->smallwinheight/2;
        changed = true;
    }
    if(changed){
        e->accept();
        this->showSmall();
    }
}

void PhotoDialog::on_pushButton_clicked()
{
    this->bigrate = this->ui->lineEdit->text().toInt();
    this->showSmall();
    this->setFocus();
}
void PhotoDialog::loadMat(){
    this->img = imread(this->filename.toStdString().c_str());
}

void PhotoDialog::on_radioButMoravec_clicked()
{
    this->ui->labelK->setVisible(false);
    this->ui->lineEditK->setVisible(false);
    this->corn.setPixMethod(Corner::PIX_MORAVEC);
    this->ui->lineEditThreshold->setText(QString::number(corn.getThreshold(Corner::PIX_MORAVEC)));
    this->ui->lineEditWinsize->setText(QString::number(this->corn.getWinsize()));
    this->corn.getCorner(img,this->corners);
    this->showSmall();
}

void PhotoDialog::on_radioButHarris_clicked()
{
    this->ui->labelK->setVisible(true);
    this->ui->lineEditK->setVisible(true);
    this->corn.setPixMethod(Corner::PIX_HARRIS);
    this->ui->lineEditThreshold->setText(QString::number(corn.getThreshold(Corner::PIX_HARRIS)));
    this->ui->lineEditWinsize->setText(QString::number(this->corn.getWinsize()));
    this->ui->lineEditK->setText(QString::number(this->corn.getK()));
    this->corn.getCorner(img,this->corners);
    this->showSmall();
}

void PhotoDialog::on_radioButNobel_clicked()
{
    this->ui->labelK->setVisible(false);
    this->ui->lineEditK->setVisible(false);
    this->corn.setPixMethod(Corner::PIX_NOBEL);
    this->ui->lineEditThreshold->setText(QString::number(corn.getThreshold(Corner::PIX_NOBEL)));
    this->ui->lineEditWinsize->setText(QString::number(this->corn.getWinsize()));
    this->corn.getCorner(img,this->corners);
    this->showSmall();
}

void PhotoDialog::on_radioButShiTomasi_clicked()
{
    this->ui->labelK->setVisible(false);
    this->ui->lineEditK->setVisible(false);
    this->corn.setPixMethod(Corner::PIX_SHI_TOMASI);
    this->ui->lineEditThreshold->setText(QString::number(corn.getThreshold(Corner::PIX_SHI_TOMASI)));
    this->ui->lineEditWinsize->setText(QString::number(this->corn.getWinsize()));
    this->corn.getCorner(img,this->corners);
    this->showSmall();
}

void PhotoDialog::on_radioButFitting_clicked()
{
    this->corn.setSubPixMethod(Corner::SUBPIX_FITTING);
    this->corn.getCorner(img,this->corners);
    this->showSmall();
}

void PhotoDialog::on_radioButVector_clicked()
{
    this->corn.setSubPixMethod(Corner::SUBPIX_VECTOR);
    this->corn.getCorner(img,this->corners);
    this->showSmall();
}

void PhotoDialog::on_changePraBut_clicked()
{
    bool ok;
    double th = this->ui->lineEditThreshold->text().toDouble(&ok);
    if(!ok)return;
    double k = this->ui->lineEditK->text().toDouble(&ok);
    if(!ok)return;
    int winsize = this->ui->lineEditWinsize->text().toInt(&ok);
    if(!ok)return;
    this->corn.setThreshold(th);
    this->corn.setK(k);
    this->corn.setWinsize(winsize);
    this->corn.getCorner(img,this->corners);
    this->showSmall();
    this->setFocus();
}
