#include "photodialog.h"
#include "ui_photodialog.h"
#include "uti.h"
#include <QDebug>
#include <QKeyEvent>

PhotoDialog::PhotoDialog(Mat &img, vector<Point2f> &corners, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PhotoDialog)
{
    ui->setupUi(this);
    this->img = img.clone();
    this->corners = corners;
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
    qDebug()<<"clicked!"<<endl;
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
