#include "cornerdialog.h"
#include "ui_cornerdialog.h"
#include <QWheelEvent>
#include "uti.h"
#include <QDebug>

using namespace std;

CornerDialog::CornerDialog(Mat &img, vector<Point2f> &corners, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CornerDialog)
{
    ui->setupUi(this);
    this->img = img.clone();
    this->corners = corners;
    this->firstload = true;
    rate = 1;
    Mat showmat;
    this->calMat(showmat,Point(img.cols/2,img.rows/2),1);
    this->ui->label->setPixmap(QPixmap::fromImage(Mat2QImage(showmat)));
}

CornerDialog::~CornerDialog()
{
    delete ui;
}

void CornerDialog::wheelEvent(QWheelEvent *event){
    //if(event->angleDelta())
}
void CornerDialog::calMat(Mat &showmat, Point c,double prerate){
    if(this->firstload){
        this->firstload = false;
        winwidth = this->img.cols;
        winheight = this->img.rows;
    }else{
        winwidth = ui->scrollArea->width();
        winheight = ui->scrollArea->height();
    }

    int swinwidth = winwidth / rate;
    rate = winwidth*1.0/swinwidth;
    int swinheight = winheight / rate;


    //小于显示窗口，就不放大了
    if(swinwidth>=this->img.cols||swinheight>=this->img.rows){
        showmat  = this->img.clone();
        for(int i = 0;i<corners.size();i++){
            circle(showmat,Point(corners[i].x,corners[i].y),6,Scalar(0,0,255));
        }
        Size dsize(this->img.cols*rate,this->img.rows*rate);
        cv::resize(showmat,showmat,dsize);
        this->showdx = -1;
        this->showdy = -1;
        return;
    }
    if(this->showdx==-1)
        this->showdx = this->showdy = 0;

    Rect r;
    r.x = this->showdx + c.x/prerate - c.x/rate;
    r.y = this->showdy + c.y/prerate - c.y/rate;
    r.width = swinwidth;
    r.height = swinheight;
    if(r.x<0)r.x = 0;
    if(r.y<0)r.y = 0;
    if(r.x+swinwidth>=img.cols)r.x = img.cols - swinwidth;
    if(r.y+swinheight>=img.rows)r.y = img.rows - swinheight;

    qDebug()<<"img width"<<img.cols<<endl;
    qDebug()<<"img height"<<img.rows<<endl;
    qDebug()<<"r.x"<<r.x<<endl;
    qDebug()<<"r.y"<<r.y<<endl;
    qDebug()<<"swinwidth"<<swinwidth<<endl;
    qDebug()<<"swinheight"<<swinheight<<endl;
    Size dsize(winwidth,winheight);
    cv::resize(this->img(r),showmat,dsize,0,0,INTER_NEAREST);
    for(int i = 0;i<this->corners.size();i++){
        if(corners[i].x>=r.x&&corners[i].x<=r.x+swinwidth&&
                corners[i].y>=r.y&&corners[i].y<=r.y+swinheight){
            circle(showmat,Point((corners[i].x-r.x)*rate,(corners[i].y-r.y)*rate),6,Scalar(0,0,255));
        }
    }
    this->showdx = r.x;
    this->showdy = r.y;
}

void CornerDialog::mouseDoubleClickEvent(QMouseEvent *event){
    double prerate = this->rate;
    if(event->button()==Qt::LeftButton){
        this->rate *= 1.1;
    }
    else if(event->button()==Qt::RightButton){
        this->rate*=0.9;
    }
    int x = event->x() - ui->scrollArea->x();
    int y = event->y() - ui->scrollArea->y();
    if(rate>=100)rate=100;
    if(rate<=0.1)rate=0.1;
    Mat showmat;
    if(this->showdx!=-1)
        this->calMat(showmat,Point(x,y),prerate);
    else this->calMat(showmat,Point(this->img.cols/2,this->img.rows/2),prerate);
    this->ui->label->setPixmap(QPixmap::fromImage(Mat2QImage(showmat)));
}
