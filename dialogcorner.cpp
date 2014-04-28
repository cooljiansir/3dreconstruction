#include "dialogcorner.h"
#include "ui_dialogcorner.h"
#include "uti.h"
#include <QKeyEvent>

DialogCorner::DialogCorner(Mat &img,vector<Point2f> &corners,QWidget *parent) :
    QDialog(parent),
    ui(new Ui::DialogCorner)
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

DialogCorner::~DialogCorner()
{
    delete ui;
}
void DialogCorner::resizeEvent(QResizeEvent *){
    showSmall();
    showBig();
}



void DialogCorner::scaledTo(Mat &src, Mat &res, int w, int h){
    int w1 = w;
    int h1 = w*src.rows/src.cols;
    if(h1>h){
        h1 = h;
        w1 = h*src.cols/src.rows;
    }
    cv::resize(src,res,Size(w1,h1));
}

void DialogCorner::showBig(){

}
void DialogCorner::showSmall(){
    this->smallwinwidth = ui->bigBorder->width()/bigrate;
    this->smallwinheight = ui->bigBorder->height()/bigrate;
    double realsx = ui->bigBorder->width()*1.0/this->smallwinwidth;
    double realsy = ui->bigBorder->height()*1.0/this->smallwinheight;
    Mat showmat = img.clone();

    int w = this->smallwinwidth;
    int h = this->smallwinheight;
    if(w>img.cols)w = img.cols;
    if(h>img.rows)h = img.rows;
    int x = this->smallx*this->smallrate - this->smallwinwidth/2;
    int y = this->smally*this->smallrate - this->smallwinheight/2;
    if(x<0)x = 0;
    if(y<0)y = 0;
    if(x+w>=img.cols)x = img.cols - w;
    if(y+h>=img.rows)y = img.rows - h;
    cv::rectangle(showmat,Rect(x,y,w,h),Scalar(0,0,255),4);
    Mat sMat = img(Rect(x,y,w,h));
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
            int li = 15;
            if(p.x-li>=0&&p.y-li>=0&&p.x+li<showbigmat.cols&&p.y+li<showbigmat.rows){
                line(showbigmat,Point(p.x-li,p.y),Point(p.x+li,p.y),Scalar(0,0,255));
                line(showbigmat,Point(p.x,p.y-li),Point(p.x,p.y+li),Scalar(0,0,255));
            }
        }
    }
    this->scaledTo(showbigmat,showbigmat,ui->bigBorder->width(),ui->bigBorder->height());
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
void DialogCorner::keyPressEvent(QKeyEvent *e){
    bool changed = false;
    if(e->key()==Qt::Key_Left){
        this->smallx --;
        if(this->smallx*this->smallrate<this->smallwinwidth/2)
            this->smallx = this->smallwinwidth/this->smallrate/2;
        if(this->smallx*this->smallrate+this->smallwinwidth/2>=img.cols)
            this->smallx = (this->img.cols-1-this->smallwinwidth/2)/this->smallrate;
        changed = true;
    }
    else if(e->key()==Qt::Key_Right){
        this->smallx ++;
        if(this->smallx*this->smallrate<this->smallwinwidth/2)
            this->smallx = this->smallwinwidth/this->smallrate/2;
        if(this->smallx*this->smallrate+this->smallwinwidth/2>=img.cols)
            this->smallx = (this->img.cols-1-this->smallwinwidth/2)/this->smallrate;
        changed = true;
    }else if(e->key()==Qt::Key_Up){
        this->smally--;
        if(this->smally*this->smallrate<this->smallwinheight/2)
            this->smally=this->smallwinheight/this->smallrate/2;
        if(this->smally*this->smallrate+this->smallwinheight/2>=img.rows)
            this->smally = (img.rows-1-this->smallwinheight/2)/this->smallrate;
        changed = true;
    }else if(e->key()==Qt::Key_Down){
        this->smally++;
        if(this->smally*this->smallrate<this->smallwinheight/2)
            this->smally=this->smallwinheight/this->smallrate/2;
        if(this->smally*this->smallrate+this->smallwinheight/2>=img.rows)
            this->smally = (img.rows-1-this->smallwinheight/2)/this->smallrate;
        changed = true;
    }
    if(changed){
        e->accept();
        this->showSmall();
    }
}

void DialogCorner::on_changeScale_clicked()
{
    this->bigrate = this->ui->lineEdit->text().toInt();
    this->showSmall();
    this->setFocus();
}

void DialogCorner::onScaleLabelClicked(int x, int y){
    this->smallx = x;
    this->smally = y;
    this->showBig();
    this->showSmall();
}
