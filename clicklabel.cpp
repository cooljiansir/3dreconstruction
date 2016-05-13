#include "clicklabel.h"
#include "ui_clicklabel.h"
#include "sigaldialog.h"
#include <QWidget>
#include <QMessageBox>
#include <QPainter>
#include <QMouseEvent>
#include "uti.h"



ClickLabel::ClickLabel(int isright, QString filename, RTFDocument *doc, QWidget *parent) :
    QLabel(parent),
    ui(new Ui::ClickLabel)
{
    ui->setupUi(this);
    this->isright = isright;
    this->doc = doc;
    this->doc->selectBegin(filename.toStdString(),selectimg,harriscorners);
    QWidget::setMouseTracking(true);
    //this->pixmap = new QPixmap(filename);
    //this->setPixmap(*pixmap);
    this->setPixmap(QPixmap::fromImage(Mat2QImage(selectimg)));
    this->father  = parent;
    this->status = ClickLabel::STATUS_FINDING;
    connect(this,SIGNAL(ok()),parent,SLOT(onLabelOk()));
}

ClickLabel::~ClickLabel()
{
    delete ui;
    //delete this->pixmap;
}

void ClickLabel::mouseMoveEvent(QMouseEvent *e){
    this->mouse_x = e->x();
    this->mouse_y = e->y();
    this->repaint();
}
void ClickLabel::paintEvent(QPaintEvent *t){
    QLabel::paintEvent(t);
    QPainter painter(this);
    painter.setPen(QPen(Qt::red,1));
    switch (status) {
    case ClickLabel::STATUS_FINDING:
        painter.drawLine(mouse_x,0,mouse_x,this->height());
        painter.drawLine(0,mouse_y,this->width(),mouse_y);
        for(int i = 0;i<this->mouse_x_v.size();i++){
            painter.drawLine(mouse_x_v[i]-5,mouse_y_v[i],mouse_x_v[i]+5,mouse_y_v[i]);
            painter.drawLine(mouse_x_v[i],mouse_y_v[i]-5,mouse_x_v[i],mouse_y_v[i]+5);
        }
        break;
    case ClickLabel::STATUS_FOUND:
        for(int i = 0;i<this->corners.size();i++){
            for(int j = 0;j<this->corners[i].size();j++){
                if(i==j&&i==0){//坐标原点
                    painter.setPen(QPen(Qt::yellow,2));
                }else{
                    painter.setPen(QPen(Qt::red,1));
                }
                painter.drawLine(corners[i][j].x-5,corners[i][j].y-5,corners[i][j].x-5,corners[i][j].y+5);
                painter.drawLine(corners[i][j].x-5,corners[i][j].y-5,corners[i][j].x+5,corners[i][j].y-5);
                painter.drawLine(corners[i][j].x+5,corners[i][j].y+5,corners[i][j].x-5,corners[i][j].y+5);
                painter.drawLine(corners[i][j].x+5,corners[i][j].y+5,corners[i][j].x+5,corners[i][j].y-5);
                painter.drawLine(corners[i][j].x,corners[i][j].y+5,corners[i][j].x,corners[i][j].y-5);
                painter.drawLine(corners[i][j].x-5,corners[i][j].y,corners[i][j].x+5,corners[i][j].y);
            }
        }
        break;
    }
}
void ClickLabel::mousePressEvent(QMouseEvent *ev){
    //QMessageBox::information(this,"debug","debug");
    if(this->status!=ClickLabel::STATUS_FINDING)
        return;
    if(this->doc->havepoint(this->harriscorners,ev->x(),ev->y())){
        this->mouse_x_v.push_back(ev->x());
        this->mouse_y_v.push_back(ev->y());
        if(this->mouse_x_v.size()==4){
            vector<CPoint> points_4;
            for(int i = 0;i<mouse_x_v.size();i++){
                points_4.push_back(CPoint(mouse_x_v[i],mouse_y_v[i]));
            }
            this->doc->getCornerByHand(this->harriscorners,points_4,this->corners);
            if(this->corners.size()==0){
                QMessageBox::information(this,"ERROR","You don't select a triangle !");
                this->mouse_x_v.clear();
                this->mouse_y_v.clear();
                return ;
            }
            //SigalDialog *sig = (SigalDialog*)father;
            //sig->setOkButEnable(true);
            this->onOk();
            this->status = ClickLabel::STATUS_FOUND;
            this->repaint();
        }
    }else{
        QMessageBox::critical(this,"ERROR","You don't select a corner!");
    }
}

void ClickLabel::onOk(){
    emit this->ok();
}


