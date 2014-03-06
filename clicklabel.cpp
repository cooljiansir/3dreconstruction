#include "clicklabel.h"
#include "ui_clicklabel.h"
#include <QWidget>
#include <QMessageBox>
#include <QPainter>
#include <QMouseEvent>



ClickLabel::ClickLabel(QString filename, RTFDocument *doc, QWidget *parent) :
    QLabel(parent),
    ui(new Ui::ClickLabel)
{
    ui->setupUi(this);
    this->doc = doc;
    QWidget::setMouseTracking(true);
    this->pixmap = new QPixmap(filename);
    this->setPixmap(*pixmap);
}

ClickLabel::~ClickLabel()
{
    delete ui;
    delete this->pixmap;
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
    painter.drawLine(mouse_x,0,mouse_x,this->height());
    painter.drawLine(0,mouse_y,this->width(),mouse_y);
    for(int i = 0;i<this->mouse_x_v.size();i++){
        painter.drawLine(mouse_x_v[i]-4,mouse_y_v[i],mouse_x_v[i]+4,mouse_y_v[i]);
        painter.drawLine(mouse_x_v[i],mouse_y_v[i]-4,mouse_x_v[i],mouse_y_v[i]+4);
    }
}
void ClickLabel::mousePressEvent(QMouseEvent *ev){
    //QMessageBox::information(this,"debug","debug");
    if(this->doc->havepoint(ev->x(),ev->y())){
        this->mouse_x_v.push_back(ev->x());
        this->mouse_y_v.push_back(ev->y());
        if(this->mouse_x_v.size()==4){

        }
    }else{
        QMessageBox::critical(this,"ERROR","没有选中角点");
    }
}

void ClickLabel::onOk(){
    if(mouse_x_v.size()!=4)
        return;
    vector<CPoint> points_4;
    for(int i = 0;i<mouse_x_v.size();i++){
        points_4.push_back(CPoint(mouse_x_v[i],mouse_y_v[i]));
    }
    vector<vector<Point2f> > corners;
    this->doc->getCornerByHand(points_4,corners);
}

