#include "clickframe.h"
#include "ui_clickframe.h"
#include <QMessageBox>
#include <QWidget>
#include <QCursor>
#include <QPainter>

ClickFrame::ClickFrame(QWidget *parent) :
    QFrame(parent),
    ui(new Ui::ClickFrame)
{
    ui->setupUi(this);
    QWidget::setMouseTracking(true);
}

ClickFrame::~ClickFrame()
{
    delete ui;
}
void ClickFrame::mouseMoveEvent(QMouseEvent *){
//    QMessageBox::information(this,"debug","debug");
    this->mouse_x = QCursor::pos().x();
    this->mouse_y = QCursor::pos().y();
    this->repaint();
    char temp1[100];
    sprintf(temp1,"%d",mouse_x);
    char temp2[100];
    sprintf(temp2,"%d",mouse_y);
    QMessageBox::information(this,"debug",QString(" x ")+temp1+QString(" y ")+temp2);
}


void ClickFrame::paintEvent(QPaintEvent *t){
    QFrame::paintEvent(t);
    QPainter painter(this);
    painter.drawText(5,5,"haha");
    //painter.drawRect(this->mouse_x,this->mouse_y,100,100);
}
