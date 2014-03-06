#include "clicklabel.h"
#include "ui_clicklabel.h"
#include <QWidget>
#include <QMessageBox>
#include <QPainter>
#include <QMouseEvent>



ClickLabel::ClickLabel(QString filename, QWidget *parent) :
    QLabel(parent),
    ui(new Ui::ClickLabel)
{
    ui->setupUi(this);
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
    painter.drawLine(mouse_x,0,mouse_x,this->height());
    painter.drawLine(0,mouse_y,this->width(),mouse_y);
}
void ClickLabel::mousePressEvent(QMouseEvent *ev){
    QMessageBox::information(this,"debug","debug");
}
