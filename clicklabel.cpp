#include "clicklabel.h"
#include "ui_clicklabel.h"
#include <QMouseEvent>

ClickLabel::ClickLabel(QWidget *parent) :
    QLabel(parent),
    ui(new Ui::ClickLabel)
{
    ui->setupUi(this);
    if(parent!=0){
        if(parent->parentWidget()!=0){
            if(parent->parentWidget()->parent()!=0)
            connect(this,SIGNAL(on_pressed(int,int))
                    ,parent->parentWidget()->parent(),SLOT(on_pressed(int,int)));
        }
    }
}

ClickLabel::~ClickLabel()
{
    delete ui;
}
void ClickLabel::mousePressEvent(QMouseEvent *ev){
    emit this->on_pressed(ev->x(),ev->y());
}
