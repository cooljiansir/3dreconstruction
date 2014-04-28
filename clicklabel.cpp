#include "clicklabel.h"
#include <QMouseEvent>

ClickLabel::ClickLabel(QWidget *parent) :
    QLabel(parent)
{
    if(parent!=NULL&&parent->parent()!=NULL){
        connect(this,SIGNAL(mousePressed(int,int)),parent->parent(),SLOT(onScaleLabelClicked(int,int)));
    }
}
void ClickLabel::mousePressEvent(QMouseEvent *ev){
    emit mousePressed(ev->x(),ev->y());
}
