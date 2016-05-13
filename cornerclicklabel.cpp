#include "cornerclicklabel.h"
#include <QMouseEvent>
#include <QDebug>

CornerClickLabel::CornerClickLabel(QWidget *parent) :
    QLabel(parent)
{
    if(parent!=NULL&&parent->parent()!=NULL){
        connect(this,SIGNAL(mousePressed(int,int)),parent->parent(),SLOT(onScaleLabelClicked(int,int)));
    }
}
void CornerClickLabel::mousePressEvent(QMouseEvent *ev){

    emit mousePressed(ev->x(),ev->y());
}
