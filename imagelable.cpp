#include "imagelable.h"
#include "ui_imagelable.h"

ImageLabel::ImageLabel(QString filename, QWidget *parent) :
    QLabel(parent),
    ui(new Ui::ImageLabel)
{
    ui->setupUi(this);
    this->pixmap = new QPixmap(filename);
    this->setPixmap(*this->pixmap);
}

ImageLable::~ImageLable()
{
    delete ui;
    delete this->pixmap;
}
void ImageLable::mouseMoveEvent(QMouseEvent *ev){

}
