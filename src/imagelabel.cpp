#include "imagelabel.h"
#include "ui_imagelabel.h"
#include <QMessageBox>
#include <QMouseEvent>
#include <QCursor>

ImageLabel::ImageLabel(QString filename, QWidget *parent) :
    QLabel(parent),
    ui(new Ui::ImageLabel)
{
    ui->setupUi(this);
    this->status = STATUS_FINDING;
    this->pixmap = new QPixmap(filename);
    QMessageBox::about(this,"debug","debug");
    //跟踪鼠标，否则只有在鼠标按下拖动的时候才能响应mousemove
    QWidget::setMouseTracking(true);
    this->mouse_x = 0;
    this->mouse_y = 0;
}

ImageLabel::~ImageLabel()
{
    delete ui;
}
void ImageLabel::mouseMoveEvent(QMouseEvent *ev){
    this->mouse_x = QCursor::pos().x();
    this->mouse_y = QCursor::pos().y();
}
void ImageLabel::paintEvent(QPaintEvent *t){
    QLabel::paintEvent(t);
}
