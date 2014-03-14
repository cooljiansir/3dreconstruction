#include "photodialog.h"
#include "ui_photodialog.h"
#include "uti.h"
#include <QFileDialog>

PhotoDialog::PhotoDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PhotoDialog)
{
    ui->setupUi(this);
    this->capture = 0;
    this->capn = 0;
    this->isSwitching = false;
    this->isshot = false;
    this->openNext();
    this->timerInt = this->startTimer(30);

    this->setWindowFlags(Qt::Dialog
                         |Qt::WindowMaximizeButtonHint
                         |Qt::WindowMinimizeButtonHint
                         |Qt::WindowCloseButtonHint);
    this->setWindowTitle("Take Photo");
    this->showMaximized();
}

PhotoDialog::~PhotoDialog()
{
    delete ui;
    if(this->capture){
        this->capture->release();
        delete this->capture;
    }
}
void PhotoDialog::timerEvent(QTimerEvent *){
    if(this->isSwitching||this->isshot)
        return;
    if(this->capture->isOpened())
        (*this->capture)>>imagemat;
    QImage img = Mat2QImage(this->imagemat);
    this->ui->imageLabel->setPixmap(QPixmap::fromImage(img));
}
void PhotoDialog::openNext(){
    if(this->isSwitching)return;
    this->isSwitching = true;
    VideoCapture *an = new VideoCapture(this->capn++);
    while(!an->isOpened()){
        an->release();
        delete an;
        if(this->capn<=2){//only one camera
            this->capn = 1;
            this->isSwitching = false;
            return;
        }
        this->capn = 0;
        an = new VideoCapture(this->capn++);
    }
    if(this->capture){
        this->capture->release();
        delete this->capture;
    }
    this->capture = an;
    this->isSwitching = false;
}

void PhotoDialog::on_pushButton_4_clicked()
{
    this->openNext();
}

void PhotoDialog::on_printBut_clicked()
{
    if(!this->isshot){
        this->isshot = true;
        this->ui->printBut->setText("Save");
    }else{
        QString fileName = QFileDialog::getSaveFileName(
                    this,
                    tr("Save File"),
                    QDir::currentPath(),
                    "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
        if(!fileName.isNull()){
            imwrite(fileName.toStdString().c_str(),this->imagemat);
            this->close();
        }
    }
}

void PhotoDialog::on_cancelBut_clicked()
{
    if(this->isshot){
        this->ui->printBut->setText("Print");
        this->isshot = false;
    }else{
        this->close();
    }
}
