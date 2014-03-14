#include "photodialog2.h"
#include "ui_photodialog2.h"
#include "uti.h"
#include <QFileDialog>

PhotoDialog2::PhotoDialog2(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PhotoDialog2)
{
    ui->setupUi(this);
    this->capl = -1;
    this->capr = -1;
    this->isshot = false;
    this->capleft = this->capright = 0;
    this->isswitch = false;
    ui->leftEdit->setText("0");
    ui->rightEdit->setText("1");
    this->on_pushButton_clicked();
    this->startTimer(30);
    this->setWindowFlags(Qt::Dialog
                         |Qt::WindowMaximizeButtonHint
                         |Qt::WindowMinimizeButtonHint
                         |Qt::WindowCloseButtonHint);
    this->setWindowTitle("Take Photos");
    this->showMaximized();
}

PhotoDialog2::~PhotoDialog2()
{
    delete ui;
    if(this->capleft){
        this->capleft->release();
        delete this->capleft;
    }
    if(this->capright){
        this->capright->release();
        delete this->capright;
    }
}
void PhotoDialog2::timerEvent(QTimerEvent *){
    if(this->isswitch||this->isshot)
        return;
    if(this->capleft->isOpened()){
        (*capleft)>>leftMat;
        QImage imgl = Mat2QImage(this->leftMat);
        ui->labelLeft->setPixmap(QPixmap::fromImage(imgl));
    }
    if(this->capright->isOpened()){
        (*capright)>>rightMat;
        QImage imgr = Mat2QImage(this->rightMat);
        ui->labelRight->setPixmap(QPixmap::fromImage(imgr));
    }
}

void PhotoDialog2::on_pushButton_clicked()
{
    if(this->isswitch)return;
    this->isswitch = true;
    bool t;
    int tl = this->ui->leftEdit->text().toInt(&t);
    if(!t){
        this->isswitch = false;
        return;
    }
    int tr = this->ui->rightEdit->text().toInt(&t);
    if(!t){
        this->isswitch = false;
        return;
    }
    if(this->capleft){
        this->capleft->release();
        delete this->capleft;
    }
    if(this->capright){
        this->capright->release();
        delete this->capright;
    }
    VideoCapture *p = new VideoCapture(tl);
    if(p->isOpened()){
        this->capleft = p;
    }
    VideoCapture *pr = new VideoCapture(tr);
    if(pr->isOpened()){
        this->capright = pr;
    }
    this->isswitch = false;
}


void PhotoDialog2::on_cancelBut_clicked()
{
    if(this->isshot){
        this->isshot = false;
        this->ui->printBut->setText("Print");
    }else{
        this->close();
    }
}

void PhotoDialog2::on_printBut_clicked()
{
    if(!this->isshot){
        this->isshot = true;
        ui->printBut->setText("Save");
    }else{
        QString fileNamel = QFileDialog::getSaveFileName(
                    this,
                    tr("Save Left Image"),
                    QDir::currentPath(),
                    "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
        if(!fileNamel.isNull()){
            QString fileNamer = QFileDialog::getSaveFileName(
                        this,
                        tr("Save Right Image"),
                        QDir::currentPath(),
                        "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
            if(!fileNamer.isNull()){
                imwrite(fileNamel.toStdString().c_str(),this->leftMat);
                imwrite(fileNamer.toStdString().c_str(),this->rightMat);
                this->close();
            }else {
                this->isshot = false;
            }
        }else{
            this->isshot = false;
        }

    }
}
