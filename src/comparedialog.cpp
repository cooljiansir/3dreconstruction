#include "comparedialog.h"
#include "ui_comparedialog.h"
#include "uti.h"

CompareDialog::CompareDialog(Mat &leftMat, Mat &rightMat, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::CompareDialog)
{
    ui->setupUi(this);
    ui->leftLabel->setPixmap(QPixmap::fromImage(Mat2QImage(leftMat)));
    ui->rightLabel->setPixmap(QPixmap::fromImage(Mat2QImage(rightMat)));
    this->setWindowFlags(Qt::Dialog
                         |Qt::WindowMaximizeButtonHint
                         |Qt::WindowMinimizeButtonHint
                         |Qt::WindowCloseButtonHint);
    this->showMaximized();
}

CompareDialog::~CompareDialog()
{
    delete ui;
}

void CompareDialog::on_pushButton_clicked()
{
    this->close();
}
