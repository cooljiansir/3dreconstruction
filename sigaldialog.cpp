#include "sigaldialog.h"
#include "ui_sigaldialog.h"
#include "clicklabel.h"
#include <QStackedLayout>

SigalDialog::SigalDialog(QString filename, RTFDocument *doc, QWidget *parent):
    QDialog(parent),
    ui(new Ui::SigalDialog)
{
    ui->setupUi(this);

    QStackedLayout *layout = new QStackedLayout(ui->ImageContainer);
    this->label = new ClickLabel(filename,doc,ui->ImageContainer);
    layout->addWidget(label);
//    /label->setPixmap(QPixmap(filename));

    this->setWindowFlags(Qt::Dialog
                         |Qt::WindowMaximizeButtonHint
                         |Qt::WindowMinimizeButtonHint
                         |Qt::WindowCloseButtonHint);
    ui->okBut->setEnabled(false);
    ui->undoBut->setEnabled(false);
}

SigalDialog::~SigalDialog()
{
    delete ui;
}

void SigalDialog::on_cancelBut_clicked()
{
    this->close();
}

void SigalDialog::on_okBut_clicked()
{
    ClickLabel *la = (ClickLabel*)this->label;
    la->onOk();
}
void SigalDialog::setOkButEnable(bool b){
    //ui->okBut->
}
