#include "sigaldialog.h"
#include "ui_sigaldialog.h"
#include "clicklabel.h"
#include <QStackedLayout>

SigalDialog::SigalDialog(QString filename, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SigalDialog)
{
    ui->setupUi(this);
    this->parent = parent;

    QStackedLayout *layout = new QStackedLayout(ui->ImageContainer);
    ClickLabel *label = new ClickLabel(filename,ui->ImageContainer);
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
