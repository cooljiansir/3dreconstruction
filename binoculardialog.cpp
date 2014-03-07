#include "binoculardialog.h"
#include "ui_binoculardialog.h"

BinocularDialog::BinocularDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::BinocularDialog)
{
    ui->setupUi(this);
}

BinocularDialog::~BinocularDialog()
{
    delete ui;
}
