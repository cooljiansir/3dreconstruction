#include "photodialog.h"
#include "ui_photodialog.h"

PhotoDialog::PhotoDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::PhotoDialog)
{
    ui->setupUi(this);
}

PhotoDialog::~PhotoDialog()
{
    delete ui;
}

