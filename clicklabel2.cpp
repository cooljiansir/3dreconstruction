#include "clicklabel2.h"
#include "ui_clicklabel2.h"

ClickLabel2::ClickLabel2(QWidget *parent) :
    QLabel(parent),
    ui(new Ui::ClickLabel2)
{
    ui->setupUi(this);
}

ClickLabel2::~ClickLabel2()
{
    delete ui;
}
