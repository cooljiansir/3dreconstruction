#include "sigaldialog.h"
#include "ui_sigaldialog.h"
#include "clicklabel.h"
#include <QStackedLayout>
#include <QMessageBox>

SigalDialog::SigalDialog(int isright,QString filename, RTFDocument *doc, QWidget *parent):
    QDialog(parent),
    ui(new Ui::SigalDialog)
{
    ui->setupUi(this);
    this->isright = isright;
    this->doc = doc;
    QStackedLayout *layout = new QStackedLayout(ui->ImageContainer);
    this->label = new ClickLabel(isright,filename,doc,this);
    layout->addWidget(label);
//    /label->setPixmap(QPixmap(filename));
    this->setWindowFlags(Qt::Dialog
                         |Qt::WindowMaximizeButtonHint
                         |Qt::WindowMinimizeButtonHint
                         |Qt::WindowCloseButtonHint);
    this->showMaximized();
    ui->okBut->setEnabled(false);
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
    QString width = ui->widthEdit->text();
    if(width.length()==0){
        QMessageBox::information(this,"Information","Please Input the width of each small square!");
        return ;
    }
    ClickLabel *la = (ClickLabel*)this->label;
    doc->addCorner(isright,la->selectimg,la->corners,width.toDouble());
    this->close();
}

void SigalDialog::onLabelOk(){
    ui->okBut->setEnabled(true);
}
