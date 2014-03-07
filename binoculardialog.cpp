#include "binoculardialog.h"
#include "ui_binoculardialog.h"
#include <QStackedLayout>
#include <clicklabel.h>

BinocularDialog::BinocularDialog(QString filenamel, QString filenamer, RTFDocument *doc, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::BinocularDialog)
{
    ui->setupUi(this);
    this->countok = 0;
    ui->okBut->setEnabled(false);
    QStackedLayout *layout = new QStackedLayout(ui->lImageContainer);
    this->labell = new ClickLabel(0,filenamel,doc,this);
    label->setPixmap(QPixmap(filenamel));
    layout->addWidget(label);
    QStackedLayout *layout2 = new QStackedLayout(ui->rImageContainer);
    this->labelr = new ClickLabel(1,filenamer,doc,this);
    label2->setPixmap(QPixmap(filenamer));
    layout2->addWidget(label2);
    this->setWindowFlags(Qt::Dialog
                         |Qt::WindowMaximizeButtonHint
                         |Qt::WindowMinimizeButtonHint
                         |Qt::WindowCloseButtonHint);
    this->showMaximized();
}

BinocularDialog::~BinocularDialog()
{
    delete ui;
}

void BinocularDialog::on_okBut_clicked()
{

}

void BinocularDialog::on_cancelBut_clicked()
{
    this->close();
}
void BinocularDialog::onLabelOk(){
    this->countok++;
    if(this->countok==2){
        ui->okBut->setEnabled(true);
    }
}
