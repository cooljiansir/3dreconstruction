#include "binoculardialog.h"
#include "ui_binoculardialog.h"
#include <QStackedLayout>
#include <clicklabel.h>
#include <QMessageBox>

BinocularDialog::BinocularDialog(QString filenamel, QString filenamer, RTFDocument *doc, QWidget *parent) :
    QDialog(parent),
    ui(new Ui::BinocularDialog)
{
    ui->setupUi(this);
    this->countok = 0;
    this->doc = doc;
    ui->okBut->setEnabled(false);
    QStackedLayout *layout = new QStackedLayout(ui->lImageContainer);
    this->labell = new ClickLabel(0,filenamel,doc,this);
    labell->setPixmap(QPixmap(filenamel));
    layout->addWidget(labell);
    QStackedLayout *layout2 = new QStackedLayout(ui->rImageContainer);
    this->labelr = new ClickLabel(1,filenamer,doc,this);
    labelr->setPixmap(QPixmap(filenamer));
    layout2->addWidget(labelr);
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
    ClickLabel *lal = (ClickLabel*)this->labell;
    ClickLabel *lar = (ClickLabel*)this->labelr;
    if(lal->corners.size()!=lar->corners.size()){
        QMessageBox::information(this,"Information","Left rectangle is not the same as right!");
        return ;
    }
    if(lal->corners.size()>0){
        if(lal->corners[0].size()!=lar->corners[0].size()){
            QMessageBox::information(this,"Information","Left rectangle is not the same as right!");
            return ;
        }
    }
    QString width = ui->widthEdit->text();
    if(width.length()==0){
        QMessageBox::information(this,"Information","Please Input the width of each small square!");
        return ;
    }

    this->doc->addBinData(lal->selectimg,lal->corners,lar->corners,width.toDouble());
    this->close();
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
