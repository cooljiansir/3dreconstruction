#ifndef BINOCULARDIALOG_H
#define BINOCULARDIALOG_H

#include <QDialog>
#include "rtfdocument.h"
#include <QLabel>

namespace Ui {
class BinocularDialog;
}

class BinocularDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit BinocularDialog(QString filenamel,QString filenamer,RTFDocument *doc,QWidget *parent = 0);
    ~BinocularDialog();
    
private slots:
    void on_okBut_clicked();

    void on_cancelBut_clicked();

    void onLabelOk();

private:
    Ui::BinocularDialog *ui;
    RTFDocument *doc;
    int countok;
    QLabel *labell;
    QLabel *labelr;

};

#endif // BINOCULARDIALOG_H
