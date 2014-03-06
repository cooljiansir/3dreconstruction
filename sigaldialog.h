#ifndef SIGALDIALOG_H
#define SIGALDIALOG_H

#include <QDialog>
#include "rtfdocument.h"
#include <QLabel>

namespace Ui {
class SigalDialog;
}

class SigalDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit SigalDialog(QString filename,RTFDocument *doc,QWidget *parent = 0);
    ~SigalDialog();
    void setOkButEnable(bool b);

private slots:
    void on_cancelBut_clicked();
    void on_okBut_clicked();

private:
    Ui::SigalDialog *ui;
    QLabel *label;
};

#endif // SIGALDIALOG_H
