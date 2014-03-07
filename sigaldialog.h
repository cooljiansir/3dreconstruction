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
    explicit SigalDialog(int isright, QString filename, RTFDocument *doc, QWidget *parent = 0);
    ~SigalDialog();

private slots:
    void on_cancelBut_clicked();
    void on_okBut_clicked();
    void onLabelOk();

private:
    Ui::SigalDialog *ui;
    QLabel *label;
    int isright;
    RTFDocument *doc;
};

#endif // SIGALDIALOG_H
