#ifndef SIGALDIALOG_H
#define SIGALDIALOG_H

#include <QDialog>

namespace Ui {
class SigalDialog;
}

class SigalDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit SigalDialog(QString filename,QWidget *parent = 0);
    ~SigalDialog();

private slots:
    void on_cancelBut_clicked();

private:
    Ui::SigalDialog *ui;
    QWidget *parent;
};

#endif // SIGALDIALOG_H
