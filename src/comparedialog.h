#ifndef COMPAREDIALOG_H
#define COMPAREDIALOG_H

#include <QDialog>
#include "cv.h"
using namespace cv;

namespace Ui {
class CompareDialog;
}

class CompareDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit CompareDialog(Mat &leftMat,Mat &rightMat,QWidget *parent = 0);
    ~CompareDialog();
    
private slots:
    void on_pushButton_clicked();

private:
    Ui::CompareDialog *ui;
};

#endif // COMPAREDIALOG_H
