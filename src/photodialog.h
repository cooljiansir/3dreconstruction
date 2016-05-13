#ifndef PHOTODIALOG_H
#define PHOTODIALOG_H

#include <QDialog>
#include <cv.h>
#include <highgui.h>
using namespace cv;

namespace Ui {
class PhotoDialog;
}

class PhotoDialog : public QDialog
{
    Q_OBJECT
    
protected:
    void timerEvent(QTimerEvent *);

public:
    explicit PhotoDialog(QWidget *parent = 0);
    ~PhotoDialog();
    //打开下一个摄像头
    void openNext();
    
private slots:
    void on_pushButton_4_clicked();

    void on_printBut_clicked();

    void on_cancelBut_clicked();

private:
    Ui::PhotoDialog *ui;
    int timerInt;
    VideoCapture *capture;
    int capn;//摄像头编号
    Mat imagemat;
    bool isSwitching;
    bool isshot;
};

#endif // PHOTODIALOG_H
