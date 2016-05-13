#ifndef PHOTODIALOG2_H
#define PHOTODIALOG2_H

#include <QDialog>
#include <cv.h>
#include <QTimer>
#include <highgui.h>

using namespace cv;

namespace Ui {
class PhotoDialog2;
}

class PhotoDialog2 : public QDialog
{
    Q_OBJECT
    
protected:
    void timerEvent(QTimerEvent *);
public:
    explicit PhotoDialog2(QWidget *parent = 0);
    ~PhotoDialog2();
    
private slots:
    void on_pushButton_clicked();

    void on_cancelBut_clicked();

    void on_printBut_clicked();

    void on_save();

private:
    Ui::PhotoDialog2 *ui;
    int timerInt;
    VideoCapture *capleft;
    Mat leftMat;
    int capl;
    VideoCapture *capright;
    Mat rightMat;
    int capr;
    bool isswitch;
    bool isshot;
    QTimer *timer;
};

#endif // PHOTODIALOG2_H
