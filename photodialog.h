#ifndef PHOTODIALOG_H
#define PHOTODIALOG_H

#include <QDialog>
#include <cv.h>
#include <vector>
#include "corner.h"

using namespace cv;
using namespace std;

namespace Ui {
class PhotoDialog;
}

class PhotoDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit PhotoDialog(QString filename,QWidget *parent = 0);
    ~PhotoDialog();

    void resizeEvent(QResizeEvent *);
    void keyPressEvent(QKeyEvent *);
public slots:
    void onScaleLabelClicked(int x,int y);
    
private slots:
    void on_pushButton_clicked();

    void on_radioButMoravec_clicked();

    void on_radioButHarris_clicked();

    void on_radioButNobel_clicked();

    void on_radioButShiTomasi_clicked();

    void on_radioButFitting_clicked();

    void on_radioButVector_clicked();

    void on_changePraBut_clicked();

private:
    Ui::PhotoDialog *ui;
    QString filename;
    Mat img;
    vector<Point2f> corners;
    void scaledTo(Mat &src, Mat &res, int w, int h);

    double smallrate;
    int bigrate;
    int smallx,smally;
    int smallwinwidth,smallwinheight;
    void showBig();
    void showSmall();
    void loadMat();

    Corner corn;

};

#endif // PHOTODIALOG_H
