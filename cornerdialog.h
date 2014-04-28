#ifndef CORNERDIALOG_H
#define CORNERDIALOG_H

#include <QDialog>
#include "cv.h"
#include <vector>

using namespace cv;

namespace Ui {
class CornerDialog;
}

class CornerDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit CornerDialog(Mat &img,vector<Point2f> &corners, QWidget *parent = 0);
    ~CornerDialog();
    void wheelEvent(QWheelEvent *);
    void mouseDoubleClickEvent(QMouseEvent *);
    
private:
    Ui::CornerDialog *ui;
    Mat img;
    vector<Point2f> corners;
    double rate;
    void calMat(Mat &showmat, Point c, double prerate);
    int showdx;
    int showdy;
    int winwidth;
    int winheight;
    bool firstload;
};

#endif // CORNERDIALOG_H
