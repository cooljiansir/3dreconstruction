#ifndef PHOTODIALOG_H
#define PHOTODIALOG_H

#include <QDialog>
#include <cv.h>
#include <vector>

using namespace cv;
using namespace std;

namespace Ui {
class PhotoDialog;
}

class PhotoDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit PhotoDialog(Mat &img,vector<Point2f> &corners,QWidget *parent = 0);
    ~PhotoDialog();

    void resizeEvent(QResizeEvent *);
    void keyPressEvent(QKeyEvent *);
public slots:
    void onScaleLabelClicked(int x,int y);
    
private slots:
    void on_pushButton_clicked();

private:
    Ui::PhotoDialog *ui;
    Mat img;
    vector<Point2f> corners;
    void scaledTo(Mat &src, Mat &res, int w, int h);

    double smallrate;
    int bigrate;
    int smallx,smally;
    int smallwinwidth,smallwinheight;
    void showBig();
    void showSmall();
};

#endif // PHOTODIALOG_H
