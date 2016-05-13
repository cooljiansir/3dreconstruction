#ifndef DIALOGCORNER_H
#define DIALOGCORNER_H

#include <QDialog>


#include <cv.h>
#include <vector>

using namespace cv;
using namespace std;

namespace Ui {
class DialogCorner;
}

class DialogCorner : public QDialog
{
    Q_OBJECT
    
public:
    explicit DialogCorner(Mat &img,vector<Point2f> &corners,QWidget *parent = 0);
    ~DialogCorner();

    void resizeEvent(QResizeEvent *);
    void keyPressEvent(QKeyEvent *);

public slots:
    void onScaleLabelClicked(int x,int y);

private slots:
    void on_changeScale_clicked();

private:
    Ui::DialogCorner *ui;
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

#endif // DIALOGCORNER_H
