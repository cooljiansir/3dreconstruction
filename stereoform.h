#ifndef STEREOFORM_H
#define STEREOFORM_H

#include <QWidget>
#include <cv.h>
#include "stereo.h"

using namespace cv;

namespace Ui {
class StereoForm;
}

class StereoForm : public QWidget
{
    Q_OBJECT
    
public:
    explicit StereoForm(Mat &leftmat,Mat &rightmat,QWidget *parent = 0);
    ~StereoForm();
    void resizeEvent(QResizeEvent *);
    
private:
    Ui::StereoForm *ui;
    void resizeMat(Mat &src,Mat &res,int width,int height);


private:
    Mat leftmat;
    Mat rightmat;
    Mat dismat;
    Stereo stereo;

};

#endif // STEREOFORM_H
