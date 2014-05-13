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
    void initialForm();
    void updateImage();

public slots:

    void checkForm();
private slots:
    void on_pushButton_refresh_clicked();

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
