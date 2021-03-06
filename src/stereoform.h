#ifndef STEREOFORM_H
#define STEREOFORM_H

#include <QWidget>
#include <cv.h>
#include "stereo.h"
#include <QMutex>

using namespace cv;

namespace Ui {
class StereoForm;
}

class StereoForm : public QWidget
{
    Q_OBJECT
    
public:
    explicit StereoForm(QWidget *parent = 0);
    ~StereoForm();
    void resizeEvent(QResizeEvent *);
    void initialForm();
    void updateImage();

    void setImage(Mat &left,Mat &right,Mat &dis);

signals:
    void updateImages(Mat &left,Mat &right,Mat &dis);
public slots:

    void checkForm();
private slots:
    void on_pushButton_refresh_clicked();

    void on_pushButtonSave_clicked();

    void on_pushButton_Open_clicked();

private:
    Ui::StereoForm *ui;
    void resizeMat(Mat &src,Mat &res,int width,int height);

    QMutex mutex;


private:
    Mat leftmat;
    Mat rightmat;
    Mat dismat;
    Stereo stereo;

};

#endif // STEREOFORM_H
