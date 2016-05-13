#ifndef GLWIDGET_H
#define GLWIDGET_H

#include <QGLWidget>
#include <QMouseEvent>
#include "cv.h"

using namespace cv;

class GLWidget : public QGLWidget
{
    Q_OBJECT
protected:
    void initializeGL();
    void paintGL();
    void resizeGL(int w, int height);


    void mousePressEvent(QMouseEvent *);
    void mouseMoveEvent(QMouseEvent *);

public:
    explicit GLWidget(QWidget *parent = 0);
    void setMat(Mat &mat3d,Mat &mattexture);
    
    void rotateBy(int xAngle, int yAngle, int zAngle);

    void translateBy(int x,int y,int z);
    void scaledBy(double scale);
signals:
    
public slots:
private:
    Mat mat3d;
    Mat mattexture;
    QPoint lastPos;
    int xRot;
    int yRot;
    int zRot;
    int xTr;
    int yTr;
    int zTr;
    double scale;
};

#endif // GLWIDGET_H
