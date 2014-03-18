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
    explicit GLWidget(Mat &mat3d,QWidget *parent = 0);
    
    void rotateBy(int xAngle, int yAngle, int zAngle);
signals:
    
public slots:
private:
    Mat mat3d;
    QPoint lastPos;
    int xRot;
    int yRot;
    int zRot;
};

#endif // GLWIDGET_H
