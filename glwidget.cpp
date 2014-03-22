#include "glwidget.h"
#include <QDebug>

GLWidget::GLWidget(QWidget *parent) :
    QGLWidget(parent)
{
    //this->mat3d = mat3d;
    //this->mattexture = texture;
}
void GLWidget::setMat(Mat &mat3d, Mat &mattexture){
    this->mat3d = mat3d;
    this->mattexture = mattexture;
}

void GLWidget::initializeGL(){
    glShadeModel(GL_SMOOTH);
    glClearColor(0.0,0.0,0.0,0.0);
    glClearDepth(GL_DEPTH_TEST);
    glDepthFunc(GL_LEQUAL);
    glHint(GL_PERSPECTIVE_CORRECTION_HINT,GL_NICEST);
}


void GLWidget::paintGL(){
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glLoadIdentity();
    glPointSize(2.0);
    glColor3f(1.0,0.0,0.0);
    float mmax = 0,mmay=0,mmaz=0;

    glRotatef(xRot / 16.0f, 1.0f, 0.0f, 0.0f);
    glRotatef(yRot / 16.0f, 0.0f, 1.0f, 0.0f);
    glRotatef(zRot / 16.0f, 0.0f, 0.0f, 1.0f);
    glBegin(GL_POINTS);
        //glVertex3f(0.0,0.0,0.0);
        for(int i = 0;i<this->mat3d.rows;i++){
            for(int j = 0;j<this->mat3d.cols;j++){
                //Point3f p = this->mat3d.at<Point3f>(i,j);
                Point3f *p = (Point3f *)this->mat3d.ptr(i,j);
                if(p->z>0){
//                    Vec3b c = this->mattexture.at<Vec3b>(i,j);
                    Vec3b *c = (Vec3b *)this->mattexture.ptr(i,j);
                    glColor3f((*c)[2]/256.0,(*c)[1]/256.0,(*c)[1]/256.0);
                    glVertex3f(p->x,-p->y,-p->z);
                }
            }
        }
    glEnd();
    glFlush();
}


void GLWidget::resizeGL(int width, int height){
    //int side = qMin(width, height);
    //glViewport((width - side) / 2, (height - side) / 2, side, side);
    if(height==0)
        height = 1;
    glViewport(0,0,width,height);

    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
#ifdef QT_OPENGL_ES_1
    glOrthof(-0.5, +0.5, -0.5, +0.5, 4.0, 15.0);
#else
    glOrtho(-200.0*width/height, +200.0*width/height, -200.0, +200.0, -20000.0, 20000.0);
#endif
    glMatrixMode(GL_MODELVIEW);
}
void GLWidget::mousePressEvent(QMouseEvent *e){
    this->lastPos = e->pos();
}
void GLWidget::mouseMoveEvent(QMouseEvent *event){
    int dx = event->x() - lastPos.x();
    int dy = event->y() - lastPos.y();

    if (event->buttons() & Qt::LeftButton) {
        rotateBy(8 * dy, 8 * dx, 0);
    } else if (event->buttons() & Qt::RightButton) {
        rotateBy(8 * dy, 0, 8 * dx);
    }
    lastPos = event->pos();
}
void GLWidget::rotateBy(int xAngle, int yAngle, int zAngle){
    xRot += xAngle;
    yRot += yAngle;
    zRot += zAngle;
    updateGL();
}
