#ifndef CLICKLABEL_H
#define CLICKLABEL_H

#include <QLabel>
#include <vector>
#include "mainwindow.h"


using namespace std;

namespace Ui {
class ClickLabel;
}

class ClickLabel : public QLabel
{
    Q_OBJECT
    
protected:
    void paintEvent(QPaintEvent *);
public:
    explicit ClickLabel(int isright,QString filename,RTFDocument *doc,QWidget *parent = 0);
    ~ClickLabel();
    void mouseMoveEvent(QMouseEvent *);
    void mousePressEvent(QMouseEvent *ev);
    void onOk();
public:
    Mat selectimg;
    vector<vector<Point2f> > corners;
private:
    Ui::ClickLabel *ui;
    int isright;
    int status;
    vector<Point2f> harriscorners;
    QPixmap *pixmap;
    int mouse_x;
    int mouse_y;
    vector<int> mouse_x_v;
    vector<int> mouse_y_v;
    RTFDocument *doc;
    QWidget *father;

static const int STATUS_FINDING = 1;
static const int STATUS_FOUND   = 2;

public:
static const int KIND_SINGAL    = 1;
static const int KIND_BINOCULAR = 2;


signals:
    void ok();

};

#endif // CLICKLABEL_H
