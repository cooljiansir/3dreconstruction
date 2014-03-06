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
    explicit ClickLabel(QString filename,RTFDocument *doc,QWidget *parent = 0);
    ~ClickLabel();
    void mouseMoveEvent(QMouseEvent *);
    void mousePressEvent(QMouseEvent *ev);
    void onOk();
private:
    Ui::ClickLabel *ui;
    int status;
    QPixmap *pixmap;
    int mouse_x;
    int mouse_y;
    vector<int> mouse_x_v;
    vector<int> mouse_y_v;
    RTFDocument *doc;
    QWidget *father;
    vector<vector<Point2f> > corners;
static const int STATUS_FINDING = 1;
static const int STATUS_FOUND   = 2;
};

#endif // CLICKLABEL_H
