#ifndef IMAGELABEL_H
#define IMAGELABEL_H

#include <QLabel>

namespace Ui {
class ImageLabel;
}

class ImageLabel : public QLabel
{
    Q_OBJECT
    
protected:
    void paintEvent(QPaintEvent *);
public:
    explicit ImageLabel(QString filename,QWidget *parent = 0);
    ~ImageLabel();
    void mouseMoveEvent(QMouseEvent *ev);
private:
    Ui::ImageLabel *ui;
    QPixmap *pixmap;
    int status;
    int mouse_x;
    int mouse_y;
static const int STATUS_FINDING = 1;
static const int STATUS_FOUND = 1;
};

#endif // IMAGELABEL_H
