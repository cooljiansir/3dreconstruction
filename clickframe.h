#ifndef CLICKFRAME_H
#define CLICKFRAME_H

#include <QFrame>

namespace Ui {
class ClickFrame;
}

class ClickFrame : public QFrame
{
    Q_OBJECT

protected:
    void paintEvent(QPaintEvent *);
public:
    explicit ClickFrame(QWidget *parent = 0);
    ~ClickFrame();
    void mouseMoveEvent(QMouseEvent *);
private:
    Ui::ClickFrame *ui;
    int mouse_x;
    int mouse_y;
};

#endif // CLICKFRAME_H
