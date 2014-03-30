#ifndef CLICKLABEL_H
#define CLICKLABEL_H

#include <QLabel>

namespace Ui {
class ClickLabel;
}

class ClickLabel : public QLabel
{
    Q_OBJECT
protected:
    void mousePressEvent(QMouseEvent *ev);
public:
    explicit ClickLabel(QWidget *parent = 0);
    ~ClickLabel();
signals:
    void on_pressed(int x,int y);
    
private:
    Ui::ClickLabel *ui;
};

#endif // CLICKLABEL_H
