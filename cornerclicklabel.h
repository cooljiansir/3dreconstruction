#ifndef CORNERCLICKLABEL_H
#define CORNERCLICKLABEL_H

#include <QLabel>

class CornerClickLabel : public QLabel
{
    Q_OBJECT
public:
    explicit CornerClickLabel(QWidget *parent = 0);

    void mousePressEvent(QMouseEvent *ev);

signals:
    void mousePressed(int x,int y);
    
};

#endif // CORNERCLICKLABEL_H
