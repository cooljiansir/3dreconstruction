#ifndef CLICKLABEL_H
#define CLICKLABEL_H


#include <QLabel>

class ClickLabel : public QLabel
{
    Q_OBJECT
public:
    explicit ClickLabel(QWidget *parent = 0);
    void mousePressEvent(QMouseEvent *ev);
    
signals:
    void mousePressed(int x,int y);
public slots:
    
};

#endif // CLICKLABEL_H
