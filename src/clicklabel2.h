#ifndef CLICKLABEL2_H
#define CLICKLABEL2_H

#include <QLabel>

namespace Ui {
class ClickLabel2;
}

class ClickLabel2 : public QLabel
{
    Q_OBJECT
    
public:
    explicit ClickLabel2(QWidget *parent = 0);
    ~ClickLabel2();
    
private:
    Ui::ClickLabel2 *ui;
};

#endif // CLICKLABEL2_H
