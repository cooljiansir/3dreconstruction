#ifndef BINOCULARDIALOG_H
#define BINOCULARDIALOG_H

#include <QDialog>

namespace Ui {
class BinocularDialog;
}

class BinocularDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit BinocularDialog(QWidget *parent = 0);
    ~BinocularDialog();
    
private:
    Ui::BinocularDialog *ui;
};

#endif // BINOCULARDIALOG_H
