#ifndef PHOTODIALOG_H
#define PHOTODIALOG_H

#include <QDialog>

namespace Ui {
class PhotoDialog;
}

class PhotoDialog : public QDialog
{
    Q_OBJECT
    
public:
    explicit PhotoDialog(QWidget *parent = 0);
    ~PhotoDialog();
    
private:
    Ui::PhotoDialog *ui;
};

#endif // PHOTODIALOG_H
