#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "photodialog.h"
#include "rtfdocument.h"
#include <QFile>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    void setupCalibUI();
    
private slots:
    void on_actionTake_Photoes_triggered();

    void on_actionOpen_file_triggered();

    void on_actionSave_triggered();

    void on_actionSigalCamera_triggered();

    void on_actionNew_triggered();

private:
    Ui::MainWindow *ui;
    RTFDocument *doc;

};

#endif // MAINWINDOW_H
