#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "photodialog.h"
#include "rtfdocument.h"
#include <QFile>
#include <QStandardItemModel>

#define FILE_NAME   "dre"   //文件后缀名

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

    void on_adddataBut_clicked();
    void on_pushButton_clicked();

    void on_rightAddData_clicked();

    void on_actionDual_triggered();

    void on_bincularAddBut_clicked();

public:
    void loadCalidUI();
    void setupBinUI();
    void loadBinUI();
private:
    Ui::MainWindow *ui;
    QStandardItemModel *singal_model;
    QStandardItemModel *singal_model_r;
    QStandardItemModel *bin_model;
public:
    RTFDocument *doc;

};

#endif // MAINWINDOW_H
