#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <cv.h>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT
    
public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
public slots:
    void on_pressed(int x,int y);
private:
    Ui::MainWindow *ui;
    cv::Mat matLeft;
    cv::Mat matLeft2;
    cv::Mat matRight;
    cv::Mat uninMat;
    cv::Mat matLeftLab;
};

#endif // MAINWINDOW_H
