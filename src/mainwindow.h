#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include "photodialog.h"
#include "rtfdocument.h"
#include <QFile>
#include <QMutex>
#include <QStandardItemModel>
#include "glwidget.h"
#include "stereoform.h"

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

    void setupStereoUI();
    
private slots:
    void on_actionTake_Photoes_triggered();

    void on_actionOpen_file_triggered();

    void on_actionSave_triggered();

    void on_actionSigalCamera_triggered();

    void on_actionNew_triggered();

    void on_adddataBut_clicked();

    void on_rightAddData_clicked();

    void on_actionDual_triggered();

    void on_bincularAddBut_clicked();

    void on_actionTake_Photoes_2_triggered();

    void on_actionPolar_Correction_triggered();


    void on_leftUndistored_clicked();

    void on_rightUndistored_clicked();

    void on_action3D_reconstruction_triggered();

    void on_savePolarBut_clicked();

    void on_actionStereoMatch_triggered();
    void on_reproject_AddBut_clicked();

    void on_reproject_SubBut_clicked();

    void on_reproject_XAdd_clicked();

    void on_reproject_XSub_clicked();

    void on_reproject_YSub_clicked();

    void on_reproject_YAdd_clicked();

    void on_reproject_ZSub_clicked();

    void on_reproject_ZAdd_clicked();

private:
    //bool isStereoLoading;//互斥信号量
    QMutex mutex;
public slots:

    void on_polar_Open_clicked();

    void stereoUpate(Mat &left,Mat &right,Mat &dis);//stereoform update its image


public:
    void loadCalidUI();
    void setupBinUI();
    void loadBinUI();
    void keyPressEvent(QKeyEvent *);

    void loadStereoParam(int kind);//根据所选匹配算法加载参数
private:
    Ui::MainWindow *ui;
    QStandardItemModel *singal_model;
    QStandardItemModel *singal_model_r;
    QStandardItemModel *bin_model;
    bool opened;
    //极线矫正、左右图像
    Mat polar_left,polar_right;

    bool polar_changed;//重建图片是否改变
    bool polar_finished;
    Mat dispMat;//差视图
    bool disOk;//
    Mat Q;//Q矩阵

public:
    RTFDocument *doc;

};

#endif // MAINWINDOW_H
