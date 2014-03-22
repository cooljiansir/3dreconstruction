#include "mainwindow.h"
#include "ui_mainwindow.h"
#include <QStandardItemModel>
#include <QFileDialog>
#include <QDir>
#include <QMessageBox>
#include "sigaldialog.h"
#include "binoculardialog.h"
#include <QKeyEvent>
#include "photodialog2.h"
#include "comparedialog.h"
#include "glwidget.h"
#include "uti.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    this->doc = new RTFDocument();
    this->opened = false;
    this->disOk = false;
    this->polar_finished = false;
    ui->actionSave->setEnabled(false);
    ui->actionNew->setEnabled(true);
    ui->actionOpen_file->setEnabled(true);
    ui->actionDual->setEnabled(false);
    ui->actionTake_Photoes->setEnabled(false);
    ui->actionSigalCamera->setEnabled(false);
    ui->action3D_reconstruction->setEnabled(false);
    ui->actionTake_Photoes_2->setEnabled(false);
    ui->actionPolar_Correction->setEnabled(false);
    this->ui->stackedWidget->setCurrentIndex(0);//welcome!
    this->setupCalibUI();
    this->setupBinUI();
}

MainWindow::~MainWindow()
{
    delete ui;
    delete this->doc;
}

//单摄像机标定界面初始化
void MainWindow::setupCalibUI(){

    //显示列表
    singal_model = new QStandardItemModel();
    singal_model->setColumnCount(6);
    this->ui->tableView->setModel(singal_model);
    singal_model->setHeaderData(0,Qt::Horizontal,QString::fromLocal8Bit("Image"));
    singal_model->setHeaderData(1,Qt::Horizontal,QString::fromLocal8Bit("u"));
    singal_model->setHeaderData(2,Qt::Horizontal,QString::fromLocal8Bit("v"));
    singal_model->setHeaderData(3,Qt::Horizontal,QString::fromLocal8Bit("x"));
    singal_model->setHeaderData(4,Qt::Horizontal,QString::fromLocal8Bit("y"));
    singal_model->setHeaderData(5,Qt::Horizontal,QString::fromLocal8Bit("z"));
    singal_model_r = new QStandardItemModel();
    singal_model_r->setColumnCount(6);
    this->ui->tableView_2->setModel(singal_model_r);
    singal_model_r->setHeaderData(0,Qt::Horizontal,QString::fromLocal8Bit("Image"));
    singal_model_r->setHeaderData(1,Qt::Horizontal,QString::fromLocal8Bit("u"));
    singal_model_r->setHeaderData(2,Qt::Horizontal,QString::fromLocal8Bit("v"));
    singal_model_r->setHeaderData(3,Qt::Horizontal,QString::fromLocal8Bit("x"));
    singal_model_r->setHeaderData(4,Qt::Horizontal,QString::fromLocal8Bit("y"));
    singal_model_r->setHeaderData(5,Qt::Horizontal,QString::fromLocal8Bit("z"));
    this->loadCalidUI();
}
void MainWindow::loadCalidUI(){
    vector<vector<Point2f> > image_point;
    vector<vector<Point3f> > object_point;
    this->doc->getCorner(0,image_point,object_point);
    int count = 0;
    for(int i = 0;i<image_point.size();i++){
        for(int j = 0;j<image_point[i].size();j++){
            singal_model->setItem(count,0,new QStandardItem(QString::number(i)));
            singal_model->setItem(count,1,new QStandardItem(QString::number(image_point[i][j].x)));
            singal_model->setItem(count,2,new QStandardItem(QString::number(image_point[i][j].y)));
            singal_model->setItem(count,3,new QStandardItem(QString::number(object_point[i][j].x)));
            singal_model->setItem(count,4,new QStandardItem(QString::number(object_point[i][j].y)));
            singal_model->setItem(count,5,new QStandardItem(QString::number(object_point[i][j].z)));
            count++;
        }
    }
    this->doc->getCorner(1,image_point,object_point);
    count = 0;
    for(int i = 0;i<image_point.size();i++){
        for(int j = 0;j<image_point[i].size();j++){
            singal_model_r->setItem(count,0,new QStandardItem(QString::number(i)));
            singal_model_r->setItem(count,1,new QStandardItem(QString::number(image_point[i][j].x)));
            singal_model_r->setItem(count,2,new QStandardItem(QString::number(image_point[i][j].y)));
            singal_model_r->setItem(count,3,new QStandardItem(QString::number(object_point[i][j].x)));
            singal_model_r->setItem(count,4,new QStandardItem(QString::number(object_point[i][j].y)));
            singal_model_r->setItem(count,5,new QStandardItem(QString::number(object_point[i][j].z)));
            count++;
        }
    }
    Mat l_intrinsic;
    Mat l_distortion;
    Mat r_intrinsic;
    Mat r_distortion;
    if(this->doc->getLintrisic(l_intrinsic)){
        QString str="";
        for(int i = 0;i<l_intrinsic.rows;i++) {
            for(int j = 0;j<l_intrinsic.cols;j++){
                QString s = QString::number(l_intrinsic.at<double>(i,j));
                str += s;
                str += QString(20-s.length(),' ');
                str += '\r';
            }
            str +='\n';
        }
        this->ui->l_IntrinLabel->setText(str);
    }
    if(this->doc->getLdistortion(l_distortion)){
        QString str="";
        for(int i = 0;i<l_distortion.rows;i++) {
            for(int j = 0;j<l_distortion.cols;j++){
                QString s = QString::number(l_distortion.at<double>(i,j));
                str += s;
                str += QString(20-s.length(),' ');
                str += '\r';
            }
            str +='\n';
        }
        this->ui->l_DistorLabel->setText(str);
    }
    if(this->doc->getRintrinsic(r_intrinsic)){
        QString str="";
        for(int i = 0;i<3;i++) {
            for(int j = 0;j<3;j++){
                QString s = QString::number(r_intrinsic.at<double>(i,j));
                str += s;
                str += QString(20-s.length(),' ');
                str += '\r';
            }
            str +='\n';
        }
        this->ui->r_IntrinLabel->setText(str);
    }
    if(this->doc->getRdistortion(r_distortion)){
        QString str="";
        for(int i = 0;i<r_distortion.rows;i++) {
            for(int j = 0;j<r_distortion.cols;j++){
                QString s = QString::number(r_distortion.at<double>(i,j));
                str += s;
                str += QString(20-s.length(),' ');
                str += '\r';
            }
            str +='\n';
        }
        this->ui->r_DistorLabel->setText(str);
    }
}

//截屏
void MainWindow::on_actionTake_Photoes_triggered()
{
    PhotoDialog  dig(this);
    dig.exec();
}
//打开文件
void MainWindow::on_actionOpen_file_triggered()
{
    QString filename = QFileDialog::getOpenFileName(
       this,
       "Open Document",
      NULL,// QDir::currentPath(),
                QString("Document files (*.")+FILE_NAME+");;All files(*.*)");
    if (!filename.isNull()) { //用户选择了文件
          this->setWindowTitle(filename.mid(filename.lastIndexOf("/")+1));
          ui->actionOpen_file->setEnabled(false);
          ui->actionNew->setEnabled(false);
          ui->actionSave->setEnabled(true);

          ui->action3D_reconstruction->setEnabled(true);
          ui->actionDual->setEnabled(true);
          ui->actionPolar_Correction->setEnabled(true);
          ui->actionSigalCamera->setEnabled(true);
          ui->actionTake_Photoes->setEnabled(true);
          ui->actionTake_Photoes_2->setEnabled(true);

          //ui->stackedWidget->setCurrentIndex(1);
          this->doc->read(filename);
          this->on_actionSigalCamera_triggered();
          this->opened = true;
    } else{ // 用户取消选择

    }
}
//保存文件
void MainWindow::on_actionSave_triggered()
{
    if(this->doc->opened()){
        this->doc->save();
    }else{
        QString fileName = QFileDialog::getSaveFileName(
                    this,
                    tr("Save File"),
                    QString("untitled.")+FILE_NAME,
                    QString("data files (*.")+FILE_NAME+")");
        if(!fileName.isNull()){
            this->setWindowTitle(fileName.mid(fileName.lastIndexOf("/")+1));
            ui->statusBar->showMessage("正在保存");
            this->doc->write(fileName);
            ui->statusBar->showMessage("保存完毕");
        }
    }
}
//
void MainWindow::on_actionSigalCamera_triggered()
{
    ui->stackedWidget->setCurrentIndex(1);
    //setupCalibUI();
    this->loadCalidUI();
}
//新建文档
void MainWindow::on_actionNew_triggered()
{
    ui->actionOpen_file->setEnabled(false);
    ui->actionSave->setEnabled(true);
    ui->actionNew->setEnabled(false);

    ui->action3D_reconstruction->setEnabled(true);
    ui->actionDual->setEnabled(true);
    ui->actionPolar_Correction->setEnabled(true);
    ui->actionSigalCamera->setEnabled(true);
    ui->actionTake_Photoes->setEnabled(true);
    ui->actionTake_Photoes_2->setEnabled(true);

    this->setWindowTitle("untitiled");
    this->opened = true;
    this->on_actionSigalCamera_triggered();
}

void MainWindow::on_adddataBut_clicked()
{
    QString filename = QFileDialog::getOpenFileName(
       this,
       "Open Document",
       NULL,//QDir::currentPath(),
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!filename.isNull()) { //用户选择了文件
          //this->doc->selectBegin(filename.toStdString());
          SigalDialog dig(0,filename,this->doc,this);
          dig.setWindowTitle(filename.mid(filename.lastIndexOf("/")+1));
          dig.exec();
          this->loadCalidUI();
    }
}


void MainWindow::on_rightAddData_clicked()
{
    QString filename = QFileDialog::getOpenFileName(
       this,
       "Open Document",
       NULL,//QDir::currentPath(),
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!filename.isNull()) { //用户选择了文件
          //this->doc->selectBegin(1,filename.toStdString());
          SigalDialog dig(1,filename,this->doc,this);
          dig.setWindowTitle(filename.mid(filename.lastIndexOf("/")+1));
          dig.exec();
          this->loadCalidUI();
    }
}

void MainWindow::on_actionDual_triggered()
{
    Mat l,r;
    if(!doc->getLintrisic(l)||!doc->getRintrinsic(r)){
        QMessageBox::information(this,"Information","You haven't done Singal Camera Calibration Yet");
    }
    ui->stackedWidget->setCurrentIndex(2);
    this->loadBinUI();
}

void MainWindow::on_bincularAddBut_clicked()
{
    QString leftfilename = QFileDialog::getOpenFileName(
       this,
       "Binocular Calibration - Open Left Image",
       NULL,//QDir::currentPath(),
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!leftfilename.isNull()) { //用户选择了左图文件
        QString rightfilename = QFileDialog::getOpenFileName(
           this,
           "Binocular Calibration - Open Right Image",
           NULL,//QDir::currentPath(),
           "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
        if (!rightfilename.isNull()) { //用户选择了右图文件
            BinocularDialog dig(leftfilename,rightfilename,this->doc,this);
            dig.exec();
            loadBinUI();
        }
    }
}
void MainWindow::setupBinUI(){
    bin_model = new QStandardItemModel();
    bin_model->setColumnCount(8);
    ui->binTable->setModel(bin_model);
    bin_model->setHeaderData(0,Qt::Horizontal,QString::fromLocal8Bit("Image"));
    bin_model->setHeaderData(1,Qt::Horizontal,QString::fromLocal8Bit("u_l"));
    bin_model->setHeaderData(2,Qt::Horizontal,QString::fromLocal8Bit("v_l"));
    bin_model->setHeaderData(3,Qt::Horizontal,QString::fromLocal8Bit("u_r"));
    bin_model->setHeaderData(4,Qt::Horizontal,QString::fromLocal8Bit("v_r"));
    bin_model->setHeaderData(5,Qt::Horizontal,QString::fromLocal8Bit("x"));
    bin_model->setHeaderData(6,Qt::Horizontal,QString::fromLocal8Bit("y"));
    bin_model->setHeaderData(7,Qt::Horizontal,QString::fromLocal8Bit("z"));
}

void MainWindow::loadBinUI(){
    vector<vector<Point2f> > image_point_l;
    vector<vector<Point2f> > image_point_r;
    vector<vector<Point3f> > object_point;
    this->doc->getBinData(image_point_l,image_point_r,object_point);
    int count = 0;
    for(int i = 0;i<image_point_l.size();i++){
        for(int j = 0;j<image_point_l[i].size();j++){
            bin_model->setItem(count,0,new QStandardItem(QString::number(i)));
            bin_model->setItem(count,1,new QStandardItem(QString::number(image_point_l[i][j].x)));
            bin_model->setItem(count,2,new QStandardItem(QString::number(image_point_l[i][j].y)));
            bin_model->setItem(count,3,new QStandardItem(QString::number(image_point_r[i][j].x)));
            bin_model->setItem(count,4,new QStandardItem(QString::number(image_point_r[i][j].y)));
            bin_model->setItem(count,5,new QStandardItem(QString::number(object_point[i][j].x)));
            bin_model->setItem(count,6,new QStandardItem(QString::number(object_point[i][j].y)));
            bin_model->setItem(count,7,new QStandardItem(QString::number(object_point[i][j].z)));
            count++;
        }
    }
    QString str = "";
    QString str1 = "";
    Mat R,T;
    if(doc->getBinR(R)){
        for(int i = 0;i<R.rows;i++){
            for(int j = 0;j<R.cols;j++){
                QString t = QString::number(R.at<double>(i,j));
                str += t;
                str += QString(20-t.length(),' ');
            }
            str += '\n';
        }
    }
    if(doc->getBinT(T)){
        for(int i = 0;i<T.rows;i++){
            for(int j = 0;j<T.cols;j++){
                QString t = QString::number(T.at<double>(i,j));
                str1 += t;
                str1 += QString(20-t.length(),' ');
            }
            str1 += '\n';
        }
    }
    ui->bin_Rlabel->setText(str);
    ui->bin_Tlabel->setText(str1);
}
void MainWindow::keyPressEvent(QKeyEvent *e){
    int ukey = e->key();
    Qt::Key key = static_cast<Qt::Key>(ukey);
    Qt::KeyboardModifiers modifiers = e->modifiers();
    if(key==Qt::Key_S&&key){
        if(modifiers&Qt::ControlModifier){
            if(this->opened)
                on_actionSave_triggered();
        }
    }
}

void MainWindow::on_actionTake_Photoes_2_triggered()
{
    PhotoDialog2 dig(this);
    dig.exec();
}

void MainWindow::on_actionPolar_Correction_triggered()
{
    ui->stackedWidget->setCurrentIndex(3);
}


void MainWindow::on_leftUndistored_clicked()
{
    Mat l_ins,l_dis;
    if(!this->doc->getLintrisic(l_ins)||!this->doc->getLdistortion(l_dis)){
        QMessageBox::information(this,"Error","You haven't done calibration yet");
        return;
    }
    QString leftfilename = QFileDialog::getOpenFileName(
       this,
       "Binocular Calibration - Open Left Image",
       NULL,//QDir::currentPath(),
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!leftfilename.isNull()) { //用户选择了左图文件
        Mat mat = imread(leftfilename.toStdString().c_str());
        Mat mapx,mapy;
        initUndistortRectifyMap(l_ins,l_dis,Mat(),l_ins,mat.size(),CV_32FC1,mapx,mapy);
        Mat mat_;
        remap(mat,mat_,mapx,mapy,INTER_LINEAR);
        CompareDialog dig(mat,mat_,this);
        dig.exec();
    }
}

void MainWindow::on_rightUndistored_clicked()
{
    Mat r_ins,r_dis;
    if(!this->doc->getRintrinsic(r_ins)||!this->doc->getRdistortion(r_dis)){
        QMessageBox::information(this,"Error","You haven't done calibration yet");
        return;
    }
    QString filename = QFileDialog::getOpenFileName(
       this,
       "Binocular Calibration - Open Right Image",
       NULL,//QDir::currentPath(),
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!filename.isNull()) { //用户选择了左图文件
        Mat mat = imread(filename.toStdString().c_str());
        Mat mapx,mapy;
        initUndistortRectifyMap(r_ins,r_dis,Mat(),r_ins,mat.size(),CV_32FC1,mapx,mapy);
        Mat mat_;
        remap(mat,mat_,mapx,mapy,INTER_LINEAR);
        CompareDialog dig(mat,mat_,this);
        dig.exec();
    }
}

void MainWindow::on_action3D_reconstruction_triggered()
{
    if(!this->doc->isPolarOk()){
        QMessageBox::information(this,"ERROR","You hadn't done binocular calibration yet!");
        return;
    }
    if(!this->disOk){
        QMessageBox::information(this,"ERROR","Please Select two images in Stereo Match Menu");
        return;
    }
    Mat img3D;
    this->doc->reproject3D(this->dispMat,img3D,this->Q);
    this->ui->glWidget->setMat(img3D,this->polar_left);
    this->ui->stackedWidget->setCurrentIndex(5);
}

void MainWindow::on_savePolarBut_clicked()
{
    QString fileNameLeft = QFileDialog::getSaveFileName(
                this,
                tr("Save Left File"),
                NULL,
                "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if(!fileNameLeft.isNull()){
        QString fileNameRight = QFileDialog::getSaveFileName(
                    this,
                    tr("Save Right File"),
                    NULL,
                    "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
        if(!fileNameRight.isNull()){
            imwrite(fileNameLeft.toUtf8().data(),this->polar_left);
            imwrite(fileNameRight.toUtf8().data(),this->polar_right);
        }
    }
}

void MainWindow::on_actionStereoMatch_triggered()
{
    this->ui->stackedWidget->setCurrentIndex(4);
    on_stereoMatchLoadBut_clicked();
}

void MainWindow::on_stereoMatchLoadBut_clicked()
{
    if(!this->polar_finished){
        QMessageBox::information(this,"ERROR","Please Select two images in Polar Correction Menu");
        return;
    }

    QPixmap left = QPixmap::fromImage(Mat2QImage(this->polar_left));
    QPixmap right = QPixmap::fromImage(Mat2QImage(this->polar_right));
    ui->stereoMatchLeftLabel->setPixmap(left.scaled(ui->stereoMatchLeftLabel->width()
                                                    ,ui->stereoMatchLeftLabel->height()));

    ui->stereoMatchRightLabel->setPixmap(right.scaled(ui->stereoMatchRightLabel->width()
                                                      ,ui->stereoMatchRightLabel->height()));
    //Mat leftM = imread(fileNameLeft.toUtf8().data());
    //Mat rightM = imread(fileNameRight.toUtf8().data());
    Mat vdisp,vdisp3;
    this->doc->stereoMatch(this->polar_left,this->polar_right,this->dispMat);
    this->disOk = true;
    this->dispMat.convertTo(vdisp,CV_8U);
    cvtColor(vdisp,vdisp3,CV_GRAY2BGR);
    QPixmap re = QPixmap::fromImage(Mat2QImage(vdisp3));
    ui->stereoMatchResultLabel->setPixmap(re.scaled(ui->stereoMatchResultLabel->width()
                                                    ,ui->stereoMatchResultLabel->height()));
}

void MainWindow::on_polar_Open_clicked()
{
    QString leftfilename = QFileDialog::getOpenFileName(
       this,
       "Binocular Calibration - Open Left Image",
       NULL,//QDir::currentPath(),
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!leftfilename.isNull()) { //用户选择了左图文件
        QString rightfilename = QFileDialog::getOpenFileName(
           this,
           "Binocular Calibration - Open Right Image",
           NULL,//QDir::currentPath(),
           "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
        if (!rightfilename.isNull()) { //用户选择了右图文件
            Mat leftmat = imread(leftfilename.toUtf8().data());
            Mat rightmat = imread(rightfilename.toUtf8().data());
            cv::resize(rightmat,rightmat,leftmat.size());
            Mat maplx,maply,maprx,mapry;
            this->doc->getPolarParam(maplx,maply,maprx,mapry,Q,leftmat.size());
            Mat leftmat_,rightmat_;
            remap(leftmat,leftmat_,maplx,maply,INTER_LINEAR);
            remap(rightmat,rightmat_,maprx,mapry,INTER_LINEAR);
            this->polar_left = leftmat_;
            this->polar_right = rightmat_;
            this->polar_finished = true;
            Mat united;
            united.create(leftmat_.rows,leftmat_.cols+rightmat_.cols,leftmat.type());
            Mat unitedl = united(Rect(0,0,leftmat_.cols,leftmat_.rows));
            Mat unitedr = united(Rect(leftmat_.cols,0,rightmat_.cols,united.rows));
            leftmat_.copyTo(unitedl);
            rightmat_.copyTo(unitedr);
            for(int i = 0;i<10;i++){
                int temp = united.rows*i/10;
                if(temp>=united.rows)temp = united.rows;
                line(united,Point(0,temp),Point(united.cols,temp),Scalar(0,0,255));
            }
            this->ui->imgLabel->setPixmap(QPixmap::fromImage(Mat2QImage(united)));
        }
    }
    return;
}
