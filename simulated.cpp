#include <stdio.h>
#include <stdlib.h>
#include "cv.h"
#include <math.h>
#include <QString>
#include <QFileDialog>
#include "highgui.h"
#include <QDebug>
#include <QApplication>


using namespace cv;

#define RAND01 (rand()/double(RAND_MAX))


void stereo_simulate(Mat &left,Mat &right,Mat &dis,int dismax){
    if(left.size()!=right.size())
        return;
    Mat leftgray,rightgray;
    Size size = left.size();
    leftgray.create(size,CV_8U);
    rightgray.create(size,CV_8U);
    cvtColor(left,leftgray,CV_BGR2GRAY);
    cvtColor(right,rightgray,CV_BGR2GRAY);


    unsigned char *leftptr = leftgray.data;
    unsigned char *rightptr = rightgray.data;
    dis.create(size,CV_32F);
    float *disptr = (float*)dis.data;

    double T = 200000;
    double Tend = 0.001;
    double Tk = 0.995;
    int P1 = 80;
    int P2 = 320;

    int *disint =  new int[size.width*size.height];

    //initalborder
    for(int i = 0;i<size.width;i++)
        disint[i] = disint[(size.height-1)*size.width+i] = -1;
    for(int i = 0;i<size.height;i++)
        disint[i*size.width] = disint[i*size.width+size.width-1] = -1;

    //inital state
    for(int i = 0;i<size.height;i++){
        for(int j = 0;j<size.width;j++){
            disint[i*size.width+j] = rand()%dismax;
        }
    }

    if(0){
    //use sgbm to inital
    int SADWindowSize = 7;
    int numberOfDisparities = 16*6;
    StereoSGBM sgbm;
    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

    int cn = 1;//leftgray.channels();

    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = numberOfDisparities;
    sgbm.uniquenessRatio = 10;
    sgbm.speckleWindowSize = 100;
    sgbm.speckleRange = 32;
    sgbm.disp12MaxDiff = 1;
    sgbm.fullDP = true;

    Mat disp;
    sgbm(leftgray,rightgray,disp);
    short int *sgbptr = (short int*)disp.data;
    for(int i = 0;i<size.height;i++){
        for(int j = 0;j<size.width;j++){
            disint[i*size.width+j] = sgbptr[i*size.width+j]/16;
        }
    }
    }

    while((T*=Tk)>Tend){
        for(int i = 1;i+1<size.height;i++){
            int *disinti = disint+i*size.width;
            unsigned char *leftptri = leftptr + i*size.width;
            //unsigned char *leftptri_p = leftptri - size.width;
            //unsigned char *leftptri_n = leftptri + size.width;
            unsigned char *rightptri = rightptr + i*size.width;
            //unsigned char *rightptri_p = rightptri - size.width;
            //unsigned char *rightptri_n = rightptri + size.width;
            int *disinti_p = disinti-size.width;
            int *disinti_n = disinti+size.width;
            for(int j = 1;j+1<size.width;j++){
                double delta=0;
                int d = disinti[j];
                int f=0;
                if(rand()>=(RAND_MAX>>1)){
                    if(d+1<dismax&&d<j){
                        f =1;
                    }
                }else{
                    if(d>0){
                        f =-1;
                    }
                }
                if(f!=0){
                    //delta Edata
                    delta = (leftptri[j]- rightptri[j-d-f])*(leftptri[j]- rightptri[j-d-f])-
                            (leftptri[j]- rightptri[j-d])*(leftptri[j]- rightptri[j-d]);
                    //delta Esmooth
                    int pre;
                    int no;
                    //up
                    if(abs(disinti_p[j]-d)>0){
                        if(abs(disinti_p[j]-d)==1){
                            pre = P1;
                        }else{
                            pre = P2;
                        }
                    }else {
                        pre = 0;
                    }
                    if(abs(disinti_p[j]-d-f)>0){
                        if(abs(disinti_p[j]-d-f)==1){
                            no = P1;
                        }else{
                            no = P2;
                        }
                    }else {
                        no = 0;
                    }
                    delta += 2*(no-pre);
                    //down
                    if(abs(disinti_n[j]-d)>0){
                        if(abs(disinti_n[j]-d)==1){
                            pre = P1;
                        }else{
                            pre = P2;
                        }
                    }else {
                        pre = 0;
                    }
                    if(abs(disinti_n[j]-d-f)>0){
                        if(abs(disinti_n[j]-d-f)==1){
                            no = P1;
                        }else{
                            no = P2;
                        }
                    }else {
                        no = 0;
                    }
                    delta += 2*(no-pre);
                    //left
                    if(abs(disinti[j-1]-d)>0){
                        if(abs(disinti[j-1]-d)==1){
                            pre = P1;
                        }else{
                            pre = P2;
                        }
                    }else {
                        pre = 0;
                    }
                    if(abs(disinti[j-1]-d-f)>0){
                        if(abs(disinti[j-1]-d-f)==1){
                            no = P1;
                        }else{
                            no = P2;
                        }
                    }else {
                        no = 0;
                    }
                    delta += 2*(no-pre);

                    //right
                    if(abs(disinti[j+1]-d)>0){
                        if(abs(disinti[j+1]-d)==1){
                            pre = P1;
                        }else{
                            pre = P2;
                        }
                    }else {
                        pre = 0;
                    }
                    if(abs(disinti[j+1]-d-f)>0){
                        if(abs(disinti[j+1]-d-f)==1){
                            no = P1;
                        }else{
                            no = P2;
                        }
                    }else {
                        no = 0;
                    }
                    delta += 2*(no-pre);
                }
                if(delta<=0){
                    disinti[j] += f;
                }
                else if(RAND01<exp(-delta/T)){
                    disinti[j] += f;
                }
            }
        }
    }
    for(int i = 1;i+1<size.height;i++){
        for(int j=1;j+1<size.width;j++){
            disptr[i*size.width+j] = disint[i*size.width+j];
        }
    }
    delete []disint;
}

void stereoItera(Mat &left,Mat &right,Mat &dis,int maxdis){
    if(left.size()!=right.size())
        return ;
    Mat leftgray,rightgray;
    Size size = left.size();
    leftgray.create(size,CV_8U);
    rightgray.create(size,CV_8U);
    cvtColor(left,leftgray,CV_BGR2GRAY);
    cvtColor(right,rightgray,CV_BGR2GRAY);


    unsigned char *leftptr = leftgray.data;
    unsigned char *rightptr = rightgray.data;


    int *disint = new int[size.width*size.height];
    int *disint2 = new int[size.width*size.height];

    //initalborder
    for(int i = 0;i<size.height;i++)
        for(int j = 0;j<size.width;j++)
            disint[i*size.width+j] = 0;
    if(0){
    //use sgbm to inital
    int SADWindowSize = 7;
    int numberOfDisparities = 16*6;
    StereoSGBM sgbm;
    sgbm.preFilterCap = 63;
    sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

    int cn = 1;//leftgray.channels();

    sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
    sgbm.minDisparity = 0;
    sgbm.numberOfDisparities = numberOfDisparities;
    sgbm.uniquenessRatio = 10;
    sgbm.speckleWindowSize = 100;
    sgbm.speckleRange = 32;
    sgbm.disp12MaxDiff = 1;
    sgbm.fullDP = true;

    Mat disp;
    sgbm(leftgray,rightgray,disp);
    short int *sgbptr = (short int*)disp.data;
    for(int i = 0;i<size.height;i++){
        for(int j = 0;j<size.width;j++){
            disint[i*size.width+j] = sgbptr[i*size.width+j]/16;
        }
    }
    }

    if(1){
    int T = -1;
    int Tmax = 10;
    int P1 = 40;
    int P2 = 160;
    while(++T<Tmax){
        for(int i = 1;i+1<size.height;i++){
            int *disinti = disint+i*size.width;
            int *disinti_p = disinti-size.width;
            int *disinti_n = disinti+size.width;
            int *disint2i = disint2+i*size.width;
            unsigned char *leftptri = leftptr+i*size.width;
            unsigned char *rightptri = rightptr+i*size.width;
            for(int j = 1;j<size.width;j++){
                int mmin=1<<29;
                int mmindex;
                /*四邻域
                int near[4]={disinti_p[j],disinti_n[j],disinti[j-1],disinti[j+1]};
                for(int d = 0;d<maxdis&&d<=j;d++){
                    int cost = (leftptri[j] - rightptri[j-d])*(leftptri[j] - rightptri[j-d]);
                    for(int ni = 0;ni<4;ni++){
                        if(abs(d-near[ni])==1){
                            cost += P1;
                        }else if(abs(d-near[ni])>1){
                            cost += P2;
                        }
                    }
                    if(cost<mmin){
                        mmin = cost;
                        mmindex = d;
                    }
                }
                */
                //八邻域
                int near[8]={disinti_p[j-1],disinti_p[j],disinti_p[j+1],
                             disinti[j-1],disinti[j+1],
                             disinti_n[j-1],disinti_n[j],disinti_n[j+1]
                            };
                for(int d = 0;d<maxdis&&d<=j;d++){
                    int cost = (leftptri[j] - rightptri[j-d])*(leftptri[j] - rightptri[j-d]);
                    for(int ni = 0;ni<8;ni++){
                        if(abs(d-near[ni])==1){
                            cost += P1;
                        }else if(abs(d-near[ni])>1){
                            cost += P2;
                        }
                    }
                    if(cost<mmin){
                        mmin = cost;
                        mmindex = d;
                    }
                }
                disint2i[j] = mmindex;
            }
        }

        int *temp = disint;
        disint = disint2;
        disint2 = temp;
    }
    }
    dis.create(size,CV_32F);
    float *disptr = (float*)dis.data;

    for(int i = 0;i<size.height;i++){
        for(int j = 0;j<size.width;j++){
            disptr[i*size.width+j] = disint[i*size.width+j];
        }
    }
    delete []disint;
    delete []disint2;

}

void test_Simulated(){
    QString leftfilename = QFileDialog::getOpenFileName(
       0,
       "Binocular Calibration - Open Left Image",
       NULL,
       "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
    if (!leftfilename.isNull()) { //用户选择了左图文件
        QString rightfilename = QFileDialog::getOpenFileName(
           0,
           "Binocular Calibration - Open Right Image",
           NULL,
           "photos (*.img *.png *.bmp *.jpg);;All files(*.*)");
        if (!rightfilename.isNull()) { //用户选择了左图文件
            Mat leftmat = imread(leftfilename.toUtf8().data());
            Mat rightmat = imread(rightfilename.toUtf8().data());

            Mat disp,vdisp;
            clock_t t = clock();

            stereo_simulate(leftmat,rightmat,disp,40);
//            stereoItera(leftmat,rightmat,disp,140);

            qDebug()<<"use time"<<clock()-t<<"ms"<<endl;
            disp.convertTo(vdisp, CV_8U);//, 255/(32*16.));
            normalize(vdisp,vdisp,0,255,CV_MINMAX);
            imshow("dis",vdisp);
        }
    }
}
double fx(double x){
    return x*x;
}

/*
int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    test_Simulated();
    return app.exec();
}
*/
