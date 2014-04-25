#include <QApplication>
#include "cv.h"
#include "highgui.h"
#include <QFileDialog>
#include <algorithm>

#include <QDebug>


using namespace cv;
using namespace std;

int min(int a,int b){
    return a<b?a:b;
}

void moravec(Mat &img,int thre){
    Mat imggray;
    imggray.create(img.size(),CV_8UC1);
    cvtColor(img,imggray,CV_BGR2GRAY);
    unsigned char *imgptr = imggray.data;

    Size size = img.size();
    int *E = new int[size.width*size.height];
    int du[4]={1,1,0,-1};
    int dv[4]={0,1,1,1};

    for(int i = 1;i+2<size.height;i++){
        for(int j = 2;j+2<size.width;j++){
            unsigned char *imgptrij = imgptr +i*size.width+j;
            int value[4];
            for(int k = 0;k<4;k++){
                value[k] = 0;
                for(int i1 = -1;i1<=1;i1++){
                    for(int j1 = -1;j1<=1;j1++){
                        int derta = imgptrij[i1*size.width + j1]
                                - imgptrij[(i1+dv[k])*size.width + j1 + du[k]];
                        value[k] += derta*derta;
                    }
                }
            }
            E[i*size.width+j] = min(min(value[0],value[1]),min(value[2],value[3]));
        }
    }
    for(int i = 1;i+2<size.height;i++){
        for(int j = 2;j+2<size.width;j++){
            if(E[i*size.width+j]>thre){
                circle(img,Point(j,i),6,Scalar(0,0,255));
            }
        }
    }
    imshow("test",img);
    delete []E;
}


void testMoravec(){
    QString filename = QFileDialog::getOpenFileName();
    if(!filename.isEmpty()){
        Mat img = imread(filename.toStdString().c_str());
        //imshow("test",imggray);
        moravec(img,5000);

    }
}


int main(int argc,char * argv[]){
    QApplication app(argc,argv);

    testMoravec();
    cvWaitKey(0);

    return app.exec();
}
