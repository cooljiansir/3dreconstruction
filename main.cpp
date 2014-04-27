#include <QApplication>
#include "cv.h"
#include "highgui.h"
#include <QFileDialog>
#include <algorithm>
#include "MyMat.h"
#include "cornerdialog.h"

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


void gradient_x(Mat &imggray,Mat &res){
    Size size = imggray.size();
    res.create(size,CV_64F);

    unsigned char *imgptr = imggray.data;
    double *resptr = (double *)res.data;

    //initial border
    for(int i = 0;i<size.height;i++)
        resptr[i*size.width] = resptr[(i+1)*size.width-1] = 0;
    for(int i = 0;i<size.height;i++){
        double *resptri = resptr + i*size.width;
        unsigned char *imgptri = imgptr + i*size.width;
        for(int j = 1;j+1<size.width;j++){
            resptri[j] = imgptri[j+1] - imgptri[j-1];
        }
    }
}
void gradient_x_3(Mat &imggray,Mat &res,int arg=1){
    Size size = imggray.size();
    res.create(size,CV_64F);

    unsigned char *imgptr = imggray.data;
    double *resptr = (double *)res.data;

    //initial border
    for(int i = 0;i<size.height;i++)
        resptr[i*size.width] = resptr[(i+1)*size.width-1] = 0;
    for(int i = 0;i<size.width;i++)
        resptr[i] = resptr[(size.height-1)*size.width+i] = 0;
    for(int i = 1;i+1<size.height;i++){
        double *resptri = resptr + i*size.width;
        unsigned char *imgptri = imgptr + i*size.width;
        unsigned char *imgptri_p = imgptri - size.width;
        unsigned char *imgptri_n = imgptri + size.width;
        for(int j = 1;j+1<size.width;j++){
            resptri[j] = imgptri[j+1] - imgptri[j-1]
                    +arg*(imgptri_n[j+1] - imgptri_n[j-1])
                    +imgptri_p[j+1] - imgptri_p[j-1];
        }
    }
}
void gradient_y(Mat &imggray, Mat &res){
    Size size = imggray.size();
    res.create(size,CV_64F);

    double * resptr = (double*)res.data;
    unsigned char *imgptr = imggray.data;

    //initial border
    for(int i = 0;i<size.width;i++)
        resptr[i] = resptr[(size.height-1)*size.width+i] = 0;

    for(int i = 1;i+1<size.height;i++){
        double *resptri = resptr + i*size.width;
        unsigned char *imgptri = imgptr + i*size.width;
        unsigned char *imgptri_p =imgptri - size.width;
        unsigned char *imgptri_n =imgptri + size.width;
        for(int j = 0;j<size.width;j++){
            resptri[j] = imgptri_n[j] - imgptri_p[j];
        }
    }
}
void gradient_y_3(Mat &imggray, Mat &res,int arg=1){
    Size size = imggray.size();
    res.create(size,CV_64F);

    double * resptr = (double*)res.data;
    unsigned char *imgptr = imggray.data;

    //initial border
    for(int i = 0;i<size.width;i++)
        resptr[i] = resptr[(size.height-1)*size.width+i] = 0;
    for(int i = 0;i<size.height;i++)
        resptr[i*size.width] = resptr[(i+1)*size.width-1] = 0;

    for(int i = 1;i+1<size.height;i++){
        double *resptri = resptr + i*size.width;
        unsigned char *imgptri = imgptr + i*size.width;
        unsigned char *imgptri_p =imgptri - size.width;
        unsigned char *imgptri_n =imgptri + size.width;
        for(int j = 0;j<size.width;j++){
            resptri[j] =imgptri_n[j-1] - imgptri_p[j-1]
                        +arg*(imgptri_n[j] - imgptri_p[j])
                        +imgptri_n[j+1] - imgptri_p[j+1];
        }
    }
}

void conv(Mat &mata,Mat &matb,Mat &res){
    //高斯滤波窗口
    double gauss[5*5] = {
                        1.0 / 273,	4.0  / 273,	7.0  / 273,	4.0  / 273,	1.0/273,
                        4.0 / 273,	16.0 / 273,	26.0 / 273,	16.0 / 273,	4.0/273,
                        7.0 / 273,	26.0 / 273,	41.0 / 273,	26.0 / 273,	7.0/273,
                        4.0 / 273,	16.0 / 273,	26.0 / 273,	16.0 / 273,	4.0/273,
                        1.0 / 273,	4.0  / 273,	7.0  / 273,	4.0  / 273,	1.0/273};
    if(mata.size()!=matb.size())
        return;
    Size size = mata.size();

    res.create(size,CV_64F);
    double *resptr = (double *)res.data;
    double *mataptr = (double *)mata.data;
    double *matbptr = (double *)matb.data;

    //initial border
    for(int i = 0;i<2;i++)
        for(int j = 0;j<size.width;j++)
            resptr[i*size.width+j] = resptr[(size.height-1-i)*size.width+j] = 0;
    for(int i = 0;i<size.height;i++)
        for(int j = 0;j<2;j++)
            resptr[i*size.width+j] = resptr[i*size.width+size.width-1-j] = 0;

    //begin
    for(int i = 2;i+2<size.height;i++){
        double *mataptri = mataptr + i * size.width;
        double *matbptri = matbptr + i * size.width;
        double *resptri = resptr + i * size.width;
        for(int j = 2;j+2<size.width;j++){
            double sum = 0;
            int windex = 0;
            for(int i1 = -2;i1<=2;i1++)
                for(int j1 = -2;j1<=2;j1++){
                    sum += gauss[windex]
                            * mataptri[i1*size.width+j+j1]
                            * matbptri[i1*size.width+j+j1];
                    windex++;
                }
            resptri[j] = sum;
        }
    }
}

//calculate
//(IxIx)(IyIy) - (IxIy)*(IxIy)
//-----------------------------
//(IxIx)       +     (IyIy)
void calNobel(Mat &IxIx,Mat &IyIy,Mat &IxIy,Mat &res){
    Size size = IxIx.size();
    res.create(size,CV_64F);
    double *IxIxptr = (double *)IxIx.data;
    double *IyIyptr = (double *)IyIy.data;
    double *IxIyptr = (double *)IxIy.data;
    double *resptr  = (double *)res.data;

    for(int i = 0;i<size.height;i++){
        for(int j = 0;j<size.width;j++){
            *resptr = ((*IxIxptr)*(*IyIyptr) - (*IxIyptr)*(*IxIyptr)) /
                    ((*IxIxptr) + (*IyIyptr));
            IxIxptr++;
            IyIyptr++;
            IxIyptr++;
            resptr++;
        }
    }
}



//non-maximum suppression
void getCorner(Mat &cim,int winsize,double threshold,list<Point> &cornerList){
    Size size = cim.size();
    double *cimptr = (double *)cim.data;
    for(int i = winsize;i+winsize<size.height;i++){
        double *cimptri = cimptr + i*size.width;
        for(int j = winsize;j+winsize<size.width;j++){
            if(cimptri[j]>=threshold){
                bool ismax = true;
                for(int i1 = -winsize;i1<=winsize;i1++){
                    for(int j1  = -winsize;j1<=winsize;j1++){
                        if(cimptri[i1*size.width+j+j1]>cimptri[j]){
                            ismax = false;
                            break;
                        }
                    }
                }
                if(ismax)cornerList.push_back(Point(j,i));
            }
        }
    }
}

void testMoravec(){
    QString filename = QFileDialog::getOpenFileName();
    if(!filename.isEmpty()){
        Mat img = imread(filename.toStdString().c_str());
        //imshow("test",imggray);
        moravec(img,5000);

    }
}

void testgradient(){
    QString filename = QFileDialog::getOpenFileName();
    if(!filename.isEmpty()){
        Mat img = imread(filename.toStdString().c_str());
        Mat imggray;
        cvtColor(img,imggray,CV_BGR2GRAY);
        Mat gx,gx_3,gy,gy_3;

        gradient_x(imggray,gx);
        gradient_x_3(imggray,gx_3);
        gradient_y(imggray,gy);
        gradient_y_3(imggray,gy_3);

        Mat disgx,disgx_3,disgy,disgy_3;
        gx.convertTo(disgx,CV_8U);
        gx_3.convertTo(disgx_3,CV_8U);
        gy.convertTo(disgy,CV_8U);
        gy_3.convertTo(disgy_3,CV_8U);

        imshow("gx",disgx);
        imshow("gx_3",disgx_3);
        imshow("gy",disgy);
        imshow("gy_3",disgy_3);

    }
}
void getSubPix(Mat &cim,Point &a,Point2d &as){
//    freopen("log.txt","w",stdout);
    Size size  = cim.size();
    as.x = as.y = 0;
    double *cimptr = (double*)cim.data;
    if(a.x>=2&&a.x+2<size.width&&a.y>=2&&a.y+2<size.height){
        MyMat A(25,6);
        MyMat B(25,1);
        MyMat X(6,1);
        int windex = 0;
        for(int i1=-2;i1<=2;i1++){
            for(int j1 = -2;j1<=2;j1++){
                /*
                A.at(windex,0) = (a.x + j1)*(a.x + j1);
                A.at(windex,1) = (a.x + j1)*(a.y + i1);
                A.at(windex,2) = (a.y + i1)*(a.y + i1);
                A.at(windex,3) = (a.x + j1);
                A.at(windex,4) = (a.y + i1);
                A.at(windex,5) = 1;

                B.at(windex,0) = cimptr[(a.y + i1)*size.width+a.x+j1];
                */
                A.at(windex,0) = j1*j1;
                A.at(windex,1) = j1*i1;
                A.at(windex,2) = i1*i1;
                A.at(windex,3) = j1;
                A.at(windex,4) = i1;
                A.at(windex,5) = 1;

                B.at(windex,0) = cimptr[(a.y + i1)*size.width+a.x+j1];
                windex++;
            }
        }
        MyMat ATA = A.transpose()*A;
        MyMat ATA_(6,6);
        ATA.inverse(ATA_);
        X = ATA_*(A.transpose())*B;

        /*
        printf("A:\n");
        A.show();
        ATA.show();
        printf("ATA:\n");
        ATA.show();
        printf("ATA_:\n");
        ATA_.show();
        printf("X:\n");
        X.show();
        fflush(stdout);
        */

        as.x = (2*X.at(2,0)*X.at(3,0) - X.at(1,0)*X.at(4,0))/(X.at(1,0)*X.at(1,0)-4*X.at(0,0)*X.at(2,0));
        as.y = (2*X.at(0,0)*X.at(4,0) - X.at(1,0)*X.at(3,0))/(X.at(1,0)*X.at(1,0)-4*X.at(0,0)*X.at(2,0));
        as.x += a.x;
        as.y += a.y;
    }
}

void getSubPix2(Mat &Ix,Mat &Iy,Point &a, Point2d &as){
    Size size = Ix.size();
    as.x = as.y = 0;
    double *Ixptr = (double*)Ix.data;
    double *Iyptr = (double*)Iy.data;

    if(a.x>=2&&a.x+2<size.width&&
            a.y>=2&&a.y+2<size.height){
        MyMat A(25,2);
        MyMat B(25,1);
        MyMat X(2,1);
        int windex = 0;
        for(int i1 = -2;i1<=2;i1++){
            for(int j1 = -2;j1<=2;j1++){
                double dx = Ixptr[(a.y+i1)*size.width+a.x+i1];
                double dy = Iyptr[(a.y+i1)*size.width+a.x+i1];
                A.at(windex,0) = dx;
                A.at(windex,1) = dy;

                B.at(windex,0) = dx*j1+dy*i1;
                windex++;
            }
        }
        MyMat ATA = A.transpose()*A;
        MyMat ATA_(2,2);
        ATA.inverse(ATA_);
        X = ATA_*A.transpose()*B;
        as.x = X.at(0,0);
        as.y = X.at(1,0);
        as.x += a.x;
        as.y += a.y;
    }
}

void testNobel(){
    QString filename = QFileDialog::getOpenFileName();
    if(!filename.isEmpty()){
        Mat img = imread(filename.toStdString().c_str());
        Mat imggray;
        cvtColor(img,imggray,CV_BGR2GRAY);


        clock_t t1 = clock();
        Mat Ix,Iy;
        gradient_x_3(imggray,Ix);
        gradient_y_3(imggray,Iy);

        Mat IxIx,IyIy,IxIy;

        conv(Ix,Ix,IxIx);
        conv(Iy,Iy,IyIy);
        conv(Ix,Iy,IxIy);

        Mat cim;
        calNobel(IxIx,IyIy,IxIy,cim);

        list<Point> cornerList;
        getCorner(cim,3,2000,cornerList);



        cout<<"total used "<<clock()-t1<<"ms"<<endl;

        vector<Point2d> sublist;
        for(list<Point>::iterator it = cornerList.begin();it!=cornerList.end();it++){
            Point2d as;
//            getSubPix(cim,*it,as);
            getSubPix2(Ix,Iy,*it,as);
            sublist.push_back(as);
        }

        CornerDialog *dia = new CornerDialog(img,sublist);
        dia->exec();

        return;

        for(list<Point>::iterator it = cornerList.begin();it!=cornerList.end();it++){
            circle(img,*it,6,Scalar(0,0,255));
        }

        imshow("Nobel ",img);
    }
}
void testcvharris(){
    QString filename = QFileDialog::getOpenFileName();
    if(!filename.isEmpty()){
        Mat img = imread(filename.toStdString().c_str());
        Mat imggray;
        cvtColor(img,imggray,CV_BGR2GRAY);

        int thresh = 90;

        Mat dst, dst_norm, dst_norm_scaled;
        dst = Mat::zeros( img.size(), CV_32FC1 );

        /// Detector parameters
        int blockSize = 2;
        int apertureSize = 3;
        double k = 0.04;

        /// Detecting corners
        cornerHarris( imggray, dst, blockSize, apertureSize, k, BORDER_DEFAULT );

        /// Normalizing
        normalize( dst, dst_norm, 0, 255, NORM_MINMAX, CV_32FC1, Mat() );
        convertScaleAbs( dst_norm, dst_norm_scaled );

        /// Drawing a circle around corners
        for( int j = 0; j < dst_norm.rows ; j++ )
         { for( int i = 0; i < dst_norm.cols; i++ )
              {
                if( (int) dst_norm.at<float>(j,i) > thresh )
                  {
                   circle( img, Point( i, j ), 5,  Scalar(0), 2, 8, 0 );
                  }
              }
         }
        /// Showing the result
        imshow("opencv harris",img);
    }
}

int main(int argc,char * argv[]){
    QApplication app(argc,argv);

//    testMoravec();
//    testgradient();
    testNobel();
//    testcvharris();
    cvWaitKey(0);

    return app.exec();
}
