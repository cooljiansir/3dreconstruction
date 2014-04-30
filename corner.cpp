#include "corner.h"
#include <QDebug>

/*************************相关参数状态机*************************************/
Corner::Corner(){
    this->pixmethod = PIX_HARRIS;
    this->subpixel = SUBPIX_FITTING;
    thres[PIX_MORAVEC] = 5000;
    thres[PIX_HARRIS] = 40000;
    thres[PIX_NOBEL] = 15000;
    thres[PIX_SHI_TOMASI] = 15000;
    k = 0.04;
    winsize = 3;
}


void Corner::setPixMethod(int method){
    this->pixmethod = method;
}

void Corner::setSubPixMethod(int method){
    this->subpixel = method;
}

void Corner::setThreshold(double threshold){
    this->thres[this->pixmethod] = threshold;
}

double Corner::getThreshold(int method){
    return this->thres[method];
}
double Corner::getK(){
    return this->k;
}
void Corner::setK(double k){
    this->k = k;
}

void Corner::setWinsize(int w){
    this->winsize = w;
}
int Corner::getWinsize(){
    return this->winsize;
}
/*************************相关参数状态机*************************************/


/*************************亚像素角点检测*************************************/
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

void calMoravec(Mat &imggray,Mat &cim){
    Size size = imggray.size();

    cim.create(size,CV_64F);

    unsigned char *imgptr = imggray.data;
    double *cimptr = (double*)cim.data;

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
            cimptr[i*size.width+j] = min(min(value[0],value[1]),min(value[2],value[3]));
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

//
//两个特征值为：
//IxIx + IyIy +- sqrt((IxIx - IyIy)*(IxIx - IyIy)+4*IxIy*IxIy)
//------------------------------------------------------------
//                            2
//其中较小的一个为sqrt取负号时的值
//
void calShiTomas(Mat &IxIx,Mat &IyIy,Mat &IxIy,Mat &res){
    Size size = IxIx.size();
    res.create(size,CV_64F);
    double *IxIxptr = (double *)IxIx.data;
    double *IyIyptr = (double *)IyIy.data;
    double *IxIyptr = (double *)IxIy.data;
    double *resptr  = (double *)res.data;

    for(int i = 0;i<size.height;i++){
        for(int j = 0;j<size.width;j++){
            *resptr = 0.5*(
                        (*IxIxptr)+(*IyIyptr)
                        - sqrt(((*IxIxptr)-(*IyIyptr))*((*IxIxptr)-(*IyIyptr))+4*(*IxIyptr)*(*IxIyptr))
                        );
            IxIxptr++;
            IyIyptr++;
            IxIyptr++;
            resptr++;
        }
    }
}
//Harrris 角点响应函数
//AB-C*C-k*(A+B)*(A+B)
//即
//IxIx*IyIy-IxIy*IxIy - k*(IxIx+IyIy)*(IxIx+IyIy)
//由于值太大，将结果/10000
void calHarris(Mat &IxIx,Mat &IyIy,Mat &IxIy,Mat &res){
    double k=0.04;
    Size size = IxIx.size();
    res.create(size,CV_64F);
    double *IxIxptr = (double *)IxIx.data;
    double *IyIyptr = (double *)IyIy.data;
    double *IxIyptr = (double *)IxIy.data;
    double *resptr  = (double *)res.data;

    for(int i = 0;i<size.height;i++){
        for(int j = 0;j<size.width;j++){
            *resptr = (*IxIxptr)*(*IyIyptr) - (*IxIyptr)*(*IxIyptr) -
                    k*((*IxIxptr)+(*IyIyptr))*((*IxIxptr)+(*IyIyptr));
            *resptr /= 10000;//值太大了，缩小一定的比例
            IxIxptr++;
            IyIyptr++;
            IxIyptr++;
            resptr++;
        }
    }
}

//通过阈值和非极大值抑制得到角点
//
void getCornerBy(Mat &cim,int winsize,double threshold,vector<Point> &cornerList){
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


void getSubPix(Mat &cim,Point &a,Point2f &as){
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

        as.x = (2*X.at(2,0)*X.at(3,0) - X.at(1,0)*X.at(4,0))/(X.at(1,0)*X.at(1,0)-4*X.at(0,0)*X.at(2,0));
        as.y = (2*X.at(0,0)*X.at(4,0) - X.at(1,0)*X.at(3,0))/(X.at(1,0)*X.at(1,0)-4*X.at(0,0)*X.at(2,0));
        as.x += a.x;
        as.y += a.y;
        //如果计算出的距离和初始距离偏差太大，依然选择初始距离作为角点坐标
        if((as.x-a.x)*(as.x-a.x) + (as.y-a.y)*(as.y-a.y)>11*11){
            as.x = a.x;
            as.y = a.y;
        }
    }
}

void getSubPix2_(Mat &Ix,Mat &Iy,Point2f a, Point2f &as){
    int winsize = 5;
    Size size = Ix.size();
    as.x = as.y = 0;
    double *Ixptr = (double*)Ix.data;
    double *Iyptr = (double*)Iy.data;

    if(a.x>=winsize&&a.x+winsize<size.width&&
            a.y>=winsize&&a.y+winsize<size.height){
        MyMat A((2*winsize+1)*(2*winsize+1),2);
        MyMat B((2*winsize+1)*(2*winsize+1),1);
        MyMat X(2,1);
        int windex = 0;
        int ddx = (int)a.x;
        int ddy = (int)a.y;
        for(int i1 = -5;i1<=5;i1++){
            for(int j1 = -5;j1<=5;j1++){
                double dx = Ixptr[(ddy+i1)*size.width+ddx+j1];
                double dy = Iyptr[(ddy+i1)*size.width+ddx+j1];
                A.at(windex,0) = dx;
                A.at(windex,1) = dy;

                B.at(windex,0) = dx*(j1+ddx-a.x)+dy*(i1+ddy-a.y);
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
void getSubPix2(Mat &Ix,Mat &Iy,Point &a, Point2f &as){


    double thre = 0.01*0.01;
    int maxn = 1000;
    Point2f ap(a.x,a.y);
    Point2f asp(0,0);
    Point2f asp1(100,100);

    for(int i = 0;i<maxn;i++){
        getSubPix2_(Ix,Iy,ap,asp);
        if((asp1.x-asp.x)*(asp1.x-asp.x) + (asp1.y-asp.y)*(asp1.y-asp.y)<thre)
            break;
        as.x = asp.x;
        as.y = asp.y;
        asp1 = asp;
    }
    //如果迭代计算出的距离和初始距离偏差太大，依然选择初始距离作为角点坐标
    if((asp.x-a.x)*(asp.x-a.x) + (asp.y-a.y)*(asp.y-a.y)>11*11){
        as.x = a.x;
        as.y = a.y;
        return;
    }
    as = asp;
}

void Corner::getCorner(Mat &img, vector<Point2f> &corners){
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
    if(this->pixmethod==PIX_MORAVEC){
        calMoravec(imggray,cim);
    }
    else if(this->pixmethod==PIX_HARRIS){
        calHarris(IxIx,IyIy,IxIy,cim);
    }else if(this->pixmethod==PIX_NOBEL){
        calNobel(IxIx,IyIy,IxIy,cim);
    }else if(this->pixmethod==PIX_SHI_TOMASI){
        calShiTomas(IxIx,IyIy,IxIy,cim);
    }
    vector<Point> cornerList;
    getCornerBy(cim,this->winsize,this->thres[this->pixmethod],cornerList);
    cout<<"total used "<<clock()-t1<<"ms"<<endl;

    corners.clear();
    for(int i = 0;i<cornerList.size();i++){
        Point2f as;
        as.x = cornerList[i].x;
        as.y = cornerList[i].y;
        if(this->subpixel==SUBPIX_FITTING){
            getSubPix(cim,cornerList[i],as);
        }
        else if(this->subpixel==SUBPIX_VECTOR){
            getSubPix2(Ix,Iy,cornerList[i],as);
        }
        corners.push_back(as);
    }
}
