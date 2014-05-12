#include <QApplication>
#include "cv.h"
#include "highgui.h"
#include <QFileDialog>
#include <algorithm>
#include <time.h>

#include <QDebug>


using namespace cv;
using namespace std;



//左右一致性校验
//dis为输出视差
void LRC(int *cost,int maxdis,Size size,int maxdiff,int *dis){
    int *buff = new int[size.width*2];
    int *lcost = buff;
    int *rcost = buff+size.width;
    for(int i = 0;i<size.height;i++){
        int *costi = cost+i*size.width*maxdis;
        //L
        for(int j = 0;j<size.width;j++){
            int *costij = costi+j*maxdis;
            int minc = 1<<29;
            int mind ;
            for(int d = 0;d<maxdis&&d<=j;d++){
                if(costij[d]<minc)
                    minc = costij[d],mind = d;
            }
            lcost[j] = mind;
//            dis[i*size.width+j] = mind;
        }
//        continue;
        //R
        for(int j = 0;j<size.width;j++){
            int minc = 1<<29;
            int mind;
            for(int d = 0;d<maxdis&&j+d<size.width;d++){
                int *costjd = costi+(j+d)*maxdis;
                if(costjd[d]<minc)
                    minc = costjd[d],mind = d;
            }
            rcost[j] = mind;
        }
        //check
        for(int j = 0;j<size.width;j++){
            if(abs(lcost[j]-rcost[j])>maxdiff)
                dis[i*size.width+j]=  -1;
            else{
                dis[i*size.width+j] = lcost[j];
            }
        }
    }
    delete []buff;
}

//唯一性约束
void uniquenessC(int *cost,int *dis,int maxdis,Size size){
    int *buff = new int[size.width*2];
    int *rdis = buff;
    int *rcost = buff+size.width;
    for(int i = 0;i<size.height;i++){
        int *disi = dis+i*size.width;
        int *costi = cost+i*size.width*maxdis;
        for(int j = 0;j<size.width;j++)
            rdis[j] = -1,rcost[j]=1<<29;
        for(int j = 0;j<size.width;j++){
            int *costij = costi+j*maxdis;
            if(disi[j]!=-1){
                int d = disi[j];
                int jd = j-d;
                rdis[jd] = d;
                if(rcost[jd]<costij[d])
                    rdis[jd] = d,rcost[jd] = costij[d];
            }
        }
        for(int j = 0;j<size.width;j++){
            if(disi[j]!=-1){
                int d = disi[j];
                int dr = rdis[j-d];
                if(d != dr)
                    disi[j] = -1;
            }
        }
    }

    delete []buff;
}

void connetFilter(int *dis,Size size,int diff,int minarea){
    int *buff = new int[size.width*size.height*4];
    int *label = buff;
    int *labelArea = buff+size.width*size.height;
    int *quex = buff+2*size.width*size.height;
    int *quey = buff+3*size.width*size.height;
    int queindex;
    int labelcount = 0;


    for(int i = 0;i<size.height;i++){
        for(int j = 0;j<size.width;j++){
            label[i*size.width+j] = -1;
        }
    }
    for(int i = 0;i<size.height;i++){
        int di[4]={0,0,1,-1};
        int dj[4]={1,-1,0,0};
        int *labeli = label+i*size.width;
        int *disi = dis+i*size.width;
        for(int j = 0;j<size.width;j++){
            if(labeli[j]!=-1){
                if(labelArea[labeli[j]]<minarea)
                    disi[j] = -1;
            }
            else if(disi[j]!=-1){
                queindex = 0;
                quex[queindex] = j;
                quey[queindex] = i;
                queindex++;
                int count = 0;
                while(queindex>0){
                    queindex--;
                    int dx = quex[queindex];
                    int dy = quey[queindex];
                    count++;
                    label[dy*size.width+dx] = labelcount;
                    for(int k = 0;k<4;k++){
                        int mx = dx+dj[k];
                        int my = dy+di[k];
                        if(mx>=0&&mx<size.width&&my>=0&&my<size.height
                                &&dis[my*size.width+mx]!=-1
                                &&label[my*size.width+mx]==-1
                                &&abs(dis[my*size.width+mx]-dis[dy*size.width+dx])<=diff){
                            quex[queindex] = mx;
                            quey[queindex] = my;
                            queindex++;
                        }
                    }
                }
                labelArea[labelcount++] = count;
                if(count<minarea)
                    disi[j] = -1;
            }
        }
    }
    delete []buff;
}

extern void stereo_BMBox_Cost(Mat &left,Mat &right,int *costout,int maxdis,int winsize);

void testRefine(){
    QString leftfilename = "D:/works/qtwork5_5/build-3dreconstruction-Desktop_Qt_5_1_0_MinGW_32bit-Debug/images/opencv/tsukubaL.bmp";
    QString rightfilename = "D:/works/qtwork5_5/build-3dreconstruction-Desktop_Qt_5_1_0_MinGW_32bit-Debug/images/opencv/tsukubaR.bmp";

    /*QString leftfilename = QFileDialog::getOpenFileName(
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
        */
            Mat leftmat = imread(leftfilename.toUtf8().data());
            Mat rightmat = imread(rightfilename.toUtf8().data());

            Mat dis,vdisp;


            clock_t t = clock();

            Size size = leftmat.size();
            int maxdis = 20;

            int *cost = new int[size.width*size.height*maxdis];
            int *disint = new int[size.width*size.height];
            stereo_BMBox_Cost(leftmat,rightmat,cost,maxdis,3);
            LRC(cost,maxdis,size,1,disint);
            uniquenessC(cost,disint,maxdis,size);
            connetFilter(disint,size,1,200);

            qDebug()<<"used time "<<clock()-t<<"ms"<<endl;

            dis.create(size,CV_32F);
            float *disptr = (float*)dis.data;
            for(int i = 0;i<size.height;i++)
                for(int j = 0;j<size.width;j++)
                    disptr[i*size.width+j] = disint[i*size.width+j];

            dis.convertTo(vdisp,CV_8U);
            normalize(vdisp,vdisp,0,255,CV_MINMAX);

            delete []cost;
            delete []disint;

            imshow("Filtered diff=1 minarea=200",vdisp);
//        }
//    }
}




/*
int main(int argc, char *argv[]){
    QApplication a(argc, argv);
    testRefine();
    return a.exec();
}
*/
