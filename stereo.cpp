#include "stereo.h"
#include <QDebug>
#include <highgui.h>


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

//灰度图转梯度图
void Xsobel(Mat &img,Mat &res,int thre){
    Size size = img.size();

    Mat temp;
    temp.create(size,CV_8U);
    unsigned char *resptr = temp.data;

    //initial border
    for(int i = 0;i<size.height;i++){
        resptr[i*size.width] = resptr[i*size.width+size.width-1] = 0;
    }
    for(int i = 0;i<size.width;i++){
        resptr[i] = resptr[(size.height-1)*size.width+i] = 0;
    }

    for(int i = 1;i+1<size.height;i++){
        unsigned char *imgptri = img.ptr<unsigned char>(i);
        unsigned char *imgptri_n = img.ptr<unsigned char>(i+1);
        unsigned char *imgptri_p = img.ptr<unsigned char>(i-1);
        unsigned char *resptri = resptr+i*size.width;
        for(int j = 1;j+1<size.width;j++){
            int d = (imgptri[j+1] - imgptri[j-1])*2 + imgptri_n[j+1] - imgptri_n[j-1]+imgptri_p[j+1] - imgptri_p[j-1];
            if(d<-thre)d = -thre;
            if(d>thre)d = thre;
            resptri[j] = d + 128;
        }
    }
    temp.copyTo(res);
}

/*
 *从上往下，BlockMatching
 *加入BT算法
 *
 *如果不使用Sobel预处理，令prefilter<0
 *如果不使用BT算法，令BT=false
 *
 *
 *Sum(x,y,d) = Sum(x,y-1,d) + s(x,y+winsize,d) - s(x,y-winsize-1,d)
 *s(x,y,d)   = s(x-1,y,d) + I(x+winsize,y,d)-I(x-winsize-1,y,d)
 *I(x,y,d)   = fabs(c(x,y)-c(x-d,y))
 *
 *
 */


void stereo_BMBox_BT(Mat &left,Mat &right,int *costout,int maxdis,int winsize,int prefilter,bool BT){
    if(left.size()!=right.size())
        return;
    Mat leftgr,rightgr;
    Size size = left.size();

    leftgr = Mat::zeros(size.height+2*winsize+1,size.width+2*winsize+1,CV_8U);
    rightgr = Mat::zeros(leftgr.size(),CV_8U);
    Size size2 = leftgr.size();

    Mat leftgray = leftgr(Rect(winsize+1,winsize+1,size.width,size.height));
    Mat rightgray = rightgr(Rect(winsize+1,winsize+1,size.width,size.height));

    cvtColor(left,leftgray,CV_BGR2GRAY);
    cvtColor(right,rightgray,CV_BGR2GRAY);

    if(prefilter>0){
        Xsobel(leftgray,leftgray,prefilter);
        Xsobel(rightgray,rightgray,prefilter);
    }


    unsigned char *leftptr = leftgray.data;
    unsigned char *rightptr = rightgray.data;


    int *sad = new int[maxdis];
    int dfafs;
    int *s0_   = new int[(size.height+2*winsize+1)*maxdis];
    int *s1_   = new int[(size.height+2*winsize+1)*maxdis];

    int *s0 = s0_ + (winsize+1)*maxdis;
    int *s1 = s1_ + (winsize+1)*maxdis;


    //initial s0
    for(int i = - winsize-1;i<size.height+winsize;i++){
        for(int d = 0;d<maxdis;d++)
            s0[i*maxdis+d] = s1[i*maxdis+d] = 0;
    }


    for(int j = 0;j<size.width;j++){
        for(int d = 0;d<maxdis&&d<=j;d++){
            sad[d] = 0;
            for(int i1=0;i1<winsize;i1++){
                for(int j1 = -winsize;j1<=winsize;j1++){
                    int temp = i1*size2.width+j+j1;
//                    sad[d] += abs(leftptr[i1*size2.width+j+j1]-rightptr[i1*size2.width+j+j1-d]);
                    if(!BT)
                        sad[d] += abs(leftptr[temp]-rightptr[temp-d]);
                    else{
                        int a1=leftptr[temp-1],a2=leftptr[temp],a3=leftptr[temp+1],
                                b1=rightptr[temp-d-1],b2=rightptr[temp-d],b3=rightptr[temp-d+1];
                        a1 = (a1+a2)/2;
                        a3 = (a2+a3)/2;
                        b1 = (b1+b2)/2;
                        b3 = (b2+b3)/2;
                        sad[d] +=
                        min(
                            min(
                                min(
                                    min(abs(a1-b1),abs(a1-b2)),
                                    min(abs(a1-b3),abs(a2-b1))),
                                min(
                                    min(abs(a2-b2),abs(a2-b3)),
                                    min(abs(a3-b1),abs(a3-b2)))),
                            abs(a3-b3));
                    }

//                    int A =  max(max(0,a2-max(max(b1,b2),b3)),min(min(b1,b2),b3)-a2);
//                    int B =  max(max(0,b2-max(max(a1,a2),a3)),min(min(a1,a2),a3)-b2);

//                    sad[d] += min(A,B);
                }
            }
        }

        //cal new s0
        for(int i = - winsize-1;i<size.height+winsize;i++){
            //d<j
            int tem = i*size2.width+j+winsize;
            int tem2 = i*size2.width+j-winsize-1;
            int *s0temp = s0 + i*maxdis;
            int *s1temp = s1 + i*maxdis;
            for(int d = 0;d<maxdis&&d<j;d++){
                if(!BT){
                    s1temp[d] = s0temp[d]
                        + abs(leftptr[tem]-rightptr[tem-d])
                        - abs(leftptr[tem2]-rightptr[tem2-d]);
                }
                else{
                    int a1=leftptr[tem-1],a2=leftptr[tem],a3=leftptr[tem+1],
                            b1=rightptr[tem-d-1],b2=rightptr[tem-d],b3=rightptr[tem-d+1];
                    a1 = (a1+a2)/2;
                    a3 = (a2+a3)/2;
                    b1 = (b1+b2)/2;
                    b3 = (b2+b3)/2;
                    s1temp[d] = s0temp[d] +
                            min(
                                min(
                                    min(
                                        min(abs(a1-b1),abs(a1-b2)),
                                        min(abs(a1-b3),abs(a2-b1))),
                                    min(
                                        min(abs(a2-b2),abs(a2-b3)),
                                        min(abs(a3-b1),abs(a3-b2)))),
                                abs(a3-b3));

                    a1=leftptr[tem2-1],a2=leftptr[tem2],a3=leftptr[tem2+1],
                            b1=rightptr[tem2-d-1],b2=rightptr[tem2-d],b3=rightptr[tem2-d+1];
                    a1 = (a1+a2)/2;
                    a3 = (a2+a3)/2;
                    b1 = (b1+b2)/2;
                    b3 = (b2+b3)/2;
                    s1temp[d] -=
                            min(
                                min(
                                    min(
                                        min(abs(a1-b1),abs(a1-b2)),
                                        min(abs(a1-b3),abs(a2-b1))),
                                    min(
                                        min(abs(a2-b2),abs(a2-b3)),
                                        min(abs(a3-b1),abs(a3-b2)))),
                                abs(a3-b3));
                }
            }
            //d=j
            if(j<maxdis){
                s1[i*maxdis+j] = 0;
                for(int j1=-winsize;j1<=winsize;j1++){
                    if(!BT)
                        s1[i*maxdis+j] += abs(leftptr[i*size2.width+j+j1]-rightptr[i*size2.width+j1]);
                    else{
                        int temp = i*size2.width+j1;
                        int a1 = leftptr[temp+j-1],a2=leftptr[temp+j],a3=leftptr[temp+j+1];
                        int b1 = rightptr[temp-1],b2 = rightptr[temp],b3 = rightptr[temp+1];
                        a1 = (a1+a2)/2;
                        a3 = (a2+a3)/2;
                        b1 = (b1+b2)/2;
                        b3 = (b2+b3)/2;
                        s1[i*maxdis+j] +=
                                min(
                                    min(
                                        min(
                                            min(abs(a1-b1),abs(a1-b2)),
                                            min(abs(a1-b3),abs(a2-b1))),
                                        min(
                                            min(abs(a2-b2),abs(a2-b3)),
                                            min(abs(a3-b1),abs(a3-b2)))),
                                    abs(a3-b3));
                    }

                }
            }
        }
        int *temp = s0;
        s0 = s1;
        s1 = temp;
        int *costj = costout+j*maxdis;
        for(int i = 0;i<size.height;i++){
            int mind;
            int minsad=1<<29;
            int tem = (i+winsize)*maxdis;
            int tem2 = (i-1-winsize)*maxdis;
            for(int d = 0;d<maxdis&&d<=j;d++){
                sad[d] = sad[d] + s0[tem+d] - s0[tem2+d];
                costj[d] = sad[d];
            }
            costj +=size.width*maxdis;
        }

    }
    delete []sad;
    delete []s0_;
    delete []s1_;
}

//换成Lab空间
//L = L*100/255
//a = a-128
//b = b-128
void stereo_BM_AW_Cost(Mat &left,Mat &right,int *costin,int maxdis,int winsize){
    if(left.size()!=right.size())
        return;
    double yc=7,yg=36;
//    double yc=50,yg=36;

    Size size = left.size();


    unsigned char *leftptr = left.data;
    unsigned char *rightptr = right.data;

    Mat leftlab,rightlab;
    cvtColor(left,leftlab,CV_BGR2Lab);
    cvtColor(right,rightlab,CV_BGR2Lab);

    unsigned char *leftlabptr = leftlab.data;
    unsigned char *rightlabptr = rightlab.data;

    int *cost = new int[size.width*size.height*maxdis];

    Mat dis;
    dis.create(size,CV_32F);
    float *disptr = (float*)dis.data;

    //double *mincost = new double[size.width];
    //double *
    double *w1buff = new double[(2*winsize+1)*(2*winsize+1)*size.width];
    double *w2buff = new double[(2*winsize+1)*(2*winsize+1)*size.width];


    for(int i = winsize;i+winsize<size.height;i++){
        for(int j = winsize;j+winsize<size.width;j++){
            int p = (i*size.width + j)*3;
            int windex = j*(2*winsize+1)*(2*winsize+1);
            for(int i1 = -winsize;i1<=winsize;i1++){
                for(int j1 = -winsize;j1<=winsize;j1++){
                    int q = ((i+i1)*size.width + j+j1)*3;
                    double w1 =
                            exp(-sqrt((leftlabptr[p] - leftlabptr[q])*(leftlabptr[p] - leftlabptr[q])
                                      *100.0/255.0*100.0/255.0+
                                      (leftlabptr[p+1] - leftlabptr[q+1])*(leftlabptr[p+1] - leftlabptr[q+1])+
                                      (leftlabptr[p+2] - leftlabptr[q+2])*(leftlabptr[p+2] - leftlabptr[q+2]))/yc
                            -sqrt(i1*i1+j1*j1)/yg);
                    double w2 =
                            exp(-sqrt((rightlabptr[p] - rightlabptr[q])*(rightlabptr[p] - rightlabptr[q])+
                                         (rightlabptr[p+1] - rightlabptr[q+1])*(rightlabptr[p+1] - rightlabptr[q+1])+
                                         (rightlabptr[p+2] - rightlabptr[q+2])*(rightlabptr[p+2] - rightlabptr[q+2]))/yc
                            -sqrt(i1*i1+j1*j1)/yg);
                    w1buff[windex] = w1;
                    w2buff[windex] = w2;
                    windex++;
                }
            }
        }
        int *costini = costin+i*size.width*maxdis;
        int *costi = cost+i*size.width*maxdis;
        for(int j = winsize;j+winsize<size.width;j++){
            double mincost;
            int mmindex=-1;
            int *costij = costi+j*maxdis;
            for(int d = 0;d<maxdis&&d+winsize<=j;d++){
                double sum = 0;
                double sumw = 0;
                int p = (i*size.width + j)*3;
                int p_ = p-3*d;
                int w1index = j*(2*winsize+1)*(2*winsize+1);
                int w2index = (j-d)*(2*winsize+1)*(2*winsize+1);
                for(int i1 = -winsize;i1<=winsize;i1++){
                    for(int j1 = -winsize;j1<=winsize;j1++){

                        int q = ((i+i1)*size.width + j+j1)*3;
                        int q_ = q-d*3;
//                        double e1 = fabs(leftptr[q]-rightptr[q_])+
//                                fabs(leftptr[q+1]-rightptr[q_+1])+
//                                fabs(leftptr[q+2]-rightptr[q_+2]);
                        double e1 = costini[(i1*size.width+j+j1)*maxdis+d];

                        double w1 = w1buff[w1index++];
                        double w2 = w2buff[w2index++];
                        sum += w1*w2*e1;
                        sumw += w1*w2;
                    }
                }
                costij[d] = sum/sumw;
            }
        }
        qDebug()<<"finished "<<i*100/size.height<<"%\r";
    }

    for(int k = 0;k<4;k++){
        int i0[4]={0,size.height-winsize,0,0},
                in[4]={winsize,size.height,size.height,size.height},
                j0[4]={0,0,0,size.width-winsize},
                jn[4]={size.width,size.width,winsize,size.width};
        for(int i = i0[k];i<in[k];i++){
            for(int j = j0[k];j<jn[k];j++){
                for(int d = 0;d<maxdis&&d<=j;d++){
                    cost[(i*size.width+j)*maxdis+d] = 0;
                }
            }
        }
    }
    for(int i = winsize;i+winsize<size.height;i++){
        int *costini = costin+i*size.width*maxdis;
        int *costi = cost + i*size.width*maxdis;
        for(int j = winsize;j+winsize<size.width;j++){
            int *costinij = costini+j*maxdis;
            int *costij = costi + j*maxdis;
            for(int d = 0;d<maxdis&&d<=j;d++){
                costinij[d] = costij[d];
            }
        }
    }


    delete []w1buff;
    delete []w2buff;
    delete []cost;
}


/*
 *抛物线拟合
 *d = d + (C(-1)-C(1))/(2C(-1)-4C(0)+2C(1))
 *
 */

void subpixel(int *cost,Size size,int *disint,int maxdis,float *dis){
    for(int i = 0;i<size.height;i++){
        int *costi = cost+i*size.width*maxdis;
        int *disinti = disint+i*size.width;
        float *disi = dis+i*size.width;
        for(int j = 0;j<size.width;j++){
            int *costij = costi + j*maxdis;
            int d = disinti[j];
            if(d>0&&d<maxdis){
                int c_1=costij[d-1],c0 = costij[d],c1 = costij[d+1];
                int bo = c_1-2*c0+c1;
                if(bo!=0)
                    disi[j] = d+(c_1-c1)/(bo*2.0);
                else
                    disi[j] = d;
            }else
                disi[j] = d;
        }
    }
}

void stereo_MSGM_MY(Mat &left,Mat &right,Mat &dis,int maxdis,int P1,int P2,int iter,
                    int winsize,int prefilter,bool BT,bool lrc,bool uniq,bool filt,bool subpix,int AW_FBS){
    if(left.size()!=right.size())
        return;
    Size size = left.size();

    dis.create(size,CV_32F);
    float *disptr = (float *)dis.data;

    Mat leftgray,rightgray;
    cvtColor(left,leftgray,CV_BGR2GRAY);
    cvtColor(right,rightgray,CV_BGR2GRAY);

    unsigned char *leftptr = leftgray.data;
    unsigned char *rightptr = rightgray.data;

    //-1,size.width+1
    int LrWidth = size.width+2;
    int LrMax  = maxdis+2;

    int *cost = new int[size.width*size.height*maxdis];


    if(AW_FBS==1){
        stereo_BMBox_BT(left,right,cost,maxdis,0,prefilter,BT);
        stereo_BM_AW_Cost(left,right,cost,maxdis,winsize);
    }else if(AW_FBS==2){
        stereo_BMBox_BT(left,right,cost,maxdis,0,prefilter,BT);
    }
    else{
        stereo_BMBox_BT(left,right,cost,maxdis,winsize,prefilter,BT);
    }

    int *disint = new int[size.width*size.height];
    int *LrSum   = new int[size.width*size.height*maxdis];
    int *LrBuff_ = new int[LrWidth*LrMax*4*2];//4个方向，双缓冲,Lr(j,d+1)
    int *minLr_ = new int[LrWidth*4*2];//4个方向，双缓冲
    if(!LrSum||!LrBuff_||!minLr_){
        return ;
    }

    int *Lr0 = LrBuff_+LrMax;
    int *Lr01 = Lr0+4*LrWidth*LrMax;
    int *minLr0 = minLr_+1;
    int *minLr01 = minLr0+4*LrWidth;


    //initial Lr
    for(int i = -1;i<=size.width;i++){
        int *Lr1 = Lr0+LrWidth*LrMax;
        int *Lr2 = Lr0+2*LrWidth*LrMax;
        int *Lr3 = Lr0+3*LrWidth*LrMax;
        int *Lr11 = Lr01+LrWidth*LrMax;
        int *Lr21 = Lr01+2*LrWidth*LrMax;
        int *Lr31 = Lr01+3*LrWidth*LrMax;
        for(int d = -1;d<=maxdis;d++){
            int v = 0;
            if(d==-1||d==maxdis)
                v = 1<<29;
            Lr0[i*LrMax+d+1] = v;
            Lr1[i*LrMax+d+1] = v;
            Lr2[i*LrMax+d+1] = v;
            Lr3[i*LrMax+d+1] = v;
            Lr01[i*LrMax+d+1] = v;
            Lr11[i*LrMax+d+1] = v;
            Lr21[i*LrMax+d+1] = v;
            Lr31[i*LrMax+d+1] = v;
        }
    }

    //initial minLr
    for(int i = -1;i<=size.width;i++){
        int *minLr1 = minLr0+LrWidth;
        int *minLr2 = minLr0+2*LrWidth;
        int *minLr3 = minLr0+3*LrWidth;
        int *minLr11 = minLr01+LrWidth;
        int *minLr21 = minLr01+2*LrWidth;
        int *minLr31 = minLr01+3*LrWidth;
        minLr0[i] = 0;
        minLr1[i] = 0;
        minLr2[i] = 0;
        minLr3[i] = 0;
        minLr01[i] = 0;
        minLr11[i] = 0;
        minLr21[i] = 0;
        minLr31[i] = 0;
    }

    for(int pass = 0;pass<iter;pass++){
        //四个方向
        //
        //1 2  3
        // ↖↑↗
        //0←
        //
        for(int i = 0;i<size.height;i++){
        int *Lr1 = Lr0+LrWidth*LrMax;
        int *Lr2 = Lr0+2*LrWidth*LrMax;
        int *Lr3 = Lr0+3*LrWidth*LrMax;
        int *Lr11 = Lr01+LrWidth*LrMax;
        int *Lr21 = Lr01+2*LrWidth*LrMax;
        int *Lr31 = Lr01+3*LrWidth*LrMax;
        int *minLr1 = minLr0+LrWidth;
        int *minLr2 = minLr0+2*LrWidth;
        int *minLr3 = minLr0+3*LrWidth;
        int *minLr11 = minLr01+LrWidth;
        int *minLr21 = minLr01+2*LrWidth;
        int *minLr31 = minLr01+3*LrWidth;

        int *costi = cost+i*size.width*maxdis;
        int *LrSumi = LrSum + i*size.width*maxdis;
        for(int j = 0;j<size.width;j++){
            int *costij = costi+j*maxdis;
            int *LrSumij = LrSumi+j*maxdis;

            int *Lr0j_1 = Lr0+(j-1)*LrMax;
            int *minLr0j = minLr0+j;
            *minLr0j = 1<<29;
            int *Lr0j = Lr0+j*LrMax;

            int *Lr11j = Lr11+j*LrMax;
            int *Lr1j_1 = Lr1 + (j-1)*LrMax;
            int *minLr11j = minLr11+j;
            *minLr11j =  1<<29;

            int *Lr21j = Lr21+j*LrMax;
            int *Lr2j= Lr2+j*LrMax;
            int *minLr21j = minLr21+j;
            *minLr21j = 1<<29;

            int *Lr31j = Lr31+j*LrMax;
            int *Lr3j_1 = Lr3 + (j+1)*LrMax;
            int *minLr31j = minLr31 + j;
            *minLr31j = 1<<29;

            int minc = 1<<29;
            int mind=0;
            for(int d = 0;d<maxdis&&d<=j;d++){
                Lr0j[d+1] = costij[d]
                        + min(min(Lr0j_1[d+1],minLr0[j-1]+P2),min(Lr0j_1[d],Lr0j_1[d+2])+P1)- minLr0[j-1];
                *minLr0j = min(*minLr0j,Lr0j[d+1]);

                Lr11j[d+1] = costij[d]
                        + min(min(Lr1j_1[d+1],minLr1[j-1]+P2),min(Lr1j_1[d],Lr1j_1[d+2])+P1) - minLr1[j-1];
                *minLr11j = min(*minLr11j,Lr11j[d+1]);

                Lr21j[d+1] = costij[d]
                        +min(min(Lr2j[d+1],minLr2[j]+P2),min(Lr2j[d],Lr2j[d+2])+P1) - minLr2[j];
                *minLr21j = min(*minLr21j,Lr21j[d+1]);

                Lr31j[d+1] = costij[d]
                        +min(min(Lr3j_1[d+1],minLr3[j+1]+P2),min(Lr3j_1[d],Lr3j_1[d+2])+P1) - minLr3[j+1];
                *minLr31j = min(*minLr31j,Lr31j[d+1]);

                LrSumij[d] = 0
                        +Lr0j[d+1]
                        +Lr11j[d+1]
                        +Lr21j[d+1]
                        +Lr31j[d+1]
                        ;

            }
            disptr[i*size.width+j] = mind;
        }
        int *temp = Lr0;
        Lr0  = Lr01;
        Lr01 = temp;

        temp = minLr0;
        minLr0 = minLr01;
        minLr01  = temp;
    }
        //四个方向
        //
        //1 2  3
        // ↖↑↗
        //0←
        //
        for(int i = size.height-1;i>=0;i--){
        int *Lr1 = Lr0+LrWidth*LrMax;
        int *Lr2 = Lr0+2*LrWidth*LrMax;
        int *Lr3 = Lr0+3*LrWidth*LrMax;
        int *Lr11 = Lr01+LrWidth*LrMax;
        int *Lr21 = Lr01+2*LrWidth*LrMax;
        int *Lr31 = Lr01+3*LrWidth*LrMax;
        int *minLr1 = minLr0+LrWidth;
        int *minLr2 = minLr0+2*LrWidth;
        int *minLr3 = minLr0+3*LrWidth;
        int *minLr11 = minLr01+LrWidth;
        int *minLr21 = minLr01+2*LrWidth;
        int *minLr31 = minLr01+3*LrWidth;

        int *costi = cost+i*size.width*maxdis;
        int *LrSumi = LrSum + i*size.width*maxdis;
        for(int j = size.width-1;j>=0;j--){
            int *costij = costi+j*maxdis;
            int *LrSumij = LrSumi+j*maxdis;

            int *Lr0j_1 = Lr0+(j+1)*LrMax;
            int *minLr0j = minLr0+j;
            *minLr0j = 1<<29;
            int *Lr0j = Lr0+j*LrMax;

            int *Lr11j = Lr11+j*LrMax;
            int *Lr1j_1 = Lr1 + (j-1)*LrMax;
            int *minLr11j = minLr11+j;
            *minLr11j =  1<<29;

            int *Lr21j = Lr21+j*LrMax;
            int *Lr2j= Lr2+j*LrMax;
            int *minLr21j = minLr21+j;
            *minLr21j = 1<<29;

            int *Lr31j = Lr31+j*LrMax;
            int *Lr3j_1 = Lr3 + (j+1)*LrMax;
            int *minLr31j = minLr31 + j;
            *minLr31j = 1<<29;

            int minc = 1<<29;
            int mind=0;
            for(int d = 0;d<maxdis&&d<=j;d++){
                Lr0j[d+1] = costij[d]
                        + min(min(Lr0j_1[d+1],minLr0[j+1]+P2),min(Lr0j_1[d],Lr0j_1[d+2])+P1)- minLr0[j+1];
                *minLr0j = min(*minLr0j,Lr0j[d+1]);

                Lr11j[d+1] = costij[d]
                        + min(min(Lr1j_1[d+1],minLr1[j-1]+P2),min(Lr1j_1[d],Lr1j_1[d+2])+P1) - minLr1[j-1];
                *minLr11j = min(*minLr11j,Lr11j[d+1]);

                Lr21j[d+1] = costij[d]
                        +min(min(Lr2j[d+1],minLr2[j]+P2),min(Lr2j[d],Lr2j[d+2])+P1) - minLr2[j];
                *minLr21j = min(*minLr21j,Lr21j[d+1]);

                Lr31j[d+1] = costij[d]
                        +min(min(Lr3j_1[d+1],minLr3[j+1]+P2),min(Lr3j_1[d],Lr3j_1[d+2])+P1)- minLr3[j+1];
                *minLr31j = min(*minLr31j,Lr31j[d+1]);

                LrSumij[d] += 0
                        +Lr0j[d+1]
                        +Lr11j[d+1]
                        +Lr21j[d+1]
                        +Lr31j[d+1]
                        ;
                LrSumij[d] /= 8;
                if(LrSumij[d]<minc)
                    minc = LrSumij[d],mind = d;
            }
//            disptr[i*size.width+j] = mind;
            disint[i*size.width+j] = mind;
        }
        int *temp = Lr0;
        Lr0  = Lr01;
        Lr01 = temp;

        temp = minLr0;
        minLr0 = minLr01;
        minLr01  = temp;
    }
        int *temp = cost;
        cost = LrSum;
        LrSum = temp;
    }


    if(lrc)
        LRC(cost,maxdis,size,2,disint);
    if(uniq)
        uniquenessC(cost,disint,maxdis,size);
    if(filt)
        connetFilter(disint,size,2,200);

    if(subpix)
        subpixel(cost,size,disint,maxdis,disptr);
    else
        for(int i = 0;i<size.width*size.height;i++)
            disptr[i] = disint[i];

    delete []cost;
    delete []LrSum;
    delete []LrBuff_;
    delete []minLr_;
    delete []disint;
}

//WTA BM 算法
void WTA_(Mat &left,Mat &right,Mat &dis,int maxdis,int winsize,bool BT,int prefilter,
          bool lrc,bool uniq,bool filt,bool subpix,int AW_FBS){
    if(left.size()!=right.size())
        return;
    Size size = left.size();
    int *cost = new int[size.width*size.height*maxdis];
    int *disint = new int[size.width*size.height];

    if(AW_FBS==1){
        stereo_BMBox_BT(left,right,cost,maxdis,0,prefilter,BT);
        stereo_BM_AW_Cost(left,right,cost,maxdis,winsize);
    }else if(AW_FBS==2){
        stereo_BMBox_BT(left,right,cost,maxdis,0,prefilter,BT);
    }
    else{
        stereo_BMBox_BT(left,right,cost,maxdis,winsize,prefilter,BT);
    }
    for(int i = 0;i<size.height;i++){
        int *costi = cost+i*size.width*maxdis;
        for(int j = 0;j<size.width;j++){
            int *costij = costi + j*maxdis;
            int mincost = 1<<29;
            int mind ;
            for(int d = 0;d<maxdis&&d<=j;d++){
                if(costij[d]<mincost)
                    mincost = costij[d],mind = d;
            }
            disint[i*size.width+j] = mind;
        }
    }

    if(lrc)
        LRC(cost,maxdis,size,2,disint);
    if(uniq)
        uniquenessC(cost,disint,maxdis,size);
    if(filt)
        connetFilter(disint,size,2,200);

    dis.create(size,CV_32F);
    float *disptr = (float*)dis.data;
    if(subpix){
        subpixel(cost,size,disint,maxdis,disptr);
    }
    else{
        for(int i = 0;i<size.width*size.height;i++)
            disptr[i] = disint[i];
    }
    delete []cost;
    delete []disint;
}

Stereo::Stereo()
{
    //初始化默认值
    method = METHOD_CVSGBM;
    maxdis = 16*8;
    costCalulate_BT = true;
    costCalulate_SOBEL = true;
    costAggregation = COST_AGGREGATION_FW;
    winsize = 1;
    computeDisparity = COMPUTE_DISPARITY_ITER_SGM;
    P1 = 8;
    P2 = 32;
    iterTimes = 5;
    disRefineLRC = false;
    disRefineUnique = true;
    disRefineFilter = true;
    disRefineSubPixel = true;
}

void Stereo::stereoMatch(Mat &leftmat, Mat &rightmat, Mat &dismat){
    if(this->method==METHOD_CVSGBM){
        int SADWindowSize = this->winsize;
        int numberOfDisparities = maxdis;

        StereoSGBM sgbm;
        sgbm.preFilterCap = 63;
        sgbm.SADWindowSize = SADWindowSize > 0 ? SADWindowSize : 3;

        int cn = 1;

        sgbm.P1 = 8*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.P2 = 32*cn*sgbm.SADWindowSize*sgbm.SADWindowSize;
        sgbm.minDisparity = 0;
        sgbm.numberOfDisparities = numberOfDisparities;
        sgbm.uniquenessRatio = 10;
        sgbm.speckleWindowSize = 100;
        sgbm.speckleRange = 32;
        sgbm.disp12MaxDiff = 1;
        sgbm.fullDP = true;

        Mat sgbmmat;
        Mat leftmatgray,rightmatgray;
        cvtColor(leftmat,leftmatgray,CV_BGR2GRAY);
        cvtColor(rightmat,rightmatgray,CV_BGR2GRAY);

        sgbm(leftmatgray,rightmatgray,sgbmmat);
        dismat.create(leftmat.size(),CV_32F);
        Size size = leftmat.size();
        for(int i = 0;i<size.height;i++)
            for(int j = 0;j<size.width;j++)
                *dismat.ptr<float>(i,j) = (*sgbmmat.ptr<short int>(i,j))/16.0;
    }
    else if(this->method==METHOD_CVBM){
        StereoBM bm;
        bm.state->preFilterCap = 31;
        bm.state->SADWindowSize = this->winsize;
        bm.state->minDisparity = 0;
        bm.state->numberOfDisparities = this->maxdis;
        bm.state->textureThreshold = 10;
        bm.state->uniquenessRatio = 15;
        bm.state->speckleWindowSize = 100;
        bm.state->speckleRange = 32;
        bm.state->disp12MaxDiff = 1;

        Mat leftmatgray,rightmatgray;
        cvtColor(leftmat,leftmatgray,CV_BGR2GRAY);
        cvtColor(rightmat,rightmatgray,CV_BGR2GRAY);

        bm(leftmatgray,rightmatgray,dismat,CV_32F);
    }
    else if(this->method==METHOD_CUSTOM){
        int AW_FBS = 0;
        if(this->costAggregation==Stereo::COST_AGGREGATION_AW)
            AW_FBS = 1;
        else if(this->costAggregation==Stereo::COST_AGGREGATION_FBS)
            AW_FBS = 2;
        if(this->computeDisparity==Stereo::COMPUTE_DISPARITY_ITER_SGM){
            stereo_MSGM_MY(leftmat,rightmat,dismat,
                       maxdis,P1,P2,this->iterTimes,this->winsize/2,
                       this->costCalulate_SOBEL?64:-1,this->costCalulate_BT,
                       this->disRefineLRC,this->disRefineUnique,this->disRefineFilter,
                           this->disRefineSubPixel,AW_FBS                           );
        }else if(this->computeDisparity==Stereo::COMPUTE_DISPARITY_WTA){
            WTA_(leftmat,rightmat,dismat,maxdis,winsize/2,this->costCalulate_BT,
                 this->costCalulate_SOBEL?64:-1,this->disRefineLRC
                 ,this->disRefineUnique,this->disRefineFilter,this->disRefineSubPixel,AW_FBS
                 );
        }else if(this->computeDisparity==Stereo::COMPUTE_DISPARITY_DP){

        }
    }
}
