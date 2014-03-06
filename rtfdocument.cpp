#include "rtfdocument.h"
#include <highgui.h>

RTFDocument::RTFDocument(){
    this->fileopen = false;
}
RTFDocument::~RTFDocument(){

}
bool RTFDocument::read(string filename){
    XMLDocument doc;
    doc.LoadFile(filename.c_str());
    XMLElement *root = doc.RootElement();
    XMLElement *points = root->FirstChildElement();
    XMLElement *it = points->FirstChildElement();
    while(it!=NULL){
        XMLElement *ij = it->FirstChildElement();
        vector<Point2f> img_v;
        vector<Point2f> img_r;
        while(ij!=NULL){
            double u,v,x,y;
            ij->QueryDoubleAttribute("u",&u);
            ij->QueryDoubleAttribute("v",&v);
            ij->QueryDoubleAttribute("x",&x);
            ij->QueryDoubleAttribute("y",&y);
            //printf("%lf %lf %lf %lf\n",u,v,x,y);
            Point2f p1;
            p1.x = u;
            p1.y = v;
            img_v.push_back(p1);
            Point2f p2;
            p2.x = x;
            p2.y = y;
            img_r.push_back(p2);
            ij = ij->NextSiblingElement();
        }
        this->image_point.push_back(img_v);
        this->object_point.push_back(img_r);
        it = it->NextSiblingElement();
    }
}
bool RTFDocument::opened(){
    return this->fileopen;
}
bool RTFDocument::save(){

}
bool RTFDocument::write(string filename){

}

bool RTFDocument::selectBegin(string filename){
    this->selectimg = imread(filename.c_str());
    int maxCorner = 1000;
    double quality_level = 0.1;
    double min_dis = 9;
    Mat matgray;
    cvtColor(selectimg,matgray,CV_BGR2GRAY);
    harriscorners.clear();
    cv::goodFeaturesToTrack(matgray,harriscorners,maxCorner,quality_level,min_dis);
    Size winSize = Size( 5, 5 );
    Size zeroZone = Size( -1, -1 );
    TermCriteria criteria = TermCriteria( CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 40, 0.001 );
    cornerSubPix( matgray,harriscorners, winSize, zeroZone, criteria );
    /*
    for(int i = 0;i<harriscorners.size();i++){
        circle(matgray,harriscorners[i],9,Scalar(0,0,255));
    }
    imshow("test",matgray);
    */
}
bool RTFDocument::havepoint(int x, int y){
    int count = 0;
    for(int i = 0;i<harriscorners.size();i++){
        double dis = (harriscorners[i].x - x)*(harriscorners[i].x - x)
                + (harriscorners[i].y - y)*(harriscorners[i].y - y);
        if(dis<SERACH_RANGE*SERACH_RANGE){
            count++;
        }
    }
    return count==1;
}
