#include "rtfdocument.h"
#include <highgui.h>
#include <QDebug>
#include "stereomatch.h"

#define ZERO 0.000000001


RTFDocument::RTFDocument(){
    this->fileopen = false;
    this->l_disok = this->l_insok = this->r_disok = this->r_insok = false;
    this->bin_R_isok = this->bin_T_isok = false;
    this->image_width = -1;
    this->image_height = -1;

    //各种匹配方法添加
    this->stereoMatchMethods.push_back(new StereoMatchOpencvSGBM());
    this->stereoMatchMethods.push_back(new StereoMatchOpencvBM());
    this->stereoMatchMethods.push_back(new StereoMatchOpencvVar());
    this->stereoMatchMethods.push_back(new StereoMatchDynamic());
}
RTFDocument::~RTFDocument(){
    for(int i = 0;i<this->stereoMatchMethods.size();i++){
        delete this->stereoMatchMethods[i];
    }
}
bool RTFDocument::read(QString filename){
    this->fileopen = true;
    this->filename = filename;
    tinyxml2::XMLDocument doc;
    doc.LoadFile(filename.toStdString().c_str());
    tinyxml2::XMLElement *root = doc.RootElement();
    tinyxml2::XMLElement *rootchild = root->FirstChildElement();
    while(rootchild!=NULL){
        if(strcmp(rootchild->Name(),"imagesize")==0){
            int wi,he;
            if(rootchild->QueryIntAttribute("width",&wi)==tinyxml2::XML_NO_ERROR&&
                   rootchild->QueryIntAttribute("height",&he)==tinyxml2::XML_NO_ERROR ){
                this->image_height = he;
                this->image_width = wi;
            }
        }
        else if(strcmp(rootchild->Name(),"single")==0){
            tinyxml2::XMLElement * singlechild = rootchild->FirstChildElement();
            while(singlechild!=NULL){
                if(strcmp(singlechild->Name(),"left")==0){
                    tinyxml2::XMLElement *leftchild = singlechild->FirstChildElement();
                    while(leftchild!=NULL){
                        if(strcmp(leftchild->Name(),"intrinsic")==0){
                            l_intrinsic = Mat::zeros(3, 3, CV_64F);
                            l_insok = true;
                            if(leftchild->QueryDoubleAttribute("d0",&l_intrinsic.at<double>(0,0))!=tinyxml2::XML_NO_ERROR)
                                l_insok = false;
                            if(leftchild->QueryDoubleAttribute("d1",&l_intrinsic.at<double>(0,1))!=tinyxml2::XML_NO_ERROR)
                                l_insok = false;
                            if(leftchild->QueryDoubleAttribute("d2",&l_intrinsic.at<double>(0,2))!=tinyxml2::XML_NO_ERROR)
                                l_insok = false;
                            if(leftchild->QueryDoubleAttribute("d3",&l_intrinsic.at<double>(1,0))!=tinyxml2::XML_NO_ERROR)
                                l_insok = false;
                            if(leftchild->QueryDoubleAttribute("d4",&l_intrinsic.at<double>(1,1))!=tinyxml2::XML_NO_ERROR)
                                l_insok = false;
                            if(leftchild->QueryDoubleAttribute("d5",&l_intrinsic.at<double>(1,2))!=tinyxml2::XML_NO_ERROR)
                                l_insok = false;
                            if(leftchild->QueryDoubleAttribute("d6",&l_intrinsic.at<double>(2,0))!=tinyxml2::XML_NO_ERROR)
                                l_insok = false;
                            if(leftchild->QueryDoubleAttribute("d7",&l_intrinsic.at<double>(2,1))!=tinyxml2::XML_NO_ERROR)
                                l_insok = false;
                            if(leftchild->QueryDoubleAttribute("d8",&l_intrinsic.at<double>(2,2))!=tinyxml2::XML_NO_ERROR)
                                l_insok = false;
                        }else if(strcmp(leftchild->Name(),"distoration")==0){
                            l_distortion = Mat::zeros(8, 1, CV_64F);
                            l_disok = true;
                            leftchild->QueryDoubleAttribute("d0",&l_distortion.at<double>(0,0));
                            leftchild->QueryDoubleAttribute("d1",&l_distortion.at<double>(1,0));
                            leftchild->QueryDoubleAttribute("d2",&l_distortion.at<double>(2,0));
                            leftchild->QueryDoubleAttribute("d3",&l_distortion.at<double>(3,0));
                            leftchild->QueryDoubleAttribute("d4",&l_distortion.at<double>(4,0));
                            leftchild->QueryDoubleAttribute("d5",&l_distortion.at<double>(5,0));
                            leftchild->QueryDoubleAttribute("d6",&l_distortion.at<double>(6,0));
                            leftchild->QueryDoubleAttribute("d7",&l_distortion.at<double>(7,0));
                        }
                        else if(strcmp(leftchild->Name(),"points")==0){
                            tinyxml2::XMLElement *pointchild = leftchild->FirstChildElement();
                            while(pointchild!=NULL){
                                if(strcmp(pointchild->Name(),"onepic")==0){
                                    vector<Point2f> veci;
                                    vector<Point3f> veco;
                                    tinyxml2::XMLElement *onepicchild = pointchild->FirstChildElement();
                                    while(onepicchild!=NULL){
                                        Point2f p2;
                                        Point3f p3;
                                        onepicchild->QueryFloatAttribute("u",&p2.x);
                                        onepicchild->QueryFloatAttribute("v",&p2.y);
                                        onepicchild->QueryFloatAttribute("x",&p3.x);
                                        onepicchild->QueryFloatAttribute("y",&p3.y);
                                        onepicchild->QueryFloatAttribute("z",&p3.z);
                                        veci.push_back(p2);
                                        veco.push_back(p3);
                                        onepicchild = onepicchild->NextSiblingElement();
                                    }
                                    this->image_point_l.push_back(veci);
                                    this->object_point_l.push_back(veco);
                                }
                                pointchild = pointchild->NextSiblingElement();
                            }
                        }
                        leftchild = leftchild->NextSiblingElement();
                    }
                }
                else if(strcmp(singlechild->Name(),"right")==0){
                    tinyxml2::XMLElement *rightchild = singlechild->FirstChildElement();
                    while(rightchild!=NULL){
                        if(strcmp(rightchild->Name(),"intrinsic")==0){
                            r_intrinsic = Mat::zeros(3, 3, CV_64F);
                            r_insok = true;
                            if(rightchild->QueryDoubleAttribute("d0",&r_intrinsic.at<double>(0,0))!=tinyxml2::XML_NO_ERROR)
                                r_insok = false;
                            if(rightchild->QueryDoubleAttribute("d1",&r_intrinsic.at<double>(0,1))!=tinyxml2::XML_NO_ERROR)
                                r_insok = false;
                            if(rightchild->QueryDoubleAttribute("d2",&r_intrinsic.at<double>(0,2))!=tinyxml2::XML_NO_ERROR)
                                r_insok = false;
                            if(rightchild->QueryDoubleAttribute("d3",&r_intrinsic.at<double>(1,0))!=tinyxml2::XML_NO_ERROR)
                                r_insok = false;
                            if(rightchild->QueryDoubleAttribute("d4",&r_intrinsic.at<double>(1,1))!=tinyxml2::XML_NO_ERROR)
                                r_insok = false;
                            if(rightchild->QueryDoubleAttribute("d5",&r_intrinsic.at<double>(1,2))!=tinyxml2::XML_NO_ERROR)
                                r_insok = false;
                            if(rightchild->QueryDoubleAttribute("d6",&r_intrinsic.at<double>(2,0))!=tinyxml2::XML_NO_ERROR)
                                r_insok = false;
                            if(rightchild->QueryDoubleAttribute("d7",&r_intrinsic.at<double>(2,1))!=tinyxml2::XML_NO_ERROR)
                                r_insok = false;
                            if(rightchild->QueryDoubleAttribute("d8",&r_intrinsic.at<double>(2,2))!=tinyxml2::XML_NO_ERROR)
                                r_insok = false;

                        }else if(strcmp(rightchild->Name(),"distoration")==0){
                            r_distortion = Mat::zeros(8, 1, CV_64F);
                            r_disok = true;
                            rightchild->QueryDoubleAttribute("d0",&r_distortion.at<double>(0,0));
                            rightchild->QueryDoubleAttribute("d1",&r_distortion.at<double>(1,0));
                            rightchild->QueryDoubleAttribute("d2",&r_distortion.at<double>(2,0));
                            rightchild->QueryDoubleAttribute("d3",&r_distortion.at<double>(3,0));
                            rightchild->QueryDoubleAttribute("d4",&r_distortion.at<double>(4,0));
                            rightchild->QueryDoubleAttribute("d5",&r_distortion.at<double>(5,0));
                            rightchild->QueryDoubleAttribute("d6",&r_distortion.at<double>(6,0));
                            rightchild->QueryDoubleAttribute("d7",&r_distortion.at<double>(7,0));
                        }
                        else if(strcmp(rightchild->Name(),"points")==0){
                            tinyxml2::XMLElement *pointchild = rightchild->FirstChildElement();
                            while(pointchild!=NULL){
                                if(strcmp(pointchild->Name(),"onepic")==0){
                                    vector<Point2f> veci;
                                    vector<Point3f> veco;
                                    tinyxml2::XMLElement *onepicchild = pointchild->FirstChildElement();
                                    while(onepicchild!=NULL){
                                        Point2f p2;
                                        Point3f p3;
                                        onepicchild->QueryFloatAttribute("u",&p2.x);
                                        onepicchild->QueryFloatAttribute("v",&p2.y);
                                        onepicchild->QueryFloatAttribute("x",&p3.x);
                                        onepicchild->QueryFloatAttribute("y",&p3.y);
                                        onepicchild->QueryFloatAttribute("z",&p3.z);
                                        veci.push_back(p2);
                                        veco.push_back(p3);
                                        onepicchild = onepicchild->NextSiblingElement();
                                    }
                                    this->image_point_r.push_back(veci);
                                    this->object_point_r.push_back(veco);
                                }
                                pointchild = pointchild->NextSiblingElement();
                            }
                        }
                        rightchild = rightchild->NextSiblingElement();
                    }
                }
                singlechild = singlechild->NextSiblingElement();
            }
        }
        else if(strcmp(rootchild->Name(),"binocular")==0){
            tinyxml2::XMLElement *binchild = rootchild->FirstChildElement();
            while(binchild!=NULL){
                if(strcmp(binchild->Name(),"R")==0){
                    this->bin_R_isok = true;
                    bin_R = Mat::zeros(3,3,CV_64F);
                    if(binchild->QueryDoubleAttribute("d0",&bin_R.at<double>(0,0))!=tinyxml2::XML_NO_ERROR)
                        bin_R_isok = false;
                    if(binchild->QueryDoubleAttribute("d1",&bin_R.at<double>(0,1))!=tinyxml2::XML_NO_ERROR)
                        bin_R_isok = false;
                    if(binchild->QueryDoubleAttribute("d2",&bin_R.at<double>(0,2))!=tinyxml2::XML_NO_ERROR)
                        bin_R_isok = false;
                    if(binchild->QueryDoubleAttribute("d3",&bin_R.at<double>(1,0))!=tinyxml2::XML_NO_ERROR)
                        bin_R_isok = false;
                    if(binchild->QueryDoubleAttribute("d4",&bin_R.at<double>(1,1))!=tinyxml2::XML_NO_ERROR)
                        bin_R_isok = false;
                    if(binchild->QueryDoubleAttribute("d5",&bin_R.at<double>(1,2))!=tinyxml2::XML_NO_ERROR)
                        bin_R_isok = false;
                    if(binchild->QueryDoubleAttribute("d6",&bin_R.at<double>(2,0))!=tinyxml2::XML_NO_ERROR)
                        bin_R_isok = false;
                    if(binchild->QueryDoubleAttribute("d7",&bin_R.at<double>(2,1))!=tinyxml2::XML_NO_ERROR)
                        bin_R_isok = false;
                    if(binchild->QueryDoubleAttribute("d8",&bin_R.at<double>(2,2))!=tinyxml2::XML_NO_ERROR)
                        bin_R_isok = false;
                }
                else if(strcmp(binchild->Name(),"T")==0){
                    this->bin_T_isok = true;
                    this->bin_T = Mat::zeros(3,1,CV_64F);
                    if(binchild->QueryDoubleAttribute("d0",&bin_T.at<double>(0,0))!=tinyxml2::XML_NO_ERROR){
                        bin_T_isok = false;
                    }
                    if(binchild->QueryDoubleAttribute("d1",&bin_T.at<double>(1,0))!=tinyxml2::XML_NO_ERROR){
                        bin_T_isok = false;
                    }
                    if(binchild->QueryDoubleAttribute("d2",&bin_T.at<double>(2,0))!=tinyxml2::XML_NO_ERROR){
                        bin_T_isok = false;
                    }
                }
                else if(strcmp(binchild->Name(),"points")==0){
                    tinyxml2::XMLElement *onepic = binchild->FirstChildElement();
                    while(onepic!=NULL){
                        if(strcmp(onepic->Name(),"onepic")==0){
                            tinyxml2::XMLElement *point = onepic->FirstChildElement();
                            vector<Point2f> vel;
                            vector<Point2f> ver;
                            vector<Point3f> veo;
                            while(point!=NULL){
                                Point2f pl;
                                Point2f pr;
                                Point3f po;
                                point->QueryFloatAttribute("u_l",&pl.x);
                                point->QueryFloatAttribute("v_l",&pl.y);
                                point->QueryFloatAttribute("u_r",&pr.x);
                                point->QueryFloatAttribute("v_r",&pr.y);
                                point->QueryFloatAttribute("x",&po.x);
                                point->QueryFloatAttribute("y",&po.y);
                                point->QueryFloatAttribute("z",&po.z);
                                vel.push_back(pl);
                                ver.push_back(pr);
                                veo.push_back(po);
                                point = point->NextSiblingElement();
                            }
                            bin_image_point_l.push_back(vel);
                            bin_image_point_r.push_back(ver);
                            bin_object_point_l.push_back(veo);
                        }
                        onepic = onepic->NextSiblingElement();
                    }
                }
                binchild = binchild->NextSiblingElement();
            }
        }
        rootchild = rootchild->NextSiblingElement();
    }
}
bool RTFDocument::opened(){
    return this->fileopen;
}
bool RTFDocument::save(){
    tinyxml2::XMLDocument doc;
    tinyxml2::XMLElement *root = doc.NewElement("root");
    doc.LinkEndChild(root);
    //保存图片大小
    tinyxml2::XMLElement *imgsize = doc.NewElement("imagesize");
    root->LinkEndChild(imgsize);
    imgsize->SetAttribute("width",this->image_width);
    imgsize->SetAttribute("height",this->image_height);
    //保存单目标定数据
    tinyxml2::XMLElement *single = doc.NewElement("single");
    root->LinkEndChild(single);
    tinyxml2::XMLElement *singleleft = doc.NewElement("left");
    single->LinkEndChild(singleleft);
    if(l_insok){
        tinyxml2::XMLElement *singleleftinstrin = doc.NewElement("intrinsic");
        singleleftinstrin->SetAttribute("d0",this->l_intrinsic.at<double>(0,0));
        singleleftinstrin->SetAttribute("d1",this->l_intrinsic.at<double>(0,1));
        singleleftinstrin->SetAttribute("d2",this->l_intrinsic.at<double>(0,2));
        singleleftinstrin->SetAttribute("d3",this->l_intrinsic.at<double>(1,0));
        singleleftinstrin->SetAttribute("d4",this->l_intrinsic.at<double>(1,1));
        singleleftinstrin->SetAttribute("d5",this->l_intrinsic.at<double>(1,2));
        singleleftinstrin->SetAttribute("d6",this->l_intrinsic.at<double>(2,0));
        singleleftinstrin->SetAttribute("d7",this->l_intrinsic.at<double>(2,1));
        singleleftinstrin->SetAttribute("d8",this->l_intrinsic.at<double>(2,2));
        singleleft->LinkEndChild(singleleftinstrin);
    }
    if(l_disok){
        tinyxml2::XMLElement *singleleftdis = doc.NewElement("distoration");
        singleleftdis->SetAttribute("d0",this->l_distortion.at<double>(0,0));
        singleleftdis->SetAttribute("d1",this->l_distortion.at<double>(1,0));
        singleleftdis->SetAttribute("d2",this->l_distortion.at<double>(2,0));
        singleleftdis->SetAttribute("d3",this->l_distortion.at<double>(3,0));
        singleleftdis->SetAttribute("d4",this->l_distortion.at<double>(4,0));
        singleleft->LinkEndChild(singleleftdis);
    }
    tinyxml2::XMLElement *points = doc.NewElement("points");
    singleleft->LinkEndChild(points);
    for(int i = 0;i<image_point_l.size();i++){
        tinyxml2::XMLElement *pic = doc.NewElement("onepic");//一张图片上的点
        points->LinkEndChild(pic);
        for(int j = 0;j<image_point_l[i].size();j++){
            tinyxml2::XMLElement *pa = doc.NewElement("point");
            pic->LinkEndChild(pa);
            pa->SetAttribute("u",this->image_point_l[i][j].x);
            pa->SetAttribute("v",this->image_point_l[i][j].y);
            pa->SetAttribute("x",this->object_point_l[i][j].x);
            pa->SetAttribute("y",this->object_point_l[i][j].y);
            pa->SetAttribute("z",this->object_point_l[i][j].z);
        }
    }
    tinyxml2::XMLElement *singleright = doc.NewElement("right");
    single->LinkEndChild(singleright);
    if(r_insok){
        tinyxml2::XMLElement *singlerightinstrin = doc.NewElement("intrinsic");
        singlerightinstrin->SetAttribute("d0",this->r_intrinsic.at<double>(0,0));
        singlerightinstrin->SetAttribute("d1",this->r_intrinsic.at<double>(0,1));
        singlerightinstrin->SetAttribute("d2",this->r_intrinsic.at<double>(0,2));
        singlerightinstrin->SetAttribute("d3",this->r_intrinsic.at<double>(1,0));
        singlerightinstrin->SetAttribute("d4",this->r_intrinsic.at<double>(1,1));
        singlerightinstrin->SetAttribute("d5",this->r_intrinsic.at<double>(1,2));
        singlerightinstrin->SetAttribute("d6",this->r_intrinsic.at<double>(2,0));
        singlerightinstrin->SetAttribute("d7",this->r_intrinsic.at<double>(2,1));
        singlerightinstrin->SetAttribute("d8",this->r_intrinsic.at<double>(2,2));
        singleright->LinkEndChild(singlerightinstrin);
    }
    if(r_disok){
        tinyxml2::XMLElement *singlerightdis = doc.NewElement("distoration");
        singlerightdis->SetAttribute("d0",this->r_distortion.at<double>(0,0));
        singlerightdis->SetAttribute("d1",this->r_distortion.at<double>(1,0));
        singlerightdis->SetAttribute("d2",this->r_distortion.at<double>(2,0));
        singlerightdis->SetAttribute("d3",this->r_distortion.at<double>(3,0));
        singlerightdis->SetAttribute("d4",this->r_distortion.at<double>(4,0));
        singleright->LinkEndChild(singlerightdis);
    }
    points = doc.NewElement("points");
    singleright->LinkEndChild(points);
    for(int i = 0;i<image_point_r.size();i++){
        tinyxml2::XMLElement *pic = doc.NewElement("onepic");//一张图片上的点
        points->LinkEndChild(pic);
        for(int j = 0;j<image_point_r[i].size();j++){
            tinyxml2::XMLElement *pa = doc.NewElement("point");
            pic->LinkEndChild(pa);
            pa->SetAttribute("u",this->image_point_r[i][j].x);
            pa->SetAttribute("v",this->image_point_r[i][j].y);
            pa->SetAttribute("x",this->object_point_r[i][j].x);
            pa->SetAttribute("y",this->object_point_r[i][j].y);
            pa->SetAttribute("z",this->object_point_r[i][j].z);
        }
    }
    //保存双目标定数据

    tinyxml2::XMLElement *bino = doc.NewElement("binocular");
    root->LinkEndChild(bino);
    if(this->bin_R_isok){
        tinyxml2::XMLElement *binoR = doc.NewElement("R");
        binoR->SetAttribute("d0",this->bin_R.at<double>(0,0));
        binoR->SetAttribute("d1",this->bin_R.at<double>(0,1));
        binoR->SetAttribute("d2",this->bin_R.at<double>(0,2));
        binoR->SetAttribute("d3",this->bin_R.at<double>(1,0));
        binoR->SetAttribute("d4",this->bin_R.at<double>(1,1));
        binoR->SetAttribute("d5",this->bin_R.at<double>(1,2));
        binoR->SetAttribute("d6",this->bin_R.at<double>(2,0));
        binoR->SetAttribute("d7",this->bin_R.at<double>(2,1));
        binoR->SetAttribute("d8",this->bin_R.at<double>(2,2));
        bino->LinkEndChild(binoR);
    }

    if(this->bin_T_isok){
        tinyxml2::XMLElement *binoT = doc.NewElement("T");
        binoT->SetAttribute("d0",this->bin_T.at<double>(0,0));
        binoT->SetAttribute("d1",this->bin_T.at<double>(1,0));
        binoT->SetAttribute("d2",this->bin_T.at<double>(2,0));
        bino->LinkEndChild(binoT);
    }

    points = doc.NewElement("points");
    bino->LinkEndChild(points);
    for(int i = 0;i<bin_image_point_l.size();i++){
        tinyxml2::XMLElement *pic = doc.NewElement("onepic");//一张图片上的点
        points->LinkEndChild(pic);
        for(int j = 0;j<bin_image_point_l[i].size();j++){
            tinyxml2::XMLElement *pa = doc.NewElement("point");
            pic->LinkEndChild(pa);
            pa->SetAttribute("u_l",this->bin_image_point_l[i][j].x);
            pa->SetAttribute("v_l",this->bin_image_point_l[i][j].y);
            pa->SetAttribute("u_r",this->bin_image_point_r[i][j].x);
            pa->SetAttribute("v_r",this->bin_image_point_r[i][j].y);
            pa->SetAttribute("x",this->bin_object_point_l[i][j].x);
            pa->SetAttribute("y",this->bin_object_point_l[i][j].y);
            pa->SetAttribute("z",this->bin_object_point_l[i][j].z);
        }
    }

    doc.SaveFile(filename.toStdString().c_str());
}
bool RTFDocument::write(QString filename){
    this->filename = filename;
    this->fileopen = true;
    this->save();
}

bool RTFDocument::selectBegin(string filename, Mat &selectimg, vector<Point2f> &harriscorners){
    //selectimg = imread(filename.c_str());
    selectimg = imread(filename.c_str());
    if(this->image_height!=-1&&this->image_width!=-1){
        Size s(this->image_width,this->image_height);
        //Size s(50,50);
        //resize(selectimg,selectimg,s);
        resize(selectimg,selectimg,s);
    }

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
}
bool RTFDocument::havepoint(vector<Point2f> &harriscorners, int x, int y){
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
void RTFDocument::addCorner(int isright, Mat &selectimg,vector<vector<Point2f> > selectcorner, double width){
    vector<Point2f> img_vec;
    vector<Point3f> obj_vec;
    for(int i = 0;i<selectcorner.size();i++){
        for(int j = 0;j<selectcorner[i].size();j++){
            Point3f p;
            p.x = j*width;
            p.y = i*width;
            p.z = 0;
            img_vec.push_back(selectcorner[i][j]);
            obj_vec.push_back(p);
        }
    }
    if(isright==0){
        image_point_l.push_back(img_vec);
        object_point_l.push_back(obj_vec);
    }
    else if(isright==1){
        image_point_r.push_back(img_vec);
        object_point_r.push_back(obj_vec);
    }
    this->calParams(isright,selectimg);
    if(this->image_height==-1||this->image_width==-1){
        this->image_width = selectimg.cols;
        this->image_height = selectimg.rows;
    }
}
void RTFDocument::getCorner(int isr,vector<vector<Point2f> > &image_point, vector<vector<Point3f> > &object_point){
    if(isr==0){
        image_point = this->image_point_l;
        object_point = this->object_point_l;
    }else if(isr==1){
        image_point = this->image_point_r;
        object_point = this->object_point_r;
    }
}
void RTFDocument::calParams(int isright,Mat &selectimg){
    if(isright==0&&this->image_point_l.size()>0){
        this->l_disok = this->l_insok = true;
        vector<Mat> R_vec;
        vector<Mat> T_vec;
        l_distortion = Mat::zeros(8, 1, CV_64F);
        l_intrinsic = Mat::eye(3, 3, CV_64F);
        calibrateCamera(this->object_point_l,
                        this->image_point_l,
                        selectimg.size(),
                        l_intrinsic,
                        l_distortion,
                        R_vec,
                        T_vec,
                        CV_CALIB_FIX_K3
                        );
    }
    if(isright==1&&this->image_point_r.size()>0){
        this->r_disok = this->r_insok = true;
        vector<Mat> R_vec;
        vector<Mat> T_vec;
        r_distortion = Mat::zeros(8, 1, CV_64F);
        r_intrinsic = Mat::eye(3, 3, CV_64F);
        calibrateCamera(this->object_point_r,
                        this->image_point_r,
                        selectimg.size(),
                        r_intrinsic,
                        r_distortion,
                        R_vec,
                        T_vec,
                        CV_CALIB_FIX_K3
                        );

    }
}

bool RTFDocument::getLintrisic(Mat &intrinsic){
    if(!this->l_insok)
        return false;
    intrinsic =  this->l_intrinsic;
    return true;
}

bool RTFDocument::getLdistortion(Mat &distortion){
    if(!this->l_disok)
        return false;
    distortion = this->l_distortion;
    return true;
}

bool RTFDocument::getRintrinsic(Mat &intrinsic){
    if(!this->r_insok)
        return false;
    intrinsic = this->r_intrinsic;
    return true;
}

bool RTFDocument::getRdistortion(Mat &distortion){
    if(!this->r_disok)
        return false;
    distortion =  this->r_distortion;
    return true;
}

void RTFDocument::getBinData(vector<vector<Point2f> > &bin_image_point_l,
                             vector<vector<Point2f> > &bin_image_point_r,
                             vector<vector<Point3f> > &bin_object_point_l){
    bin_image_point_l = this->bin_image_point_l;
    bin_image_point_r = this->bin_image_point_r;
    bin_object_point_l = this->bin_object_point_l;
}
void RTFDocument::addBinData(Mat &selectimg,
                             vector<vector<Point2f> > &cornerL,
                             vector<vector<Point2f> > &cornerR,
                             double width){
    vector<Point2f> vecL;
    vector<Point2f> vecR;
    vector<Point3f> vecO;
    for(int i = 0;i<cornerL.size();i++){
        for(int j = 0;j<cornerL[i].size();j++){
            vecL.push_back(cornerL[i][j]);
            vecR.push_back(cornerR[i][j]);
            Point3f p;
            p.x = j*width;
            p.y = i*width;
            p.z = 0;
            vecO.push_back(p);
        }
    }
    this->bin_image_point_l.push_back(vecL);
    this->bin_image_point_r.push_back(vecR);
    this->bin_object_point_l.push_back(vecO);
    this->calBinParam(selectimg);
    if(this->image_height==-1||this->image_width==-1){
        this->image_width = selectimg.cols;
        this->image_height = selectimg.rows;
    }
}
void RTFDocument::calBinParam(Mat &matimg){
    if(!this->l_insok||!this->l_disok)
        return;
    if(!this->r_insok||!this->r_disok)
        return;
    Mat E,F;
    this->bin_R_isok = this->bin_T_isok = true;
    this->bin_T = Mat::zeros(3, 1, CV_64F);
    stereoCalibrate(this->bin_object_point_l,
                    this->bin_image_point_l,
                    this->bin_image_point_r,
                    l_intrinsic,
                    l_distortion,
                    r_intrinsic,
                    r_distortion,
                    matimg.size(),
                    bin_R,
                    bin_T,
                    E,
                    F,
                    cvTermCriteria(CV_TERMCRIT_ITER+CV_TERMCRIT_EPS, 100, 1e-5));
}

bool RTFDocument::getBinR(Mat &bin_R){
    if(!this->bin_R_isok)
        return false;
    bin_R = this->bin_R;
    return true;
}
bool RTFDocument::getBinT(Mat &bin_T){
    if(!this->bin_T_isok)
        return false;
    bin_T = this->bin_T;
    return true;
}

bool RTFDocument::isPolarOk(){
    if(!this->bin_R_isok||!this->bin_T_isok)//还未进行双目标定
        return false;
    if(!this->l_disok||!this->l_insok||!this->r_disok||!this->r_insok)
        return false;
    return true;
}

bool RTFDocument::getPolarParam(Mat &maplx, Mat &maply, Mat &maprx, Mat &mapry, Mat &Q, Size imgsize){
    if(!this->bin_R_isok||!this->bin_T_isok)//还未进行双目标定
        return false;
    Mat Rl,Rr,Pl,Pr;
    stereoRectify(this->l_intrinsic,this->l_distortion,this->r_intrinsic,this->r_distortion,imgsize,
                  this->bin_R,this->bin_T,Rl,Rr,Pl,Pr,Q);
    initUndistortRectifyMap(this->l_intrinsic,this->l_distortion,Rl,Pl,imgsize,CV_16SC2,maplx,maply);
    initUndistortRectifyMap(this->r_intrinsic,this->r_distortion,Rr,Pr,imgsize,CV_16SC2,maprx,mapry);
}


void RTFDocument::reproject3D(Mat &disp, Mat &img3D, Mat &Q){
    reprojectImageTo3D(disp,img3D,Q,false,CV_32F);
}


int cmp_x(const Point2f &a,const Point2f &b){
    return a.x<b.x;
}
int cmp_y(const Point2f &a,const Point2f &b){
    return a.y<b.y;
}

/*
 (y - y1)*(x1-x2) - (x - x1)*(y1-y2)
 先判断c和d点是否在ab两侧
 在判断a和b点是否在cd两侧

*/
int getsign(int a){
    if(a>0)return 1;
    else if(a<0)return -1;
    return 0;
}
bool canmeet(CPoint a,CPoint b,CPoint c,CPoint d){
    int v1 = (c.y - a.y) * (a.x - b.x) - (c.x - a.x) * (a.y - b.y);
    int v2 = (d.y - a.y) * (a.x - b.x) - (d.x - a.x) * (a.y - b.y);
    int v3 = (a.y - c.y) * (c.x - d.x) - (a.x - c.x) * (c.y - d.y);
    int v4 = (b.y - c.y) * (c.x - d.x) - (b.x - c.x) * (c.y - d.y);
    bool re =  (getsign(v1) * getsign(v2)<0)&&(getsign(v3) * getsign(v4)<0);
    //char temp[1024];
    //if(re)
    //	sprintf(temp,"(%d,%d)(%d,%d)和(%d,%d),(%d,%d)\n相交",a.x,a.y,b.x,b.y,c.x,c.y,d.x,d.y);
    //else sprintf(temp,"(%d,%d)(%d,%d)和(%d,%d),(%d,%d)\n不想交相交",a.x,a.y,b.x,b.y,c.x,c.y,d.x,d.y);
    //MessageBox(temp);
    return re;
}

int findCorner(vector<Point2f> &corners,CPoint a){

    for(int i = 0;i<corners.size();i++){
        double dis_2 = (corners[i].x - a.x) *(corners[i].x - a.x)  +(corners[i].y - a.y)*(corners[i].y - a.y);
        if(dis_2<SERACH_RANGE*SERACH_RANGE)return i;
    }
    return -1;
}
/*
    (y - a.y)*(a.x-b.x) - (x - a.x)*(a.y-b.y) = 0;
    点x0,y0到
    Ax + By + C = 0;
    的距离为
    fabs(Ax0+By0+C)/sqrt(A*A+B*B)
*/
double distance(CPoint a,CPoint b,CPoint x){
    return fabs(double((x.y - a.y) * (a.x - b.x) - (x.x-a.x)*(a.y - b.y)))
        /sqrt(double((a.x-b.x)*(a.x-b.x)+(a.y-b.y)*(a.y-b.y)));
}
bool between(CPoint a,CPoint b,CPoint x){
    double x_2 = (a.x + b.x)/2;
    double y_2 = (a.y + b.y)/2;
    double r_2 = ((a.x - b.x)*(a.x - b.x) + (a.y - b.y)*(a.y - b.y))/4;
    if((x.x - x_2)*(x.x - x_2) + (x.y - y_2)*(x.y - y_2)<r_2){
        if(distance(a,b,x)<SERACH_RANGE)
            return true;
    }
    return false;
}

void findCorner(vector<Point2f> &corners,CPoint a,CPoint b,vector<Point2f> &find_corners){
    for(int i = 0;i<corners.size();i++){
        if(between(a,b,CPoint(corners[i].x,corners[i].y))){
            find_corners.push_back(corners[i]);
        }
        else{
            double d1 = (corners[i].x - a.x)*(corners[i].x - a.x) + (corners[i].y - a.y)*(corners[i].y - a.y);
            double d2 = (corners[i].x - b.x)*(corners[i].x - b.x) + (corners[i].y - b.y)*(corners[i].y - b.y);
            //a和b点本身需包括
            if(d1<SERACH_RANGE*SERACH_RANGE||d2<SERACH_RANGE*SERACH_RANGE)
                find_corners.push_back(corners[i]);
        }
    }
}


/*
自动判断选择的4点关系：
1、如果最上面的点的横坐标没有过一半，就用它做原点
2、如果最上面的点的横坐标过一半，就用挨着它左边的定点做原点
         *********************
         *      **           *
         *     *   *         *
         *    *      *       *
         *   *         *     *
         *  *            *   *
         * *               * *
         **                 **
         *  *              * *
         *    *           *  *
         *      *        *   *
         *        *     *    *
         *          *  *     *
         *           **      *
         *********************

*/
void  RTFDocument::getCornerByHand(vector<Point2f> harriscorners,vector<CPoint> &rec_4,vector<vector<Point2f> > &corner){
    //更正角点顺序为
    //0  1
    //2  3
    //最上，最下，最左，最右的点
    int m_up_i = 0,m_bottom_i = 0,m_left_i = 0,m_right_i = 0;
    for(int i = 1;i<4;i++){
        if(rec_4[i].y<rec_4[m_up_i].y)
            m_up_i = i;
        if(rec_4[i].y>rec_4[m_bottom_i].y)
            m_bottom_i = i;
        if(rec_4[i].x<rec_4[m_left_i].x)
            m_left_i = i;
        if(rec_4[i].x>rec_4[m_right_i].x)
            m_right_i = i;
    }
    int t_origin;//原点
    int t_cor;//原点的对对角点
    if(rec_4[m_up_i].x<(rec_4[m_left_i].x+rec_4[m_right_i].x)/2)
        t_origin = m_up_i;
    else t_origin = m_left_i;

    if(t_origin!=0){
        CPoint temp = rec_4[t_origin];
        rec_4[t_origin] = rec_4[0];
        rec_4[0] = temp;
    }
    if(canmeet(rec_4[0],rec_4[1],rec_4[2],rec_4[3]))
        t_cor = 1;
    else if(canmeet(rec_4[0],rec_4[2],rec_4[1],rec_4[3]))
        t_cor = 2;
    else t_cor = 3;
    if(t_cor!=3){
        CPoint temp = rec_4[t_cor];
        rec_4[t_cor] = rec_4[3];
        rec_4[3] = temp;
    }

    if(rec_4[1].x<rec_4[2].x){
        CPoint temp = rec_4[1];
        rec_4[1] = rec_4[2];
        rec_4[2] = temp;
    }

    //自动判断选中矩形的规模
    vector<Point2f> &corn = harriscorners;
    Point2f point_4[4];
    for(int i = 0;i<4;i++){
        int t_c = findCorner(corn,rec_4[i]);
        if(t_c==-1){
            return ;
        }
        point_4[i] = corn[t_c];
    }
    vector<Point2f> corner_edge[4];//上、下、左、右边上的角点
    //上面一条边上的角点
    findCorner(corn,CPoint(point_4[0].x,point_4[0].y),CPoint(point_4[1].x,point_4[1].y),corner_edge[0]);
    sort(corner_edge[0].begin(),corner_edge[0].end(),cmp_x);
    //下面一条边上的角点
    findCorner(corn,CPoint(point_4[2].x,point_4[2].y),CPoint(point_4[3].x,point_4[3].y),corner_edge[1]);
    sort(corner_edge[1].begin(),corner_edge[1].end(),cmp_x);
    //左边一条边上的角点
    findCorner(corn,CPoint(point_4[0].x,point_4[0].y),CPoint(point_4[2].x,point_4[2].y),corner_edge[2]);
    sort(corner_edge[2].begin(),corner_edge[2].end(),cmp_y);
    //右边一条边上的角点
    findCorner(corn,CPoint(point_4[1].x,point_4[1].y),CPoint(point_4[3].x,point_4[3].y),corner_edge[3]);
    sort(corner_edge[3].begin(),corner_edge[3].end(),cmp_y);

    //分别对应两种计算方式的结果
    //按行和按列计算
    //如果两种结果不相同，说明有误检
    vector<vector<Point2f> > grid_c[2];
    //依靠左右两边得到所有的点
    if(corner_edge[2].size()!=corner_edge[3].size()){
//        MessageBox("左右两边上的角点个数不相同");
        return ;
    }
    for(int i = 0;i<corner_edge[2].size();i++){
        vector<Point2f> corner_line;
        findCorner(corn,CPoint(corner_edge[2][i].x,corner_edge[2][i].y),CPoint(corner_edge[3][i].x,corner_edge[3][i].y),corner_line);
        sort(corner_line.begin(),corner_line.end(),cmp_x);
        grid_c[0].push_back(corner_line);
    }
    //依靠上下两边得到所有的点
    if(corner_edge[0].size()!=corner_edge[1].size()){
//        MessageBox("上下两边上的角点个数不相同");
        return ;
    }
    for(int i = 0;i<corner_edge[0].size();i++){
        vector<Point2f> corner_row;
        findCorner(corn,CPoint(corner_edge[0][i].x,corner_edge[0][i].y),CPoint(corner_edge[1][i].x,corner_edge[1][i].y),corner_row);
        sort(corner_row.begin(),corner_row.end(),cmp_y);
        grid_c[1].push_back(corner_row);
    }
    bool issame = true;
    //检查每一行是不是规整的
    for(int i = 1;i<grid_c[0].size();i++)
        if(grid_c[0][i].size()!=grid_c[0][0].size())
            issame = false;
    //检查每一列是不是规整的
    for(int i = 1;i<grid_c[1].size();i++)
        if(grid_c[1][i].size()!=grid_c[1][0].size())
            issame = false;
    if(grid_c[0].size()!=grid_c[1][0].size())
        issame = false;
    if(grid_c[0][0].size()!=grid_c[1].size())
        issame = false;
    if(!issame){
//        MessageBox("监测到的角点不规范");
        return ;
    }
    for(int i = 0;i<grid_c[0].size();i++){
        for(int j = 0;j<grid_c[0][i].size();j++){
            if(fabs(grid_c[0][i][j].x - grid_c[1][j][i].x)>ZERO
                ||fabs(grid_c[0][i][j].y - grid_c[1][j][i].y)>ZERO){
//                    MessageBox("监测到的角点不规范");
                    return ;
            }
        }
    }
    corner = grid_c[0];

    /*
    Mat teMat = this->selectimg.clone();
    Mat teMat2 = this->selectimg.clone();

    for(int i = 0;i<grid_c[0].size();i++){
        for(int j = 0;j<grid_c[0][i].size();j++)
            circle(teMat,grid_c[0][i][j],SERACH_RANGE,Scalar(0,0,255));
    }
    imshow("corners",teMat);

    for(int i = 0;i<grid_c[1].size();i++){
        for(int j = 0;j<grid_c[1][i].size();j++)
            circle(teMat2,grid_c[1][i][j],SERACH_RANGE,Scalar(0,0,255));
    }
    imshow("corners2",teMat2);
    */

}
