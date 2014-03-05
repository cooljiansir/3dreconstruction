#include "rtfdocument.h"

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
        vector<RPoint> img_v;
        vector<RPoint> img_r;
        while(ij!=NULL){
            double u,v,x,y;
            ij->QueryDoubleAttribute("u",&u);
            ij->QueryDoubleAttribute("v",&v);
            ij->QueryDoubleAttribute("x",&x);
            ij->QueryDoubleAttribute("y",&y);
            //printf("%lf %lf %lf %lf\n",u,v,x,y);
            RPoint p1;
            p1.x = u;
            p1.y = v;
            img_v.push_back(p1);
            RPoint p2;
            p2.x = x;
            p2.y = y;
            img_r.push_back(p2);
            ij = ij->NextSiblingElement();
        }
        this->image_pointpush_back(img_v);
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
