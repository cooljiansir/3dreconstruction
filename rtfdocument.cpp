#include "rtfdocument.h"
#include <highgui.h>

#define ZERO 0.000000001


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
void  RTFDocument::getCornerByHand(vector<CPoint> &rec_4,vector<vector<Point2f> > &corner){
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

    corner = grid_c[0];
}
