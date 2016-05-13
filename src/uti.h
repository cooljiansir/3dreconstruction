#ifndef UTI_H
#define UTI_H

#include <QImage>
#include <cv.h>
using namespace cv;
//Mat 转Qimage
QImage Mat2QImage(cv::Mat const& src);
//QImage转Mat
cv::Mat QImage2Mat(QImage const& src);
#endif // UTI_H
