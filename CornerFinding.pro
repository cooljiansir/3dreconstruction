#-------------------------------------------------
#
# Project created by QtCreator 2014-03-25T23:31:19
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = CornerFinding
TEMPLATE = app


SOURCES += main.cpp\
    MyMat.cpp \
    uti.cpp \
    clicklabel.cpp \
    photodialog.cpp \
    corner.cpp


INCLUDEPATH +=include\
            include\opencv\
            include\opencv2\


LIBS += $$_PRO_FILE_PWD_\lib\libopencv_highgui247.dll.a\
        $$_PRO_FILE_PWD_\lib\libopencv_core247.dll.a\
        $$_PRO_FILE_PWD_\lib\libopencv_imgproc247.dll.a\
        $$_PRO_FILE_PWD_\lib\libopencv_calib3d247.dll.a

HEADERS += \
    MyMat.h \
    uti.h \
    clicklabel.h \
    photodialog.h \
    corner.h

FORMS += \
    photodialog.ui
