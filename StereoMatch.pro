#-------------------------------------------------
#
# Project created by QtCreator 2014-03-25T23:31:19
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = StereoMatch
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp \
    clicklabel.cpp \
    uti.cpp \
    bm.cpp \
    simulated.cpp \
    test.cpp \
    cvsgbm.cpp

HEADERS  += mainwindow.h \
    clicklabel.h \
    uti.h

FORMS    += mainwindow.ui \
    clicklabel.ui



INCLUDEPATH +=include\
            include\opencv\
            include\opencv2\


LIBS += $$_PRO_FILE_PWD_\lib\libopencv_highgui247.dll.a\
        $$_PRO_FILE_PWD_\lib\libopencv_core247.dll.a\
        $$_PRO_FILE_PWD_\lib\libopencv_imgproc247.dll.a\
        $$_PRO_FILE_PWD_\lib\libopencv_calib3d247.dll.a
