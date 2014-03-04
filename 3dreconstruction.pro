#-------------------------------------------------
#
# Project created by QtCreator 2014-03-04T13:45:50
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = 3dreconstruction
TEMPLATE = app


SOURCES += main.cpp\
        mainwindow.cpp

HEADERS  += mainwindow.h

FORMS    += mainwindow.ui



INCLUDEPATH +=include\
            include\opencv\
            include\opencv2\

LIBS += lib\libopencv_highgui247.dll.a\
        lib\libopencv_core247.dll.a
