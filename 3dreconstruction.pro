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
        mainwindow.cpp \
    photodialog.cpp \
    rtfdocument.cpp \
    tinyxml2.cpp \
    sigaldialog.cpp \
    clicklabel.cpp \
    binoculardialog.cpp

HEADERS  += mainwindow.h \
    photodialog.h \
    rtfdocument.h \
    tinyxml2.h \
    sigaldialog.h \
    clicklabel.h \
    binoculardialog.h

FORMS    += mainwindow.ui \
    photodialog.ui \
    sigaldialog.ui \
    clicklabel.ui \
    binoculardialog.ui



INCLUDEPATH +=include\
            include\opencv\
            include\opencv2\


LIBS += $$_PRO_FILE_PWD_\lib\libopencv_highgui247.dll.a\
        $$_PRO_FILE_PWD_\lib\libopencv_core247.dll.a\
        $$_PRO_FILE_PWD_\lib\libopencv_imgproc247.dll.a\
        $$_PRO_FILE_PWD_\lib\libopencv_calib3d247.dll.a


RESOURCES += \
    res.qrc
