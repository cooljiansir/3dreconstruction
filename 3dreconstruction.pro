#-------------------------------------------------
#
# Project created by QtCreator 2014-03-04T13:45:50
#
#-------------------------------------------------

QT       += core gui opengl

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
    binoculardialog.cpp \
    photodialog2.cpp \
    uti.cpp \
    comparedialog.cpp \
    glwidget.cpp \
    cornerclicklabel.cpp \
    dialogcorner.cpp \
    stereoform.cpp \
    stereo.cpp

HEADERS  += mainwindow.h \
    photodialog.h \
    rtfdocument.h \
    tinyxml2.h \
    sigaldialog.h \
    clicklabel.h \
    binoculardialog.h \
    uti.h \
    photodialog2.h \
    comparedialog.h \
    glwidget.h \
    cornerclicklabel.h \
    dialogcorner.h \
    stereoform.h \
    stereo.h

FORMS    += mainwindow.ui \
    photodialog.ui \
    sigaldialog.ui \
    clicklabel.ui \
    binoculardialog.ui \
    photodialog2.ui \
    comparedialog.ui \
    dialogcorner.ui \
    stereoform.ui



INCLUDEPATH +=include\
            include\opencv\
            include\opencv2\


LIBS += $$_PRO_FILE_PWD_\lib\libopencv_highgui247.dll.a\
        $$_PRO_FILE_PWD_\lib\libopencv_core247.dll.a\
        $$_PRO_FILE_PWD_\lib\libopencv_imgproc247.dll.a\
        $$_PRO_FILE_PWD_\lib\libopencv_calib3d247.dll.a\
        $$_PRO_FILE_PWD_\lib\libopencv_contrib247.dll.a


RESOURCES +=
