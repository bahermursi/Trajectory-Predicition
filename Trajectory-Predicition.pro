#-------------------------------------------------
#
# Project created by QtCreator 2016-12-10T02:09:09
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets
QMAKE_MAC_SDK = macosx10.12

TARGET = Trajectory-Predicition
TEMPLATE = app

QMAKE_CXXFLAGS += -std=c++14
CONFIG += c++14

SOURCES += main.cpp\
    balltracker.cpp

HEADERS  += \
    balltracker.h \
    ui_mainwindow.h

FORMS    += mainwindow.ui

QT_CONFIG -= no-pkg-config
CONFIG  += link_pkgconfig
PKGCONFIG += opencv

