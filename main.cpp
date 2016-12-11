#include "mainwindow.h"
#include <QApplication>
#include <opencv2/opencv.hpp>
#include "balltracker.h"
int main(int argc, char *argv[])
{

   //string colorNames = {"DarkGreen","Green","White","Yellow","Red","Orange","DarkYellow","DarkYellow2","LightGreen"};

    QApplication a(argc, argv);
    BallTracker tracker("DarkYellow");
    tracker.run();

    return a.exec();
}
