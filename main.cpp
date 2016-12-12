
#include <QApplication>
#include <opencv2/opencv.hpp>
#include "balltracker.h"
int main(int argc, char *argv[])
{

   //string colorNames = {"Yellow","Blue","Green","White","Yellow","Red","Orange","DarkYellow","DarkYellow2","LightGreen","Tennis"};

    QApplication a(argc, argv);
    BallTracker tracker("Red","/Users/bahermursi/GitHub/Trajectory-Predicition/juggling.mp4",1);
    tracker.run();

    return a.exec();
}
