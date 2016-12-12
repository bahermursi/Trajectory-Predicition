#ifndef BALLTRACKER_H
#define BALLTRACKER_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <QTime>
using namespace std;
using namespace cv;
class BallTracker
{
public:
    BallTracker(string,string,bool cam = false);
    void run();
    void blur( Mat& image, Mat& blurredImage);
    void getContours( Mat&);
    Point estimateVelocity(Point , Point,bool normalized = false);
    void smoothNoise(Mat&);
    float distance2D(Point, Point);
    void getCenters(Point2f&,float&,int&,int&,int);
    void drawCircle(Mat&,Point2f& ,float&,int&,int&);
    void drawTrajectory(Mat&,vector<Point>&);
    void calculateTrajectory(Mat& frame,vector<Point>&,Point2f);
    void displayFrame(Mat&);
    void euler(Point& position, Point& velocity, Point2f& acceleration,float timeDelta);
    void getTrajectory(Point initialPosition,Point initialVelocity,Point2f& acceleration,float timeDelta,int numTrajPoints,vector<Point>& positions);
    void thresholding(const Mat& ,Mat& , Mat& );


private:

    int largest_area=0;
    int largest_contour_index=0;
    int numballs;
    float FPS;
    float ACCELERATION;
    float PIXEL_PER_METER;
    float timeStepSize;
    float TIME_STEP_PREC;
    float EULER_STEPS;
    float  gSeconds;
    float  gTimesteps;

vector<Point> pnts;
string videoFilename;
Point ballCenters = Point(0,0);
Point ballVelocities = Point(0,0);
map<string, pair<Scalar,Scalar>> colors;
vector<std::vector<Point>> contours;
vector<Vec4i> hierarchy;
bool openCam;
map<string, pair<Scalar,Scalar>>::iterator colorIter;

};

#endif // BALLTRACKER_H
