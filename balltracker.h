#ifndef BALLTRACKER_H
#define BALLTRACKER_H
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <QTime>
#include "attributescontainer.h"
using namespace std;
using namespace cv;
class BallTracker
{
public:
    BallTracker();
    void run();
    void getFrame(VideoCapture&, Mat&);
    void blur(const Mat& image, Mat& blurredImage);
    void eulerExtrapolate(Point&, Point&, Point&,float);
    void getTrajectory(const Point initialPosition,const Point initialVelocity, pair<float,float>& acceleration,float timeDelta,
                       const vector<Point>& numTrajPoints,vector<Point>& positions);
    void getContours(const Mat&);
    void rejectOutlierPoints(const vector<Point>&,vector<Point>& , int m = 2);
    vector<int> estimateVelocity(Point , Point,bool normalized = false);
    void processForThresholding(const Mat&, Mat&);
    void smoothNoise(Mat&);
    void initializeBallStates(vector<Point>&,vector<Point>&);
    float distance4D(const Point p,const Point q, const Point r, const Point s);
    float distance2D(const Point p,const Point q);
    float calculateSD(float []);
    vector<Point> findBallsInImage(const Mat&, const vector<Point>&, const vector<Point>&);
   void drawBallsAndTrajectory(Mat& frameCopy, const  vector<AttributesContainer::MyPair>& matches, vector<Point>& ballCenters,
                                            vector<Point>& ballVelocities, const vector<Point>& ballIndices, const vector<Point>& ballCentersToPair, const vector<Point>& ballVelocitiesToPair);
    bool isEqual(Point p1, Point p2);
private:
    struct hsvRange{
        int lowH; int highH;
        int lowS; int highS;
        int lowV; int highV;
        hsvRange(int lh,int hh, int ls, int hs, int lv,int hv): lowH(lh), highH(hh), lowS(ls),highS(hs),lowV(lv), highV(hv){}
    };
    std::map<string,hsvRange*> colors;
    std::vector<std::vector<Point>> contours;
    std::vector<Vec4i> hierarchy;
    vector<Scalar> ballPositionMarkerColors = {Scalar(200,0,0), Scalar(255,200,200), Scalar(0,200,0), Scalar(0,0,200)};
    vector<Scalar> ballTrajectoryMarkerColors = {Scalar(200,55,55), Scalar(255,200,200), Scalar(55,255,55), Scalar(55,55,255)};
    Mat frame;
    int numballs;
    bool weightedFilter;
    bool averagedObservedVelocity;
    bool backgroundSubtraction;
    bool outlierRejection;

    float positionPredictionWeight;
    float positionObservationWeight;
    float velocityPredictionWeight;
    float velocityObservationWeight;
    float pixelsPerMeter;
    float FPS;
    float timeStepSize;
    float timeStepPrecision;
    float eulerSteps;
    float gSeconds;
    float gTimesteps;
    string videoFilename;

};

#endif // BALLTRACKER_H
