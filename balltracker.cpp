#include "balltracker.h"
using namespace std;

BallTracker::BallTracker(String ballColor,string filename,bool cam) :openCam(cam), numballs(1)
{
    colors["DarkGreen"] = make_pair(Scalar(35,50,20),Scalar(80,255,120));
    colors["Green"] = make_pair(Scalar(30,0,0),Scalar(100,255,255));
    colors["White"] = make_pair(Scalar(0,0,80),Scalar(255,50,120));
    colors["Yellow"] = make_pair(Scalar(15, 204, 204),Scalar(20, 255, 255));
    colors["Red"] = make_pair(Scalar(0, 153, 127),Scalar(4, 230, 179));
    colors["Orange"] =  make_pair(Scalar(15, 204, 204),Scalar(20, 255, 255));
    colors["DarkYellow"] = make_pair(Scalar(20, 115, 140),Scalar(25, 205, 230));
    colors["DarkYellow2"] = make_pair(Scalar(20, 90, 117),Scalar(32, 222, 222));
    colors["LightGreen"] = make_pair(Scalar(78, 77, 128),Scalar(81, 217, 179));
    colors["Tennis"] = make_pair(Scalar(40, 86, 6),Scalar(64, 255, 255));
    colors["Blue"] = make_pair(Scalar(110, 70, 100),Scalar(130, 255, 255));
    colors["TennisGreen"] = make_pair(Scalar(0, 103, 146),Scalar(179, 255, 255));

    videoFilename = filename;
    FPS = 30.0;
    ACCELERATION = 11;

    if(colors.find(ballColor) != colors.end()){
        colorIter = colors.find(ballColor);
    }
    else {
        colorIter = colors.find("Red"); //default
    }
}

void BallTracker::run(){
    VideoCapture cap;

    if (openCam){
        cap.open(0);
    }
    else {
        cap.open((videoFilename));
    }

    cap.set(CV_CAP_PROP_FRAME_WIDTH, 800);
    cap.set(CV_CAP_PROP_FRAME_HEIGHT, 600);
    Mat frame;
    while(cap.isOpened()){
        cap >> frame;

        Mat mask, hsv;
        vector<Point> positions;
        Point2f center;
        float radius = 0.0;
        int centerX= INT_MIN;
        int centerY = INT_MIN;

        thresholding(frame,mask,hsv);
        smoothNoise(mask);
        getContours(mask);
        if(largest_area > 0){
            getCenters(center,radius,centerX,centerY,largest_contour_index);
            drawCircle(frame,center,radius,centerX,centerY);
            calculateTrajectory(frame,positions,center);
            drawTrajectory(frame,positions);
        }
        displayFrame(frame);
    }
}


void BallTracker::getCenters(Point2f& center,float& radius,int& centerX,int& centerY,int largestInd){
    if (contours.size() > 0){
        minEnclosingCircle(contours[largestInd],center,radius);
        Moments M = moments(contours[largestInd],false);
        if(M.m00 != 0){
            centerX = int(M.m10 / M.m00);
            centerY = int(M.m01 / M.m00);
        }
    }
    contours.empty();
}

void BallTracker::blur( Mat& image, Mat& blurredImage){
    medianBlur(image,blurredImage, 5);
}

void BallTracker::thresholding(const Mat& frame,Mat& mask, Mat& hsv){
    frame.copyTo(hsv);
    GaussianBlur(hsv,hsv, Size(5,5),0);
    cvtColor(frame,hsv,COLOR_BGR2HSV);
    inRange(hsv, colorIter->second.first ,colorIter->second.second, mask);
}

void BallTracker::getContours(Mat& mask){
    largest_area=0;
    largest_contour_index=0;
    findContours(mask.clone(), contours, hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    for(unsigned i = 0; i< contours.size(); i++ )
    {
        double a = contourArea(contours[i],false);
        if(a > largest_area){
            largest_area = a;
            largest_contour_index = i;
        }
    }
}

void BallTracker::euler(Point& position, Point& velocity, Point2f& acceleration,float timeDelta){
    position.x += velocity.x * timeDelta;
    position.y += velocity.y * timeDelta;
    velocity.x += acceleration.x * timeDelta;
    velocity.y += acceleration.y * timeDelta;
}

void BallTracker::getTrajectory(Point initialPosition,Point initialVelocity,Point2f& acceleration,float timeDelta,int numTrajPoints,vector<Point>& positions){
    Point position = initialPosition;
    Point velocity = initialVelocity;
    for (int i = 0; i < numTrajPoints; ++i){
        euler(position, velocity, acceleration, timeDelta);
        positions.push_back(position);
    }
}

Point BallTracker::estimateVelocity(Point pnt1 , Point pnt2,bool normalized){
    if (normalized){
        double mag = sqrt(pow((pnt2.x - pnt1.x),2) + pow((pnt2.y - pnt1.y),2));
        return Point(int((pnt2.x - pnt1.x) / mag),int((pnt2.y - pnt1.y) / mag));
    }
    else{
        return Point((pnt2.x - pnt1.x),(pnt2.y - pnt1.y));
    }
}

void BallTracker::smoothNoise(Mat& mask){
    erode(mask, mask, 2 );
    dilate( mask, mask,2 );
}

float BallTracker::distance2D( Point p, Point q){
    return sqrt(pow((p.x - q.x),2) + pow((p.y - q.y),2));
}

void BallTracker::drawCircle(Mat& frame,Point2f& center,float& radius,int& centerX,int& centerY){
    if (radius > 10){
        if(centerX != INT_MIN || centerY != INT_MIN){
            circle(frame, center, int(radius),Scalar(0, 255, 255), 2);
            circle(frame, Point(centerX,centerY), 5, Scalar(0, 0, 255), -1);
        }
    }
}

void BallTracker::calculateTrajectory(Mat& frame,vector<Point>& positions,Point2f center){
    vector<Point>::iterator it = pnts.begin();
    pnts.insert (it , center);

    ballVelocities = estimateVelocity(ballCenters,center);
    ballCenters = center;
    circle(frame, center, 6, Scalar(200,0,0), 6);

    Point2f pf(0, ACCELERATION);
    getTrajectory(ballCenters, ballVelocities, pf, 0.100, 60, positions);
}

void BallTracker::drawTrajectory(Mat& frame,vector<Point>& positions){
    for (Point position : positions){
        if (position.x < frame.cols && position.y < frame.rows){
            circle(frame,position, 3, Scalar(255,55,55),2);
        }
    }
}

void BallTracker::displayFrame(Mat& frame){
    imshow("Image with Estimated Ball Center", frame);
    waitKey(FPS);
}
