#include "balltracker.h"
#include <tuple>
////include "opencv2/bgsegm.hpp"
using namespace std;

BallTracker::BallTracker() : numballs(3)
{
    colors["darkGreen"] = new hsvRange(35,50,20,80,255,120);
    colors["green"] = new hsvRange(30,0,0,100,255,255);
    colors["white"] = new hsvRange(0,0,80,255,50,120);
    colors["yellow"] = new hsvRange(15, 204, 204,20, 255, 255);
    colors["red"] = new hsvRange(0, 153, 127,4, 230, 179);
    colors["orange"] =  new hsvRange(15, 204, 204,20, 255, 255);
    colors["darkYellow"] = new hsvRange(20, 115, 140,25, 205, 230);
    colors["darkYellowAttempt2(isolating)"] = new  hsvRange(20, 90, 117,32, 222, 222);
    colors["orange2"] = new hsvRange(2, 150, 140,19, 255, 204);

    weightedFilter = false;
    positionPredictionWeight = 0.2;
    positionObservationWeight = 0.8;
    velocityPredictionWeight = 0.2;
    velocityObservationWeight = 0.8;
    averagedObservedVelocity = false;
    backgroundSubtraction = false;
    outlierRejection = false;



    videoFilename = "juggling.mp4";

    pixelsPerMeter = 700.0; // Just a guess from looking at the video (juggling.mp4)
    // pixelsPerMeter = 980.0 // Just a guess from looking at the video (juggling2.mp4)
    FPS = 30.0;

    // Euler's method will proceed by timeStepSize / timeStepPrecision at a time
    timeStepSize = 1.0 / FPS;
    timeStepPrecision = 1.0;

    // Number of Euler's method steps to take
    eulerSteps = 18;

    // Gravitational acceleration is in units of pixels per second squared
    gSeconds = 9.81 * pixelsPerMeter;
    // Per-timestep gravitational acceleration (pixels per timestep squared)
    gTimesteps = gSeconds * (timeStepSize*timeStepSize);
}

void BallTracker::run(){

}

void BallTracker::getFrame(VideoCapture& cap, Mat& frame){
    cap >> frame;
}

void BallTracker::blur(const Mat& image, Mat& blurredImage){
    medianBlur(image,blurredImage, 5);
}
//position, velocity, acceleration, timeDelta
void BallTracker::eulerExtrapolate(Point& position, Point& velocity, pair<float,float>& acceleration,float timeDelta){
    position.x += velocity.x * timeDelta;
    position.y += velocity.y * timeDelta;
    velocity.x += acceleration.first * timeDelta;
    velocity.y += acceleration.second * timeDelta;
}

//def getTrajectory(initialPosition, initialVelocity, acceleration, timeDelta, numTrajPoints):
//getTrajectory((centerX, centerY), (velocityX, velocityY), (0, gTimesteps), timeStepSize, eulerSteps)
void BallTracker::getTrajectory( Point initialPosition, Point initialVelocity, Point& acceleration,float timeDelta, const vector<Point>& numTrajPoints,vector<Point>& positions ){
    positions.push_back(initialPosition);
    Point velocity = Point(initialVelocity.x,initialVelocity.y);
    for (Point p : numTrajPoints){
        eulerExtrapolate(p, velocity, acceleration, 1);
        for (Point p : positions){
            positions.push_back(p);
        }
    }
}

void BallTracker::getContours(const Mat& mask){
    findContours(mask.clone(), contours, hierarchy,CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
}

void BallTracker::rejectOutlierPoints(const vector<Point>& points,vector<Point>& nonOutliers, int m){
    if (points.size() == 0)
        return;
    else{
        double meanX{0};
        double meanY{0};
        for(Point p : points){
            meanX += p.x;
            meanY+=p.y;
        }
        meanX /=points.size();
        meanY/=points.size();

        double accumX = 0.0;
        double accumY = 0.0;
        std::for_each (std::begin(points), std::end(points), [&](const Point p) {
            accumX += (p.x - m) * (p.x - m);
            accumY += (p.y - m) * (p.y - m);
        });
        double stdevX = sqrt(accumX / (points.size()-1));
        double stdevY = sqrt(accumY / (points.size()-1));
        for(Point pnt : points){
            if ((abs(pnt.x - meanX) < stdevX * m) && (abs(pnt.y - meanY) < stdevY * m)){
                nonOutliers.push_back(pnt);
            }
        }
    }
}


vector<int> BallTracker::estimateVelocity(Point pnt1 , Point pnt2,bool normalized){
    if (normalized){
        double mag = sqrt(pow((pnt2.x - pnt1.x),2) + pow((pnt2.y - pnt1.y),2));
        return {int((pnt2.x - pnt1.x) / mag),int((pnt2.y - pnt1.y) / mag)};
    }
    else{
        return {(pnt2.x - pnt1.x),(pnt2.y - pnt1.y)};
    }
}

void BallTracker::processForThresholding(const Mat& frame, Mat& hsvBlurredFrame){
    //     Mat blurredFrame;
    //     blur(frame,blurredFrame);

    if (backgroundSubtraction){
        // Subtract background (makes isolation of balls more effective, in combination with thresholding)
        BackgroundSubtractorMOG mog;
        Mat fgMaskMOG;
        mog(frame, fgMaskMOG);
        bitwise_and(frame,frame,frame,fgMaskMOG);
    }
    // Convert to HSV color space
    cvtColor(frame,hsvBlurredFrame, COLOR_BGR2HSV);
}

void BallTracker::smoothNoise(Mat& src){
    erode(src, src, Mat());
    dilate(src, src, Mat());
}

void BallTracker::initializeBallStates(vector<Point>& ballCenters,vector<Point>& ballVelocities){
    for(int i = 0; i < numballs; ++i){
        ballCenters.push_back(Point(0,0));
        ballVelocities.push_back(Point(0,0));
    }
}

float BallTracker::distance4D(const Point p,const Point q, const Point r, const Point s){
    return sqrt(pow((p.x - q.x),2) + pow((p.y - q.y),2) + pow((r.x - s.x),2) + pow((r.y - s.y),2));
}

float BallTracker::distance2D(const Point p,const Point q){
    return sqrt(pow((p.x - q.x),2) + pow((p.y - q.y),2));

}
vector<Point> BallTracker:: findBallsInImage(const Mat& image, const vector<Point>& ballCenters, const vector<Point>& ballVelocities){

    unsigned numBallsToFind = ballCenters.size();
    vector<Point> unFilteredpoints;
    vector<Point> points;
    // Get a list of all of the non-blank points in the image
    for(int row = 0; row < image.rows; ++row)
        for(int col = 0; col < image.cols; ++col)
            if(image.at<uchar>(row,col) != 0)
                unFilteredpoints.push_back(Point(row,col));

    if (outlierRejection){
        // Filter out positional outliers
        rejectOutlierPoints(unFilteredpoints,points);
    }

    if (points.size() == 0)
        return{};

    vector<int> labels;
    Mat centers;
    if (points.size() >= numBallsToFind){

        // Break into clusters using k-means clustering

        TermCriteria criteria = TermCriteria( TermCriteria::EPS + TermCriteria::MAX_ITER, 10, 1.0);
        double compactness = kmeans(points,numBallsToFind,labels, criteria, 10, KMEANS_RANDOM_CENTERS,centers);

        AttributesContainer pairings;
        vector<pair<Point,Point>> ballAttr;
        unsigned minLength = min(ballCenters.size(),ballVelocities.size());
        for(unsigned i = 0; i < minLength; ++i){
            ballAttr.push_back(make_pair(ballCenters[i],ballVelocities[i]));
        }
        for (unsigned i = 0; i < ballAttr.size(); i++){
            for (unsigned j = 0; j < centers.cols; j++){
                Point ballCenter = ballAttr[i].first;
                Point ballVelocity = ballAttr[i].second;
                Point center = centers.at<Point>(0,j);
                Point theoreticalVelocity = Point(center.x - ballCenter.x, center.y - ballCenter.y );
                double distance = distance4D(ballCenter, center, theoreticalVelocity, theoreticalVelocity);
                pairings.push(make_pair(ballCenter,ballVelocity), make_pair(center, theoreticalVelocity), distance);
            }
        }
        pairings.sortContainer();
        vector<AttributesContainer::MyPair> min_matches = pairings.filterRepeatitions();
        return min_matches;
    }
    else
        return {};

}
bool BallTracker::isEqual(Point p1, Point p2){
    return p1.x == p2.x && p1.y == p2.y;
}

void BallTracker::drawBallsAndTrajectory(Mat& frameCopy, const  vector<AttributesContainer::MyPair>& matches, vector<Point>& ballCenters, vector<Point>& ballVelocities, const vector<Point>& ballIndices, const vector<Point>& ballCentersToPair, const vector<Point>& ballVelocitiesToPair){
    if (matches.size() == 0)
        return ;
    int globalIndex;
    map<int,bool> matchedIndices;
    for ( AttributesContainer::MyPair match : matches){
        bool matched = false;
        unsigned minLength = min(ballCentersToPair.size(),ballVelocitiesToPair.size());
        for(unsigned i = 0; i < minLength; ++i){
            if (isEqual(match.p1.first,ballCentersToPair[i]) && isEqual(match.p2.second,ballCentersToPair[i]) && (matchedIndices.find(i) == matchedIndices.end())  && !matched){
                globalIndex = ballIndices[i];
                matchedIndices[globalIndex] = true;
                Point previousPosition = ballCenters[globalIndex];
                Point previousVelocity = ballVelocities[globalIndex];
                Point observedPosition = match.p2.first;
                Point observedVelocity = Point(observedPosition.x - previousPosition.x, observedPosition.y - previousPosition.y);
                if (averagedObservedVelocity)
                    observedVelocity = Point((observedPosition.x + ballVelocities[globalIndex].x) / 2.0, (observedVelocity.y + ballVelocities[globalIndex].y) / 2.0);
                if (weightedFilter){
                    // Predict uncertainty for this timestep
                    Point predictedPosition = Point(previousPosition.x + previousVelocity.x, previousPosition.y + previousVelocity.y);
                    Point predictedVelocity = Point(previousVelocity.x, previousVelocity.y + gTimesteps);
                    // Update estimated state
                    ballCenters[globalIndex] = Point(predictedPosition.x*positionPredictionWeight + observedPosition.x*positionObservationWeight, predictedPosition.y*positionPredictionWeight + observedPosition.y*positionObservationWeight);
                    ballVelocities[globalIndex] = Point(predictedVelocity.x*velocityPredictionWeight + observedVelocity.x*velocityObservationWeight, predictedVelocity.y*velocityPredictionWeight + observedVelocity.y*velocityObservationWeight);

                }
                else{
                    // Just use observed positions and velocities
                    ballCenters[globalIndex] = observedPosition;
                    ballVelocities[globalIndex] = observedVelocity;
                }
                matchedIndices[i] = true;
                matched = true;

            }
        }
    }
    vector<Point> positions;
    for (int i = 0; i < ballIndices.size(); ++i){
        Point center = Point(ballCenters[i].x, ballCenters[i].y);
        Point velocity = Point(ballVelocities[i].x,ballVelocities[i].y);
        circle(frameCopy,center, 6, ballPositionMarkerColors[i], 6);
        //  getTrajectory(const Point initialPosition,const Point initialVelocity, Point& acceleration,float timeDelta,
        //  const vector<Point>& numTrajPoints,);
        getTrajectory(center, velocity, pair<0.0, gTimesteps>, timeStepSize, eulerSteps,positions);

        for( Point position : positions){
            // height, width, depth = frameCopy.shape
            if (position.x < frameCopy.cols && position.y < frameCopy.rows)
                circle(frame, position, 2, ballTrajectoryMarkerColors[i], 2);
        }
        // Draw velocity vectors
        //cv2.arrowedLine(frameCopy, (int(centerX), int(centerY)), (int(ballCenters[i][0]+ballVelocities[i][0]*2), int(ballCenters[i][1]+ballVelocities[i][1]*2)), ballTrajectoryMarkerColors[i], 2, 2, 0, 0.1)
    }


}
