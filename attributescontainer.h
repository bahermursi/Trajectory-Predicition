#ifndef ATTRIBUTESCONTAINER_H
#define ATTRIBUTESCONTAINER_H
#include <opencv2/opencv.hpp>
using namespace std;
using namespace cv;
class AttributesContainer
{
    typedef std::pair<cv::Point,cv::Point> pointPair;

public:
    struct MyPair{
        pointPair p1;
        pointPair p2;
        double dist;
        MyPair(pointPair fp,pointPair sp, double d) : p1(fp),p2(sp),dist(d){}
        inline bool operator ==(const MyPair& rhs){
            return (this->p2 == rhs.p2);
        }
    };

    struct less_than_dist
    {
        inline bool operator() (const MyPair& struct1, const MyPair& struct2)
        {
            return (struct1.dist < struct2.dist);
        }
    };

    AttributesContainer();
    void pop();
    bool isMatch(const MyPair);
    std::vector<cv::Point> filterRepeatitions();
    void push(const pointPair,const pointPair,double);
    void sortContainer();

private:
    std::vector<MyPair> myData;
};

#endif // ATTRIBUTESCONTAINER_H
