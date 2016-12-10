#include "attributescontainer.h"
#include <algorithm>
AttributesContainer::AttributesContainer(){    
}
void AttributesContainer::push(pointPair p1,pointPair p2,double dist){
    myData.push_back(MyPair(p1,p2,dist));
}

void AttributesContainer::pop(){
    myData.pop_back();
}
std::vector<AttributesContainer::MyPair> AttributesContainer::filterRepeatitions(){
    vector<Point> minMatches;
    for (int i = myData.size() - 1; i >= 0; --i) {
        bool equal = false;
        for (int j = 0; j < (int) minMatches.size(); ++j) {
            if (myData[i].p2.first == minMatches[j].p2.first) {
                equal = true;
                break;
            }
        }
        if (!equal)
            minMatches.push_back(myData[i]);
    }
}

void AttributesContainer::sortContainer(){
    std::sort(myData.begin(),myData.end(),less_than_dist());
}
