
#include "Car.h"
#include <iostream>

#include "Constants.h"

Car::Car() {
    lane_ = INITIAL_LANE; 
}
Car::~Car() {}

using namespace std;

void Car::update(double x, double y, double s, double d, double yaw, double speed){
    x_ = x; 
    y_ = y;
    s_ = checkMaxS(s);
    d_ = checkMaxD(d);
    lane_ = getLaneNumber(d_);
    yaw_ = yaw; 
    speed_ = speed; 

}


void Car::dump(){
    cout << "Car "<< id; 
    cout << " x:" << x_;
    cout << ",y:" << y_;
    cout << ",s:" << s_;
    cout << ",d:" << d_;
    cout << ",yaw:" << yaw_;
    cout << ",speed:" << speed_;
    cout << ",lane:" << lane_;
    cout << "\n";
    
}
