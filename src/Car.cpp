/**
* Copyright 2017 2Lines Software Inc
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*    http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
**/
#include "Car.h"
#include <iostream>

#include "Constants.h"
using namespace pathplanner;


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
