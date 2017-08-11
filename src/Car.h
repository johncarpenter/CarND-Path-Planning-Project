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
#ifndef CAR_H
#define CAR_H

#include <vector>

using namespace std;

namespace pathplanner{

class Car {
 public:
 
  double x_; 
  double y_; 
  double s_; 
  double d_; 
  double yaw_; 
  double speed_;
  int lane_; 
  
  // for other cars, distance to driving car
  double distance; 

  Car();
  virtual ~Car();


  int id;

  // Updates the position with know coordinates
  void update(double x, double y, double s, double d, double vx, double vy);

  

  // dumps the parameters to the console
  void dump(); 

};
}// namespace

#endif /* CAR_H */
