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
#ifndef WORLD_H
#define WORLD_H

#include <vector>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <math.h>

#include "Constants.h"
#include "spline.h"

using namespace std;
namespace pathplanner{

class World {


   // Load up map values for waypoint's x,y,s and d normalized normal vectors
   std::vector<double> maps_x;
   std::vector<double> maps_y;
   std::vector<double> maps_s;
   std::vector<double> maps_dx;
   std::vector<double> maps_dy;

  // Interpolation splines, calculated at constructor instantiation

   tk::spline spline_x;
   tk::spline spline_y;
   tk::spline spline_dx;
   tk::spline spline_dy;
   
 public:
  World(const std::string &map_file_);
  ~World();

  int ClosestWaypoint(double x, double y);
  int NextWaypoint(double x, double y, double theta);
  std::vector<double> getFrenet(double x, double y, double theta);
  std::vector<double> getXY(double s, double d);
  std::vector<double> getXY_nearest(double s, double d);


};
} //namespace pathplanner
#endif /* WORLD_H */
