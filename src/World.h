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

#endif /* WORLD_H */
