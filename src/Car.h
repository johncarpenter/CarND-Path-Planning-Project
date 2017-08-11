#ifndef CAR_H
#define CAR_H

#include <vector>

using namespace std;

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

#endif /* CAR_H */
