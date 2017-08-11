#ifndef PATHPLANNER_H
#define PATHPLANNER_H

#include <map>
#include <vector>
#include <string>


#include "World.h"
#include "Car.h"
#include "TrajectoryPlanner.h"

using namespace std;



class PathPlanner {
 private:
   double prev_s;
  double prev_d;
  double prev_v; 

 public:
  // Coordinates and mapping for the drive space
  World map_;

  // Current car
  Car car_; 

  // Other cars
  std::map<int, Car> other_cars_;

  TrajectoryPlanner trajectory_planner;

  vector<double> next_x_vals;
  vector<double> next_y_vals;




  PathPlanner(const string &map_file_);

  ~PathPlanner();

  void updateOtherCarPositions(int id, double x, double y, double s, double d, double vx, double vy);
  void updateCarPosition(double x, double y, double s, double d, double yaw, double speed);

  void generatePlan( const std::vector<double> &previous_path_x,
		const std::vector<double> &previous_path_y,double end_path_s,double end_path_d);
  void interpolatePath();   
  

  vector<Car> findNearbyCars(double s_orig);


};

#endif /* PATHPLANNER_H */
