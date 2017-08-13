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
#include "PathPlanner.h"

#include "Constants.h"
using namespace pathplanner;
PathPlanner::PathPlanner(const std::string &map_file_) : map_(map_file_) {
	prev_v = 0;
	prev_s = 0;
	prev_d = 0;
	initialized=false;
}
PathPlanner::~PathPlanner() {}


void PathPlanner::updateCarPosition(double x, double y, double s, double d, double yaw, double speed){
		car_.update(x,y,s,d,0,0);
		car_.yaw_ = yaw;
		car_.speed_ = mph2ms(speed);
}

void PathPlanner::updateOtherCarPositions(int id, double x, double y, double s, double d, double vx, double vy){

  // ensure the car is available in the list
  if(other_cars_.find(id) == other_cars_.end()){
    other_cars_[id].id = id;
  }

  auto speed_theta = cartesian2polar(vx,vy);

  s = checkMaxS(s);

  other_cars_[id].update(x,y,s,d,speed_theta[1],speed_theta[0]);

  other_cars_[id].distance = distanceS(car_.s_,s);

}

void PathPlanner::generatePlan( const std::vector<double> &previous_path_x,
		const std::vector<double> &previous_path_y,double end_path_s,double end_path_d){

		if(!initialized){
			prev_s = car_.s_;
			prev_d = car_.d_;
			initialized = true;
			map_.padSplines();
		}

		// Set initial or use previous states
		next_x_vals.clear();
		next_y_vals.clear();


		int path_size = previous_path_x.size();
		for (int i = 0; i < path_size; i++)
		{
			next_x_vals.push_back(previous_path_x[i]);
			next_y_vals.push_back(previous_path_y[i]);
		}

		// Generate predictions from current state

		// Update behavior
		//car_.dump();

		trajectory_planner.generateGoals(map_,car_,findNearbyCars(car_.s_),prev_v,prev_s);

		interpolatePath();
}

/**
*
*
*/
void PathPlanner::interpolatePath(){

	double t = SAMPLING_RATE; // predict out t seconds
	vector<double> new_xy;
	while(next_x_vals.size() < 20)
	{

		trajectory_planner.jmt_v_time += t;
		prev_v = trajectory_planner.getDeltaV(trajectory_planner.jmt_v_time);

		prev_v = (prev_v > TARGET_MAX_VEL)?TARGET_MAX_VEL:prev_v;

		prev_s = prev_s + prev_v * t;

		prev_s = checkMaxS(prev_s);

		trajectory_planner.jmt_d_time += t;
		prev_d = trajectory_planner.getDeltaD(trajectory_planner.jmt_d_time);

		prev_d = checkMaxD(prev_d);

		//cout <<"s:"<<prev_s<<" d:"<<prev_d<<" v:"<<prev_v<<"\n";

		// Map to spline
		new_xy = map_.getXY(prev_s,prev_d);
		next_x_vals.push_back(new_xy[0]);
		next_y_vals.push_back(new_xy[1]);
	}


}


vector<Car> PathPlanner::findNearbyCars(double s_orig){

	vector<Car> cars;

	for (auto const& x : other_cars_)
	{
		Car t_car = x.second;

		if(t_car.distance < 100){
			cars.push_back(t_car);
		}
	}

	return cars;

}
