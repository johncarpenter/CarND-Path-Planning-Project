#include "TrajectoryPlanner.h"

TrajectoryPlanner::TrajectoryPlanner(){
    adjustTargetSpeed(0,TARGET_MAX_VEL,0);
    adjustTargetLane(INITIAL_LANE,INITIAL_LANE,0);
};
TrajectoryPlanner::~TrajectoryPlanner(){};


void TrajectoryPlanner::generateGoals(World &map,Car car, vector<Car> nearby_cars, double prev_s, double prev_d, double prev_v ){

    BehaviorState behavior = getMinimumCostBehavior(car, nearby_cars);

        switch(behavior){
            case KL:
                applyKeepLaneBehavior(car,nearby_cars);
                return;
            case LCL:
                applyLaneChange(car.lane_-1,car,nearby_cars);
            break;
            case LCR:
                applyLaneChange(car.lane_+1,car,nearby_cars);
            break;
        }


}

BehaviorState TrajectoryPlanner::getMinimumCostBehavior(Car car, vector<Car> nearby_cars){

    vector<double> costs; 
    // Keep lane
    double kl_cost = logistic(TARGET_MAX_VEL/getLaneSpeed(car.lane_,car,nearby_cars)); 
    costs.push_back(kl_cost);
  
    double lcl_cost = 1;
    if(car.lane_ != 0){
        lcl_cost = calculateLaneChangeCost(car.lane_,car.lane_-1,car,nearby_cars);
    }
    costs.push_back(lcl_cost);

    double rcl_cost = 1;
    if(car.lane_ != 2){
        rcl_cost = calculateLaneChangeCost(car.lane_,car.lane_+1,car,nearby_cars);
    }
    costs.push_back(rcl_cost);

    int min_index = 0; 
    double min = 10; 
    for( int i=0;i<costs.size();i++){
        if(costs[i] < min){
            min_index = i; 
            min = costs[i];
        }
    }

    //cout << "KL: "<< kl_cost << " LCL: "<< lcl_cost << " RCL: "<< rcl_cost<<"\n";
    
    switch(min_index){
        case 0:
         return BehaviorState::KL;
        case 1:
         return BehaviorState::LCL;
        case 2:
         return BehaviorState::LCR;
        default:
         return BehaviorState::KL;
    }

}

double TrajectoryPlanner::calculateLaneChangeCost(int from_lane, int to_lane, Car car, vector<Car> nearby_cars){

    // check if the lane is free
    vector<Car> nearbyInLane = filterCarsByLane(nearby_cars,to_lane);
    sortCarsByDistance(nearbyInLane);
    if(nearbyInLane.size() > 0 && nearbyInLane[0].distance < 20) {
        return 1; 
    }

    return logistic(TARGET_MAX_VEL/getLaneSpeed(to_lane,car,nearby_cars))+0.05;

}


 double TrajectoryPlanner::getLaneSpeed(int lane, Car car,vector<Car> nearby_cars){

    vector<Car> infront = filterCarsInFrontOf(nearby_cars,car.s_);
    vector<Car> nearbyInLane = filterCarsByLane(infront,lane);
    sortCarsByDistance(nearbyInLane);
    if(nearbyInLane.size() > 0 && nearbyInLane[0].distance < 50) 
        return nearbyInLane[0].speed_;
    else
        return TARGET_MAX_VEL;
}

void TrajectoryPlanner::applyLaneChange(int to_lane, Car car, vector<Car> nearby_cars){
    if(to_lane != getLaneNumber(jmt_d.finish)){
        double lane_speed = getLaneSpeed(to_lane,car,nearby_cars);
        adjustTargetSpeed(car.speed_,lane_speed,0);
        adjustTargetLane(car.lane_,to_lane,0);
    }
    
   
}

void TrajectoryPlanner::applyKeepLaneBehavior(Car car, vector<Car> nearby_cars){
    double lane_speed = getLaneSpeed(car.lane_,car,nearby_cars);

    double targetDiff = fabs(lane_speed - jmt_v.finish);
    if(targetDiff > 0.5){
        adjustTargetSpeed(car.speed_,lane_speed,0);
    }    
}

double TrajectoryPlanner::getDeltaV(double t){

    if(jmt_v.duration <= t){
        return jmt_v.finish;
    }
    
    vector<double> jmt = jmt_v.jmt; 

    return   (jmt[0] + jmt[1]*t + jmt[2]*t*t + jmt[3]*t*t*t + jmt[4]*t*t*t*t + jmt[5]*t*t*t*t*t);
}

double TrajectoryPlanner::getDeltaD(double t){

    if(jmt_d.duration <= t){
        return adjustToCenterOfLane(jmt_d.finish);
    }

    vector<double> jmt = jmt_d.jmt; 

    return  (jmt[0] + jmt[1]*t + jmt[2]*t*t + jmt[3]*t*t*t + jmt[4]*t*t*t*t + jmt[5]*t*t*t*t*t);
}

void TrajectoryPlanner::adjustTargetSpeed(double currentSpeed, double targetSpeed, double deltaT){
       
        cout << "Adjusting Speed from: " << currentSpeed <<"m/s to :" << targetSpeed << " m/s \n";

       //targetSpeed = (targetSpeed > TARGET_MAX_VEL)? TARGET_MAX_VEL:targetSpeed; 

       double max = fabs(targetSpeed-currentSpeed)/TARGET_ACCEL;
       deltaT = (deltaT == 0 || deltaT > max)?max:deltaT;


       jmt_v.jmt =  JMT({currentSpeed,0,0},{targetSpeed,0,0},deltaT);
       jmt_v.start = currentSpeed; 
       jmt_v.finish = targetSpeed; 
       jmt_v.duration = deltaT;
       jmt_v.direction = (targetSpeed > currentSpeed)?1:-1;

       jmt_v_time = 0;
}

void TrajectoryPlanner::adjustTargetLane(int currentLane, int targetLane, double deltaT){

       cout << "Changing lane to :" << targetLane << " \n";

       jmt_d_time = 0; 

       if(currentLane == targetLane){
            deltaT = 0;
       }else{
            deltaT = (deltaT == 0 || deltaT > LANE_CHANGE_TIME)?LANE_CHANGE_TIME:deltaT;
       }  
       jmt_d.jmt =  JMT({getLaneCenter(currentLane),0,0},{getLaneCenter(targetLane),0,0},deltaT);
       jmt_d.start = getLaneCenter(currentLane); 
       jmt_d.finish = getLaneCenter(targetLane); 
       jmt_d.duration = deltaT;
       jmt_d.direction = (currentLane > targetLane)?1:-1;
}


void TrajectoryPlanner::sortCarsByDistance(vector<Car> &cars){

	sort( cars.begin( ), cars.end( ), [&]( const Car& lhs, const Car& rhs )
	{
		return lhs.distance < rhs.distance;
	});
}

vector<Car> TrajectoryPlanner::filterCarsInFrontOf(vector<Car> cars, double s){

	vector<Car> filtered; 
	copy_if(cars.begin(), cars.end(),
				std::back_inserter(filtered),
				[&](const Car& car) { return isInFront(s,car.s_); });
	return filtered;

}

vector<Car> TrajectoryPlanner::filterCarsByLane(vector<Car> cars, int lane){

	vector<Car> filtered; 
	copy_if(cars.begin(), cars.end(),
				std::back_inserter(filtered),
				[&](const Car& car) { return car.lane_ == lane; });
	return filtered;

}

vector<double> TrajectoryPlanner::JMT(vector< double> start, vector <double> end, double T)
{
  MatrixXd A = MatrixXd(3, 3);
	A << T*T*T, T*T*T*T, T*T*T*T*T,
			    3*T*T, 4*T*T*T,5*T*T*T*T,
			    6*T, 12*T*T, 20*T*T*T;

	MatrixXd B = MatrixXd(3,1);
	B << end[0]-(start[0]+start[1]*T+.5*start[2]*T*T),
			    end[1]-(start[1]+start[2]*T),
			    end[2]-start[2];

	MatrixXd Ai = A.inverse();

	MatrixXd C = Ai*B;

	vector <double> result = {start[0], start[1], .5*start[2]};
	for(int i = 0; i < C.size(); i++)
	{
	    result.push_back(C.data()[i]);
	}

    return result;

}

