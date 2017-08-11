#ifndef TRAJECTORYPLANNER_H
#define TRAJECTORYPLANNER_H

#include <vector>
#include <iostream>

#include "World.h"
#include "Car.h"
#include "Constants.h"

#include "Eigen-3.3/Eigen/Core"
#include "Eigen-3.3/Eigen/Dense"
using Eigen::MatrixXd;
using Eigen::VectorXd;

using namespace std;

struct Trajectory{
    vector<double> jmt; 
    float direction; 
    double start; 
    double finish; 
    double duration;
};

enum BehaviorState {KL, LCL, LCR};


class TrajectoryPlanner {
private:

  

public:

    TrajectoryPlanner();
    virtual ~TrajectoryPlanner();

    Trajectory jmt_v;
    double jmt_v_time; 
    
    Trajectory jmt_d; 
    double jmt_d_time; 
    

    vector<BehaviorState> getPossibleStates(Car &car);

    void generateGoals(World &map, Car car, vector<Car> nearby_cars, double prev_s, double prev_d, double prev_v);

    BehaviorState getMinimumCostBehavior(Car car, vector<Car> nearby_cars);
    double calculateLaneChangeCost(int from_lane, int to_lane, Car car, vector<Car> nearby_cars);
    void applyKeepLaneBehavior(Car car, vector<Car> nearby_cars);
    void applyLaneChange(int to_lane, Car car, vector<Car> nearby_cars);

    double getLaneSpeed(int lane, Car car, vector<Car> nearby_cars);
    void sortCarsByDistance(vector<Car> &cars);
    vector<Car> filterCarsInFrontOf(vector<Car> cars, double s);
    vector<Car> filterCarsByLane(vector<Car> cars, int lane);
    void adjustTargetSpeed(double currentSpeed, double targetSpeed,double deltaT);
    void adjustTargetLane(int currentLane, int targetLane, double deltaT);
    double getDeltaV(double t);
    double getDeltaD(double t);
    /*
    Calculate the Jerk Minimizing Trajectory that connects the initial state
    to the final state in time T.

    INPUTS

    start - the vehicles start location given as a length three array
        corresponding to initial values of [s, s_dot, s_double_dot]

    end   - the desired end state for vehicle. Like "start" this is a
        length three array.

    T     - The duration, in seconds, over which this maneuver should occur.

    OUTPUT
    an array of length 6, each value corresponding to a coefficent in the polynomial
    s(t) = a_0 + a_1 * t + a_2 * t**2 + a_3 * t**3 + a_4 * t**4 + a_5 * t**5

    EXAMPLE

    > JMT( [0, 10, 0], [10, 10, 0], 1)
    [0.0, 10.0, 0.0, 0.0, 0.0, 0.0]
    */
    vector<double> JMT(vector< double> start, vector <double> end, double T);
 
};

#endif /* TRAJECTORYPLANNER_H */