#ifndef CONSTANTS_H
#define CONSTANTS_H

#include <math.h>

// Simulator Constants
 const int NUMBER_OF_LANES = 3; 
 const double LANE_WIDTH = 4; 

 const double SAMPLING_RATE = 0.02; 

// Car Considerations

const double MAX_ACCEL = 10; // m/s^2
const double TARGET_ACCEL = 3; // m/s^2

const double MAX_JERK = 50; //m/s^3

const double LANE_CHANGE_TIME = 3; // time in s to change a lane

 // Reference initial parameters
 // Starts in the middle lane. 0 = left, 1 = center, 2 = right
 const int INITIAL_LANE = 1; 

inline double mph2ms(double mph){
  return mph * 0.44704; 
}
const double TARGET_MAX_VEL = mph2ms(46); // mph


// Environment Considerations
const double SPEED_LIMIT = mph2ms(50); // mph



inline double distance(double x1, double y1, double x2, double y2)
{
	return sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
}

inline std::vector<double> cartesian2polar(double vx, double vy) {
  double speed = sqrt(vx * vx + vy * vy);
  double theta = atan2(vy, vx);
  if(theta < 0) theta += 2 * M_PI;
  return {speed, theta};
}


// For converting back and forth between radians and degrees.
constexpr double pi() { return M_PI; }
inline double deg2rad(double x) { return x * pi() / 180; }
inline double rad2deg(double x) { return x * 180 / pi(); }

// The max s value before wrapping around the track back to 0
 const double max_s = 6945.554;
 inline double checkMaxS(double s){
	 if(s>= 0 && s < max_s) return s;

	 return fmod(s+ max_s, max_s);
 }

 inline double checkMaxD(double d){
   double max_d = NUMBER_OF_LANES*LANE_WIDTH;
	 if(d>= 0 && d <= max_d) return d;
	 if(d >= 0) return 0;
   if(d<= max_d) return max_d;

    return d;
 }



 inline int	getLaneNumber(double d){
	return int(d / LANE_WIDTH);
 }
 
 inline double getLaneCenter(int lane){
	return lane*LANE_WIDTH + LANE_WIDTH/2;
 }

 inline double adjustToCenterOfLane(double d){
  return getLaneCenter(getLaneNumber(d));
}

 inline double distanceS(const double from, const double to){
    //if(to < from ) return fabs(from - (to+max_s));
    double dist_f = fabs(from - (to+max_s));
    double dist_b = fabs(from - to);


    return (dist_f<dist_b)?dist_f:dist_b;

 }

 inline bool isInFront(double s_from, double s_to){
    double dist = fabs(s_from - (s_to+max_s));

    return (s_to > s_from) || (dist < 200);
  }

 /*
A function that returns a value between 0 and 1 for x in the
range [0, infinity] and -1 to 1 for x in the range [-infinity, infinity].
Useful for cost functions.
 */
inline double logistic(double x){

	return 2.0 / (1 + exp(-x)) - 1.0;

}


#endif /* CONSTANTS_H */
