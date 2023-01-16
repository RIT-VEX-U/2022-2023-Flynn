#include "../core/include/utils/math_util.h"

#ifndef PI
#define PI 3.141592654
#endif

/**
* Constrain the input between a minimum and a maximum value
* 
* @param val  the value to be restrained
* @param low  the minimum value that will be returned
* @param high the maximum value that will be returned
**/
double clamp(double val, double low, double high){
  if (val < low){
    return low;
  } else if (val > high){
    return high;
  }
  return val;
}
/**
* Returns the sign of a number
* @param x
* 
* returns the sign +/-1 of x. special case at 0 it returns +1
**/
double sign(double x){
  if (x<0){
    return -1;
  }
  return 1;
}


double wrap_angle_deg(double input)
{
  double angle = fmod(input, 360);
  if(angle < 0)
    angle += 360;

  return angle;
}

double wrap_angle_rad(double input)
{
  double angle = fmod(input, 2*PI);
  if(angle < 0)
    angle += 2*PI;

  return angle;
}