#include <math.h>
#include <ecl/geometry.hpp>
#include <ros/ros.h>


#ifndef LIBRARY
#define LIBRARY

enum ITYPE {linear, spline};

bool equal_eps(double a, double b){
  double eps = 0.05;
  return (a + eps >= b) && (a - eps <= b);
}

bool calculate_inverse_kinematic(double x, double y, double z, double& t1, double& t2, double& t3){
  double a1 = 1;
  double a2 = 1;
  double r = sqrt(pow(x, 2) + pow(y, 2));
  z -= a1;
  if(!equal_eps(sqrt(x*x + y*y + z*z), a2) || z < 0 || z > 1 || x < -1 || x > 1 || y < -1 || y > 1){
    return false;
  }
  
  t2 = acos(r/a2);

  double t1_1 = acos(x/r);
  if(r == 0){
    t1 = 0;
  }else{
    t1 = t1_1;
  }

  if(y < 0){
    t1 = -t1;
  }

  t3 = 0;

  return true;
}

double calculate_interpolation(double x1, double diff, double t, double T, ITYPE interpolation){
  double ret;
  double x2 = x1 + diff;
  switch(interpolation)
  {
    case linear: {
      ret = x1+diff*(t/T); 
      break;
    }
    case spline: {
      double x11 = x1 + diff*0.2;
      double x22 = x1 + diff*0.8;
      ecl::Array<double> x_arr(4);
      ecl::Array<double> y_arr(4);
      x_arr << 0, 0.4*T, 0.6*T, T;
      y_arr << x1, x11, x22, x2;
      double max_curvature = 50.0;
      try{
        ecl::SmoothLinearSpline spline(x_arr, y_arr, max_curvature);
        ret = spline(t);
      }catch(...){
        ROS_ERROR("Max curvature limitation exceeded");
      }
      break;
    }
  } 
  return ret;
}

double calculate_joint_interpolation(double x1, double x2, double t, double T, ITYPE interpolation){
  double diff = x2-x1;
  if(diff > M_PI){
    diff = 2*M_PI-diff;
    diff = -diff;
  }else if(diff < -M_PI){
    diff += 2*M_PI;
  }

  double i = calculate_interpolation(x1, diff, t, T, interpolation);
  if(i < -M_PI){
    i = M_PI + i + M_PI;
  }
  if(i > M_PI){
    i -= 2*M_PI;
  }
  return i;
}

void calculate_kinematic(double &x, double &y, double &z, double t1, double t2, double t3){
  double a1 = 1, a2 = 1;
  double r = a2*cos(t2);
  z = a2 * sin(t2) + 1;
  x = r * cos(t1);
  y = r * sin(t1);
}


#endif