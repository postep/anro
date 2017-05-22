#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include "lab4/oint_control_srv.h"
#include <math.h>
#include <kdl/frames.hpp>
#include <iostream>
#include <ecl/geometry.hpp>


#define LOOP_RATE 30

using namespace std;
using namespace ecl;


enum ITYPE {linear, spline};
double old_pos[3];
uint seq_axis_no = 0;
uint seq_path_no = 0;
uint seq_pose_no = 0;
ros::Publisher pose_stamped_pub;
ros::Publisher path_pub;

double calculate_interpolation(double x1, double x2, double t, double T, ITYPE interpolation){
  double ret;
  switch(interpolation)
  {
    case linear: {
      ret = x1+(x2-x1)*(t/T); 
      break;
    }
    case spline: {
      double x11 = x1 + (x2-x1)*0.2;
      double x22 = x1 + (x2-x1)*0.8;
      Array<double> x_arr(4);
      Array<double> y_arr(4);
      x_arr << 0, 0.4*T, 0.6*T, T;
      y_arr << x1, x11, x22, x2;
      double max_curvature = 1.0;
      SmoothLinearSpline spline(x_arr, y_arr, max_curvature);
      ret = spline(t);
      break;
    }
  } 
  return ret;
}


void fill_header(std_msgs::Header & header, uint& seq_no){
  header.seq = seq_no;
  header.frame_id="base_link";
  header.stamp = ros::Time::now();
  ++seq_no;
}

void fill_pos_message(geometry_msgs::Point &msg, double pos1, double pos2, double pos3){
  msg.x=pos1;
  msg.y=pos2;
  msg.z=pos3;
}


void add_path_position(nav_msgs::Path &path, double pos1, double pos2, double pos3){
  geometry_msgs::PoseStamped pose_stamped;

  fill_header(pose_stamped.header, seq_pose_no);
  pose_stamped.pose.position.x = pos1;
  pose_stamped.pose.position.y = pos2;
  pose_stamped.pose.position.z = pos3;

  path.poses.push_back(pose_stamped);

  pose_stamped.pose.orientation.x = 0;
  pose_stamped.pose.orientation.y = 0;
  pose_stamped.pose.orientation.z = 0;
  pose_stamped.pose.orientation.w = 0;
}

bool interpolate(lab4::oint_control_srv::Request& request, lab4::oint_control_srv::Response& response)
{
  double pos[3];
  double time_counter = 0;
  double time_end = 0;


  ROS_INFO("x=%f, y=%f, z=%f, time=%f", (float)request.x, (float)request.y, (float)request.z, (float)request.time);

  if(request.time <= 0){
    response.status = "Wrong time";
    return true;
  }
  if(request.type != "linear" && request.type != "spline"){
    response.status = "Wrong interpolation type";
    return true;
  }
  ITYPE type;
  if(request.type == "linear"){
    type = linear;
  }
  if(request.type == "spline"){
    type = spline;
  }

  pos[0] = request.x;
  pos[1] = request.y;
  pos[2] = request.z;

  time_end = request.time;
  time_counter = 0;
  
  nav_msgs::Path path;
  ros::Rate loop_rate(30);
  while(time_counter < time_end)
  {
    double pos1, pos2, pos3;
    double ang[3];
    pos1 = calculate_interpolation(old_pos[0], pos[0], time_counter, time_end, type); //x
    pos2 = calculate_interpolation(old_pos[1], pos[1], time_counter, time_end, type); //y

    double delta = 4-4*(1-sqrt(1-pow(pos1, 2) - pow(pos2, 2)));
    double rozw;
    rozw = (2 + sqrt(delta))/2;
    if (rozw>0){
      pos3=rozw;
    }
    else {
      pos3 = (2 - sqrt(delta))/2;
    }

    geometry_msgs::PoseStamped msg;
    fill_header(msg.header, seq_axis_no);
    fill_pos_message(msg.pose.position, pos1, pos2, pos3);

    fill_header(path.header, seq_path_no);
    add_path_position(path, pos1, pos2, pos3);

    pose_stamped_pub.publish(msg);
    path_pub.publish(path);

    time_counter += 1/30.0;
    loop_rate.sleep();
  }
  
  response.status = "Sucess";
  old_pos[0] = pos[0];
  old_pos[1] = pos[1];
  old_pos[2] = pos[2];
  return true;
}


int main(int argc, char **argv)
{
  old_pos[0] = 0; old_pos[1] = 0; old_pos[2] = 0;
  ros::init(argc, argv, "oint_server");
  ros::NodeHandle n;
  
  pose_stamped_pub=n.advertise<geometry_msgs::PoseStamped>("pose_stamped", 1);
  path_pub=n.advertise<nav_msgs::Path>("path",1);
  geometry_msgs::PoseStamped msg;

  ros::Rate loop_rate(1);
  loop_rate.sleep();
  fill_header(msg.header, seq_axis_no);
  fill_pos_message(msg.pose.position, 0, 0, 0);
  pose_stamped_pub.publish(msg);

  ros::ServiceServer service = n.advertiseService("oint_control_srv", interpolate);

  ROS_INFO("Ready to interpolate.");
  ros::spin();
  return 0;
}
