#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include "lab4/oint_control_srv.h"
#include <math.h>
#include <kdl/frames.hpp>
#include <iostream>

#define LOOP_RATE 30

using namespace std;
using namespace KDL;

enum ITYPE {linear, spline};
double old_pos[3];
double old_angle[3];
uint seq_axis_no = 0;
uint seq_path_no = 0;
uint seq_pose_no = 0;
ros::Publisher pose_stamped_pub;
ros::Publisher path_pub;

double calculate_interpolation(double x1, double x2, double t, double T){
  double ret;
  ret = x1+(x2-x1)*(t/T);
  return ret;
}


void fill_header(std_msgs::Header & header, uint& seq_no){
  header.seq = seq_no;
  header.frame_id="map";
  header.stamp = ros::Time::now();
  ++seq_no;
}

void fill_pos_message(geometry_msgs::Point &msg, double pos1, double pos2, double pos3){
  msg.x=pos1;
  msg.y=pos2;
  msg.z=pos3;
}

void fill_quat_message(geometry_msgs::Quaternion &msg, double angle[]){
  KDL::Rotation rot = KDL::Rotation::RPY(angle[0], angle[1], angle[2]);
  double x,y,z,w;
  rot.GetQuaternion(x,y,z,w);
  msg.x=x;
  msg.y=y;
  msg.z=z;
  msg.w=w;
}

void add_path_position(nav_msgs::Path &path, double pos1, double pos2, double pos3, double angle[]){
  geometry_msgs::PoseStamped pose_stamped;

  fill_header(pose_stamped.header, seq_pose_no);
  pose_stamped.pose.position.x = pos1;
  pose_stamped.pose.position.y = pos2;
  pose_stamped.pose.position.z = pos3;

  KDL::Rotation rot = KDL::Rotation::RPY(angle[0], angle[1], angle[2]);
  double x, y, z, w;
  rot.GetQuaternion(x,y,z,w);
  pose_stamped.pose.orientation.x=x;
  pose_stamped.pose.orientation.y=y;
  pose_stamped.pose.orientation.z=z;
  pose_stamped.pose.orientation.w=w;

  path.poses.push_back(pose_stamped);
}

bool interpolate(lab4::oint_control_srv::Request& request, lab4::oint_control_srv::Response& response)
{
  double pos[3];
  double angle[3];
  double quat[4]={0};
  double time_counter = 0;
  double time_end = 0;
  
  ROS_INFO("x=%f, y=%f, z=%f, roll=%f, pitch=%f, yaw=%f, time=%f", (float)request.x, (float)request.y, (float)request.z, (float)request.roll, (float)request.pitch, (float)request.yaw, (float)request.time);

  if(request.time <= 0){
    response.status = "Wrong time";
    return true;
  }
  pos[0] = request.x;
  pos[1] = request.y;
  pos[2] = request.z;
  angle[0] = request.roll;
  angle[1] = request.pitch;
  angle[2] = request.yaw;

  time_end = request.time;
  time_counter = 0;
  
  nav_msgs::Path path;
  ros::Rate loop_rate(30);
  while(time_counter < time_end)
  {
    double pos1, pos2, pos3;
    double ang[3];
    pos1 = calculate_interpolation(old_pos[0], pos[0], time_counter, time_end);
    pos2 = calculate_interpolation(old_pos[1], pos[1], time_counter, time_end);
    pos3 = calculate_interpolation(old_pos[2], pos[2], time_counter, time_end);
    
    ang[0] = calculate_interpolation(old_angle[0], angle[0], time_counter, time_end);
    ang[1] = calculate_interpolation(old_angle[1], angle[1], time_counter, time_end);
    ang[2] = calculate_interpolation(old_angle[2], angle[2], time_counter, time_end);
    
    geometry_msgs::PoseStamped msg;
    fill_header(msg.header, seq_axis_no);
    fill_pos_message(msg.pose.position, pos1, pos2, pos3);
    fill_quat_message(msg.pose.orientation, ang);

    fill_header(path.header, seq_path_no);
    add_path_position(path, pos1, pos2, pos3, ang);

    pose_stamped_pub.publish(msg);
    path_pub.publish(path);

    time_counter += 1/30.0;
    loop_rate.sleep();
  }
  
  response.status = "Sucess";
  old_pos[0] = pos[0];
  old_pos[1] = pos[1];
  old_pos[2] = pos[2];

  old_angle[0] = angle[0];
  old_angle[1] = angle[1];
  old_angle[2] = angle[2];
  return true;
}


int main(int argc, char **argv)
{
  old_pos[0] = 0; old_pos[1] = 0; old_pos[2] = 0;
  old_angle[0] = 0; old_angle[1] = 0; old_angle[2] = 0;
  ros::init(argc, argv, "oint_server");
  ros::NodeHandle n;
  
  pose_stamped_pub=n.advertise<geometry_msgs::PoseStamped>("pose_stamped",1);
  path_pub=n.advertise<nav_msgs::Path>("path",1);
  geometry_msgs::PoseStamped msg;

  fill_pos_message(msg.pose.position, 0, 0, 0);
  fill_quat_message(msg.pose.orientation, old_angle);
  pose_stamped_pub.publish(msg);

  ros::ServiceServer service = n.advertiseService("oint_control_srv", interpolate);

  ROS_INFO("Ready to interpolate.");
  ros::spin();
  return 0;
}
