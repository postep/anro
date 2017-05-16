#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Path.h>
#include "lab4/jint_control_srv.h"
#include <math.h>
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>


#define LOOP_RATE 30

using namespace std;
using namespace KDL;

enum ITYPE {linear, spline};
double old_joints[3];
uint seq_joint_no = 0;
uint seq_path_no = 0;
uint seq_pose_no = 0;
ros::Publisher joint_states_pub;
ros::Publisher path_pub;
KDL::Chain chain;

double calculate_interpolation(double x1, double x2, double t, double T, ITYPE interpolation){
  double ret;
  switch(interpolation)
  {
    case linear: ret = x1+(x2-x1)*(t/T); break;
  }
  return ret;
}


void fill_header(std_msgs::Header & header, uint& seq_no){
  header.seq = seq_no;
  header.frame_id="base_link";
  header.stamp = ros::Time::now();
  ++seq_no;
}

void fill_joint_message(sensor_msgs::JointState &msg, double j1, double j2, double j3){
  msg.name.push_back("base_link_cylinder_1");
  msg.name.push_back("cylinder_1_cylinder_2");
  msg.name.push_back("cylinder_2_cylinder_3");
  msg.position.push_back(j1);
  msg.position.push_back(j2);
  msg.position.push_back(j3);
}

void add_path_position(nav_msgs::Path &path, double j1, double j2, double j3){
  ChainFkSolverPos_recursive solver(chain);
  JntArray q(3);
  Frame F_results;
  q(0)=j1;
  q(1)=j2;
  q(2)=j3;   	
  solver.JntToCart(q, F_results);

  geometry_msgs::PoseStamped pose_stamped;
  fill_header(pose_stamped.header, seq_pose_no);
  pose_stamped.pose.position.x = F_results.p.data[0];
  pose_stamped.pose.position.y = F_results.p.data[1];
  pose_stamped.pose.position.z = F_results.p.data[2];
  double x, y, z, w;
  F_results.M.GetQuaternion(x, y, z, w);
  pose_stamped.pose.orientation.x=x;
  pose_stamped.pose.orientation.y=y;
  pose_stamped.pose.orientation.z=z;
  pose_stamped.pose.orientation.w=w;
  path.poses.push_back(pose_stamped);
}

bool interpolate(lab4::jint_control_srv::Request& request, lab4::jint_control_srv::Response& response)
{
  double joints[3];
  double time_counter = 0;
  double time_end = 0;
  
  ROS_INFO("request: t1=%f, t2=%f, t3=%f", (float)request.t1, (float)request.t2, (float)request.t3);
  ROS_INFO("interpolation type: %s", request.type.c_str());
    
  if(request.t1 < -M_PI || request.t1 > M_PI || request.t2 < -M_PI || request.t2 > M_PI || request.t3 < -M_PI || request.t3 > M_PI){
    response.status = "Wrong joints";
    return true;
  }
  if(request.t <= 0){
    response.status = "Wrong time";
    return true;
  }

  ITYPE interpolation;
  if(request.type == "linear"){
    interpolation = linear;
  }
  if(request.type == "spline"){
    interpolation = spline;
  }
  if(request.type != "spline" && request.type != "linear"){
    response.status = "Wrong interpolation type";
    return true;
  }
  
  joints[0] = request.t1;
  joints[1] = request.t2;
  joints[2] = request.t3;
  
  time_end = request.t;
  time_counter = 0;
  
  nav_msgs::Path path;
  ros::Rate loop_rate(30);
  while(time_counter < time_end)
  {
    double j1, j2, j3;
    j1 = calculate_interpolation(old_joints[0], joints[0], time_counter, time_end, interpolation);
    j2 = calculate_interpolation(old_joints[1], joints[1], time_counter, time_end, interpolation);
    j3 = calculate_interpolation(old_joints[2], joints[2], time_counter, time_end, interpolation);
    
    sensor_msgs::JointState msg;
    fill_header(msg.header, seq_joint_no);
    fill_joint_message(msg, j1, j2, j3);
    fill_header(path.header, seq_path_no);
    add_path_position(path, j1, j2, j3);
    joint_states_pub.publish(msg);
    path_pub.publish(path);
    time_counter += 1/30.0;
    loop_rate.sleep();
  }
  
  response.status = "Sucess";
  old_joints[0] = joints[0];
  old_joints[1] = joints[1];
  old_joints[2] = joints[2];
  return true;
}


int main(int argc, char **argv)
{
  old_joints[0] = 0; old_joints[1] = 0; old_joints[2] = 0;
  ros::init(argc, argv, "jint_server");
  ros::NodeHandle n;
  
  joint_states_pub=n.advertise<sensor_msgs::JointState>("joint_states",1);
  path_pub=n.advertise<nav_msgs::Path>("path",1);
  sensor_msgs::JointState msg;
  fill_joint_message(msg, 0, 0, 0);
  joint_states_pub.publish(msg);

  ros::ServiceServer service = n.advertiseService("oint_control_srv", interpolate);

	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0,0,1.0,0))));
	chain.addSegment(Segment(Joint(Joint::None),Frame(Frame::DH(0,-M_PI/2.0,0,0))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(1,0,0,-M_PI/2.0))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0,0,0,0))));		

  ROS_INFO("Ready to interpolate.");
  ros::spin();
  return 0;
}
