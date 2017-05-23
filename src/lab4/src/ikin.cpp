#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <geometry_msgs/PoseStamped.h>
#include "lab4/oint_control_srv.h"
#include <math.h>

#define LOOP_RATE 30.0

using namespace std;

uint seq_joint_no = 0;
ros::Publisher joint_states_pub;


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


void pose_stamped_callback(const geometry_msgs::PoseStamped pose_stamped)
{
  double a1 = 1, a2 = 1;
  double x = pose_stamped.pose.position.x;
  double y = pose_stamped.pose.position.y;
  double z = pose_stamped.pose.position.z - a1;

  double r = sqrt(pow(x, 2) + pow(y, 2));
  
  double t2_1 = asin(a2/r);
  double t2_2 = atan(r/z);
  double t2 = t2_2;

  double t1_1 = acos(x/r);
  double t1_2 = acos(y/r);
  double t1 = t1_1;

  sensor_msgs::JointState joint_states;
  fill_header(joint_states.header, seq_joint_no);
  ROS_INFO("joints: %f, %f, %f", t1, t2, 0.0);
  fill_joint_message(joint_states, t1, t2, 0);
  //joint_states_pub.publish(joint_states);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "ikin");
  ros::NodeHandle n;

  joint_states_pub=n.advertise<sensor_msgs::JointState>("joint_states",100);
  ros::Subscriber sub = n.subscribe("pose_stamped", 1000, pose_stamped_callback);
  ros::Rate loop_rate(1);
  loop_rate.sleep();
  sensor_msgs::JointState msg;
  fill_header(msg.header, seq_joint_no);
  fill_joint_message(msg, 0, 0, 0);
  joint_states_pub.publish(msg);
  ros::spinOnce();
  ROS_INFO("Ready to interpolate.");
  ros::spin();
  return 0;
}
