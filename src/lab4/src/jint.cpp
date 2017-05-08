#include "ros/ros.h"
#include "lab4/jint_control_srv.h"
#include "sensor_msgs/JointState.h"

#define LOOP_RATE 30

double old_joints[3];
double joints[3];
double time_counter = 0;
double time_end = 0;
bool interpolate(lab4::jint_control_srv::Request& request, lab4::jint_control_srv::Response& response)
{
  ROS_INFO("request: t1=%f, t2=%f, t3=%f", (float)request.t1, (float)request.t2, (float)request.t3);
  if(time_counter < time_end){
    response.result = false;
    return true;
  }
  old_joints[0] = joints[0];
  old_joints[1] = joints[1];
  old_joints[2] = joints[2];
  joints[0] = request.t1;
  joints[1] = request.t2;
  joints[2] = request.t3;
  time_end = request.t;
  time_counter = 0;
  response.result = true;
  return true;
}

double calculate_interpolation(double x1, double x2, double t, double T){
  return x1+(x2-x1)*(t/T);
}
int main(int argc, char **argv)
{
  old_joints[0] = 0; old_joints[1] = 0; old_joints[2] = 0;
  joints[0] = 0; joints[1] = 0; joints[2] = 0;
  
  ros::init(argc, argv, "jint_server");
  ros::NodeHandle n;
  uint seq_no = 0;
  ros::ServiceServer service = n.advertiseService("jint_position", interpolate);
  ros::Publisher joint_states_pub=n.advertise<sensor_msgs::JointState>("joint_states",1);
  ROS_INFO("Ready to interpolate.");
  ros::Rate loop_rate(30);
  while(ros::ok())
  {
    sensor_msgs::JointState msg;
    msg.header.seq = seq_no;
    msg.header.frame_id="";
    msg.header.stamp = ros::Time::now();
    msg.name.push_back("base_link_cylinder_1");
    msg.name.push_back("cylinder_1_cylinder_2");
    msg.name.push_back("cylinder_2_cylinder_3");
    if(time_counter < time_end){
      msg.position.push_back(calculate_interpolation(old_joints[0], joints[0], time_counter, time_end));
      msg.position.push_back(calculate_interpolation(old_joints[1], joints[1], time_counter, time_end));
      msg.position.push_back(calculate_interpolation(old_joints[2], joints[2], time_counter, time_end));
      time_counter += 1/30.0;
    }else{
      msg.position.push_back(joints[0]);
      msg.position.push_back(joints[1]);
      msg.position.push_back(joints[2]);
    }
    joint_states_pub.publish(msg);
    seq_no++;
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
