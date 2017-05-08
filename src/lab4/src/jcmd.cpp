#include "ros/ros.h"
#include "lab4/jint_control_srv.h"
#include <iostream>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "jint_server");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<lab4::jint_control_srv>("oint_control_srv");
  while(ros::ok())
  {
    lab4::jint_control_srv srv;
    double t1, t2, t3, t;
    int type;
    std::cin >> t1 >> t2 >> t3 >> t >> type;
    srv.request.t1 = t1;
    srv.request.t2 = t2;
    srv.request.t3 = t3;
    srv.request.t = t;
    srv.request.type = type;
    if(client.call(srv)){
      if(srv.response.result){
        ROS_INFO("Service called, success.");
      }else{
        ROS_INFO("Service called, fail.");
      }
    }else{
      ROS_ERROR("Failed to call service.");
    }
    ros::spinOnce();
  }
  return 0;
}
