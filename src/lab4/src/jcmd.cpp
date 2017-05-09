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
    float t1, t2, t3, t;
    std::cin >> t1 >> t2 >> t3 >> t >> srv.request.type;
    srv.request.t1 = t1;
    srv.request.t2 = t2;
    srv.request.t3 = t3;
    srv.request.t = t;
    if(client.call(srv)){
      ROS_INFO("Service called, %s", srv.response.status.c_str());
    }else{
      ROS_ERROR("Failed to call service.");
    }
    ros::spinOnce();
  }
  return 0;
}
