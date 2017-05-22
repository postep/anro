#include "ros/ros.h"
#include "lab4/oint_control_srv.h"
#include <iostream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "oint_server");
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<lab4::oint_control_srv>("oint_control_srv");
  while(ros::ok())
  {
    lab4::oint_control_srv srv;
    float x, y, z, time;
    string type;
    std::cin >> x >> y >> z >> time >> type;
    srv.request.x = x;
    srv.request.y = y;
    srv.request.z = z;
    srv.request.time = time;
    srv.request.type = type;
    if(client.call(srv)){
      ROS_INFO("Service called, %s", srv.response.status.c_str());
    }else{
      ROS_ERROR("Failed to call service.");
    }
    ros::spinOnce();
  }
  return 0;
}
