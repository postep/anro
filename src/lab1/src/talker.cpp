#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <stdio.h>

geometry_msgs::Twist generate_message(char c){
  geometry_msgs::Twist twist;
  switch (c){
    case 'k':
      twist.linear.x = -2;
      break;
    case 'i':
      twist.linear.x = 2;
      break;
    case 'l':
      twist.angular.z = -1;
      break;
    case 'j':
      twist.angular.z = 1;
      break;
    default:
      break;
  }
  return twist;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<geometry_msgs::Twist>("chatter", 1000);

  ros::Rate loop_rate(10);
  system("stty raw");
  while (ros::ok())
  {
    char c = getchar();
    if (c == 'q'){
      ros::shutdown();
      system("stty cooked");
    }else{
      geometry_msgs::Twist twist;
      twist = generate_message(c);
      chatter_pub.publish(twist);
      ros::spinOnce();
      loop_rate.sleep();
    }
  }
  return 0;
}