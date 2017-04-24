#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>

int angle=0;
geometry_msgs::PoseStamped poseStamped;

void msgReceived(const sensor_msgs::JointState &jointState){
  poseStamped.header.frame_id="base_link";
    poseStamped.header.stamp = ros::Time::now();
  
    poseStamped.pose.position.x=2;
    poseStamped.pose.position.y=3;
    poseStamped.pose.position.z=4;
  
    poseStamped.pose.orientation.x=cos(angle);
    poseStamped.pose.orientation.y=1;
    poseStamped.pose.orientation.z=1;
    poseStamped.pose.orientation.w=1;

    angle +=2;
  
    
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "NONKDL_DKIN");
  ros::NodeHandle n;
  ros::Rate loop_rate(30);
  ros::Publisher pub=n.advertise<geometry_msgs::PoseStamped>("nonkdl_marker",1);
  ros::Subscriber sub = n.subscribe("/joint_states", 100, msgReceived);
  
  

  while(ros::ok()){
    ros::spinOnce();
    loop_rate.sleep();
    pub.publish(poseStamped);
  }

  return 0;
}