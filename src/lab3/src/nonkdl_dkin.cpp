#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <sensor_msgs/JointState.h>
#include <math.h>
#include "Eigen/Dense"

using namespace Eigen;

ros::Publisher pub;

void dhToMatrix(MatrixXd &m, double a, double d, double A, double T){
  m << cos(T), -sin(T), 0, a,
      sin(T)*cos(A), cos(T)*cos(A), -sin(A), -d*sin(A),
      sin(T)*sin(A), cos(T)*sin(A), cos(A), d*cos(A),
      0, 0, 0, 1;
}

void msgReceived(const sensor_msgs::JointState &jointState){
    double t1 = jointState.position[0];
    double t2 = jointState.position[1]-M_PI/2.0;
    double t3 = jointState.position[2];
    
    MatrixXd trans1(4, 4), trans2(4, 4), trans3(4, 4);
    MatrixXd position(4, 1), rotationMatrix(4, 4), basePosition(4, 1);
    basePosition << 0, 0, 0, 1;
    
    dhToMatrix(trans1, 0, 1, 0, t1);
    dhToMatrix(trans2, 0, 0, -M_PI/2.0, t2);
    dhToMatrix(trans3, 1, 0, 0, t3);
    
    rotationMatrix = trans1 * trans2 * trans3;
    
    position = trans1 * trans2 * trans3 * basePosition;
    double w = sqrt(1.0 + rotationMatrix(0, 0) + rotationMatrix(1, 1) + rotationMatrix(2, 2)) / 2.0;
    double w4 = (4.0 * w);
    double x = (rotationMatrix(2, 1) - rotationMatrix(1, 2)) / w4 ;
    double y = (rotationMatrix(0, 2) - rotationMatrix(2, 0)) / w4 ;
    double z = (rotationMatrix(1, 0) - rotationMatrix(0, 1)) / w4 ;
    
    geometry_msgs::PoseStamped poseStamped;
    poseStamped.header.frame_id="base_link";
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.pose.position.x=position(0, 0);
    poseStamped.pose.position.y=position(1, 0);
    poseStamped.pose.position.z=position(2, 0);
    poseStamped.pose.orientation.x=x;
    poseStamped.pose.orientation.y=y;
    poseStamped.pose.orientation.z=z;
    poseStamped.pose.orientation.w=w;
    pub.publish(poseStamped);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "NONKDL_DKIN");
  ros::NodeHandle n;
  ros::Rate loop_rate(30);
  pub=n.advertise<geometry_msgs::PoseStamped>("nonkdl_marker",1);
  ros::Subscriber sub = n.subscribe("/joint_states", 100, msgReceived);
  
  ros::spin();

  return 0;
}