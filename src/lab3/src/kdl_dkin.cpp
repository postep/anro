#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/PoseStamped.h"
#include "sensor_msgs/JointState.h"
#include <kdl/chain.hpp>
#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/frames.hpp>

using namespace std;
using namespace KDL;

void callback(const sensor_msgs::JointState & msg);
void CalculateQuaternion();

double t1,t2,t3;
double a2=0,a3=0,d1=0; //default 0.4 0.2 0.2
double Quat[4];
double M[3][3];
double pos[3];

void calculateQuaternion(double T[][3]);

int main(int argc, char **argv)
{	
    //inicjacja ros
	ros::init(argc, argv, "kdl");
	
	ros::NodeHandle n;
	
    //utworzenie pub i sub
	ros::Publisher kdl_pub=n.advertise<geometry_msgs::PoseStamped>("posestamped_kdl",1);
	ros::Subscriber kdl_sub=n.subscribe("joint_states",1000,callback);	
    
    //pobranie parametrów
    n.param<double>("a2",a2,0.4);
    n.param<double>("a3",a3,0.2);
    n.param<double>("d1",d1,0.2);
    
	ros::Rate loop_rate(100);
	geometry_msgs::PoseStamped msg;
	
	//stworzenie łancucha konematycznego
	KDL::Chain chain;
	t1=0;	t2=M_PI/4.0; t3=M_PI/4.0;	
	
	chain.addSegment(Segment(Joint(Joint::None),Frame(Frame::DH(0,0,0,0)))); //a, alpha, d, theta
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0,0,1.0,0))));
	chain.addSegment(Segment(Joint(Joint::None),Frame(Frame::DH(0,-M_PI/2.0,0,0))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(1,0,0,-M_PI/2.0))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0,0,0,0))));	
	
	
	//utworzenie solveraros 
	ChainFkSolverPos_recursive solver(chain);
	JntArray q(chain.getNrOfJoints());
	Frame F_results;
	
		
	while(ros::ok())
	{
	    ROS_DEBUG_STREAM("i "<<chain.getNrOfJoints());
	    
	    q(0)=t1;
	    q(1)=t2;
      	    q(2)=t3;
       	
      	solver.JntToCart(q,F_results);
       	int k=0;
       	for(int j=0;j<3;j++) {
			pos[j]=F_results.p.data[j];
			
				for(int i=0;i<3;i++) {
		        	M[j][i]=F_results.M.data[k];
			        k++;
		        }   
		}
       	
       	calculateQuaternion(M);
		//wypełenienie wiadomości treścią
		msg.header.frame_id="base_link";
		msg.pose.orientation.x=Quat[0];
		msg.pose.orientation.y=Quat[1];
		msg.pose.orientation.z=Quat[2];
		msg.pose.orientation.w=Quat[3];
		msg.pose.position.x=pos[0];
		msg.pose.position.y=pos[1];
		msg.pose.position.z=pos[2];
	    
	    //publish
		kdl_pub.publish(msg);
	
		ros::spinOnce();
	
		loop_rate.sleep();
	}

	return 0;
}

void callback(const sensor_msgs::JointState & msg)
{
	t1=msg.position[0];
	t2=msg.position[1];
	t3=msg.position[2];
			
} 

void calculateQuaternion(double T[][3])
{
  float trace = T[0][0] + T[1][1] + T[2][2]; 
  if( trace > 0 ) {
    float s = 0.5f / sqrtf(trace+ 1.0f);
    Quat[3] = 0.25f / s;
    Quat[0] = ( T[2][1] - T[1][2] ) * s;
    Quat[1] = ( T[0][2] - T[2][0] ) * s;
    Quat[2] = ( T[1][0] - T[0][1] ) * s;
  } else {
    if ( T[0][0] > T[1][1] && T[0][0] > T[2][2] ) {
      float s = 2.0f * sqrtf( 1.0f + T[0][0] - T[1][1] - T[2][2]);
      Quat[3] = (T[2][1] - T[1][2] ) / s;
      Quat[0] = 0.25f * s;
      Quat[1] = (T[0][1] + T[1][0] ) / s;
      Quat[2] = (T[0][2] + T[2][0] ) / s;
    } else if (T[1][1] > T[2][2]) {
      float s = 2.0f * sqrtf( 1.0f + T[1][1] - T[0][0] - T[2][2]);
      Quat[3] = (T[0][2] - T[2][0] ) / s;
      Quat[0] = (T[0][1] + T[1][0] ) / s;
      Quat[1] = 0.25f * s;
      Quat[2] = (T[1][2] + T[2][1] ) / s;
    } else {
      float s = 2.0f * sqrtf( 1.0f + T[2][2] - T[0][0] - T[1][1] );
      Quat[3] = (T[1][0] - T[0][1] ) / s;
      Quat[0] = (T[0][2] + T[2][0] ) / s;
      Quat[1] = (T[1][2] + T[2][1] ) / s;
      Quat[2] = 0.25f * s;
    }
  }
  
}
