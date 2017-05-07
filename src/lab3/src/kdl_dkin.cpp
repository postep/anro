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

double t1,t2,t3;
double a2=0,a3=0,d1=0;
double Quaternion[4];
double Matrix[3][3];
double pos[3];

void calcQuat(double T[][3]);

int main(int argc, char **argv)
{
	ros::init(argc, argv, "KDL_DKIN");	
	ros::NodeHandle n;
	
	ros::Publisher kdl_pub=n.advertise<geometry_msgs::PoseStamped>("posestamped_kdl",1);
	ros::Subscriber kdl_sub=n.subscribe("/joint_states",100,callback);

	n.param<double>("a2",a2,0.4);
	n.param<double>("a3",a3,0.2);
	n.param<double>("d1",d1,0.2);
    
	ros::Rate loop_rate(30);
	geometry_msgs::PoseStamped msg;
	
//łańcuch KDL
	KDL::Chain chain;
	t1=0;
	t2=M_PI/4.0;
	t3=M_PI/4.0;	
	
	chain.addSegment(Segment(Joint(Joint::None),Frame(Frame::DH(0,0,0,0))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0,0,1.0,0))));
	chain.addSegment(Segment(Joint(Joint::None),Frame(Frame::DH(0,-M_PI/2.0,0,0))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(1,0,0,-M_PI/2.0))));
	chain.addSegment(Segment(Joint(Joint::RotZ),Frame(Frame::DH(0,0,0,0))));	
	
	
//solver
	ChainFkSolverPos_recursive solver(chain);
	JntArray q(chain.getNrOfJoints());
	Frame F_results;
	
		
	while(ros::ok())
	{
		q(0)=t1;
		q(1)=t2;
		q(2)=t3;
       	
	      	solver.JntToCart(q,F_results);
	       	int k=0;
		for(int j=0;j<3;j++) {
			pos[j]=F_results.p.data[j];
			
				for(int i=0;i<3;i++) {
		        	Matrix[j][i]=F_results.M.data[k];
			        k++;
		        }   
		}
       	
       		calcQuat(Matrix);
//wiadomość do publishera
		msg.header.frame_id="base_link";

		msg.pose.position.x=pos[0];
		msg.pose.position.y=pos[1];
		msg.pose.position.z=pos[2];

		msg.pose.orientation.x=Quaternion[0];
		msg.pose.orientation.y=Quaternion[1];
		msg.pose.orientation.z=Quaternion[2];
		msg.pose.orientation.w=Quaternion[3];

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

void calcQuat(double T[][3])
{
	float trace = T[0][0] + T[1][1] + T[2][2]; 
	if( trace > 0 ) {
		float s = 0.5f / sqrtf(trace+ 1.0f);
		Quaternion[3] = 0.25f / s;
		Quaternion[0] = ( T[2][1] - T[1][2] ) * s;
		Quaternion[1] = ( T[0][2] - T[2][0] ) * s;
		Quaternion[2] = ( T[1][0] - T[0][1] ) * s;
	} else {
		if ( T[0][0] > T[1][1] && T[0][0] > T[2][2] ) {
			float s = 2.0f * sqrtf( 1.0f + T[0][0] - T[1][1] - T[2][2]);
			Quaternion[3] = (T[2][1] - T[1][2] ) / s;
			Quaternion[0] = 0.25f * s;
			Quaternion[1] = (T[0][1] + T[1][0] ) / s;
			Quaternion[2] = (T[0][2] + T[2][0] ) / s;
		} else if (T[1][1] > T[2][2]) {
			float s = 2.0f * sqrtf( 1.0f + T[1][1] - T[0][0] - T[2][2]);
			Quaternion[3] = (T[0][2] - T[2][0] ) / s;
			Quaternion[0] = (T[0][1] + T[1][0] ) / s;
			Quaternion[1] = 0.25f * s;
			Quaternion[2] = (T[1][2] + T[2][1] ) / s;
		} else {
			float s = 2.0f * sqrtf( 1.0f + T[2][2] - T[0][0] - T[1][1] );
			Quaternion[3] = (T[1][0] - T[0][1] ) / s;
			Quaternion[0] = (T[0][2] + T[2][0] ) / s;
			Quaternion[1] = (T[1][2] + T[2][1] ) / s;
			Quaternion[2] = 0.25f * s;
		}
	}
}
