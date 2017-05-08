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
		
		//wiadomość do publishera
		msg.header.frame_id="base_link";

		msg.pose.position.x=F_results.p.data[0];
		msg.pose.position.y=F_results.p.data[1];
		msg.pose.position.z=F_results.p.data[2];

		double x, y, z, w;
		F_results.M.GetQuaternion(x, y, z, w);
		msg.pose.orientation.x=x;
		msg.pose.orientation.y=y;
		msg.pose.orientation.z=z;
		msg.pose.orientation.w=w;

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