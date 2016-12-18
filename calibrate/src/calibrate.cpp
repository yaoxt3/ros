#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <math.h>
#include <iostream>
#include <string>
#include <sstream>
using namespace  std;

struct Point
{
	double position_x;
	double position_y;
	Point():position_x(0.0),position_y(0.0){}
};


class Calibrate
{
public:
	Calibrate();
	void Stop();
	bool start_test;
private:
	void Callback(const nav_msgs::Odometry odom);
	double test_distance;
	double speed;
	double tolerance;
	double odom_linear_scale_correction;
	string base_params;
	string pid;
	double start_x;
	double start_y;
	bool is_shutdown;
	bool is_start;
	Point point;
	ros::Subscriber controller_sub;
	ros::Publisher twist_pub;
	ros::Publisher string_pub;
	ros::Publisher base_params_pub;
	ros::Publisher base_rst_pub;
	ros::NodeHandle nh;
	ros::Rate loop_rate;
	geometry_msgs::Twist move_cmd;
	std_msgs::String string_msg;
	std_msgs::String base_string_msg;
	std_msgs::String base_rst_msg;
};


int main(int argc, char **argv)
{
	cout<<"start"<<endl;
	ros::init(argc,argv,"calibrate");
	Calibrate myCalibrate;
	cout<<"end"<<endl;
	return 0;
}


Calibrate::Calibrate():
					   start_x(0.0),
					   start_y(0.0),
					   is_shutdown(false),
					   start_test(false),
					   is_start(false),
					   test_distance(1.0),
					   speed(0.15),
					   tolerance(0.01),
					   odom_linear_scale_correction(1.0),
					   loop_rate(30)
{
	ros::param::get("test_distance",test_distance);
	ros::param::get("speed",speed);
	ros::param::get("tolerance",tolerance);
	ros::param::get("odom_linear_scale_correction",odom_linear_scale_correction);
	ros::param::get("pid",pid);
	ros::param::get("base_params",base_params);
	cout<<"test_distance:"<<test_distance<<endl;
	cout<<"speed:"<<speed<<endl;
	cout<<"tolerance:"<<tolerance<<endl;
	cout<<"odom_linear_scale_correction:"<<odom_linear_scale_correction<<endl;
	cout<<"pid:"<<pid<<endl;
	cout<<"base_params:"<<base_params<<endl;

	// Reset the odometry data
        base_rst_pub = nh.advertise<std_msgs::String>("Reset",10);
        base_rst_msg.data = "1";
	cout<<base_rst_msg.data<<endl;

	int ii=100;
        while(ii-->0) base_rst_pub.publish(base_rst_msg);
	cout<<"reset end"<<endl;

	controller_sub = nh.subscribe("odom",10,&Calibrate::Callback,this);
	twist_pub = nh.advertise<geometry_msgs::Twist>("Twist",10);
	string_pub = nh.advertise<std_msgs::String>("PID",10);
	base_params_pub = nh.advertise<std_msgs::String>("Base_params",10);

	string_msg.data = pid;
	base_string_msg.data = base_params;

	// Publish the string
	string_pub.publish(string_msg);
	base_params_pub.publish(base_string_msg);

	double distances = 0.0;
	// this loop will be break, if we call shutdown()
	while(ros::ok())
	{
		//test start
		if (start_test)
		{
			double temp_x = point.position_x-start_x;
			double temp_y = point.position_y-start_y;
			distances = sqrt(temp_x*temp_x + temp_y*temp_y);
			//cout<<"distance : "<<distances<<endl;
			distances *= odom_linear_scale_correction;
			double error = distances - test_distance;
			cout<<"error distances: "<<error<<endl;
			if (abs(error) < tolerance)
			{
				//calibrate completely ,break loop;
				int k=10;
				while(--k>0) Stop();
				cout<<"final distance: "<<distances<<endl;
				break;
			}
			else
			{
				move_cmd.linear.x = copysign(speed,-1*error);
			}
		}
		twist_pub.publish(move_cmd);
		loop_rate.sleep();
		ros::spinOnce();
	}
}

void Calibrate::Callback(const nav_msgs::Odometry odom)
{
	//cout<<"callback"<<endl;
	//set start position
	if (!is_start)
	{
		//Is start
		cout<<"Is start!"<<endl;
		cout<<"start position is:(";
		start_x = odom.pose.pose.position.x;
		start_y = odom.pose.pose.position.y;
		cout<<start_x<<","<<start_y<<")"<<endl;
		is_start = true;
		start_test = true;
	}
	else
	{
		point.position_x = odom.pose.pose.position.x;
		point.position_y = odom.pose.pose.position.y;
		cout<<"x: "<<point.position_x<<endl;
		cout<<"y: "<<point.position_y<<endl;
	}
}

void Calibrate::Stop()
{
	move_cmd.linear.x = 0.0;
	move_cmd.angular.z = 0.0;
	twist_pub.publish(move_cmd);
}
