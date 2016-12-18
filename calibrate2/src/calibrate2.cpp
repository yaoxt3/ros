#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <cmath>
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

const double pi = 3.1415926;

class Calibrate2
{
public:
    Calibrate2();
    bool start_test;
private:
    void Callback(const nav_msgs::Odometry odom);
    void Stop();
    double normalize_angle(double angle);
    double radians(double angle);
    double getAngle(double oritation);
    geometry_msgs::Twist Twist_();
    bool is_start;
    double test_angle;
    double speed;
    double tolerance;
    double odom_angular_scale_correction;
    double odom_angle;
	double last_angle;
    string angular_s;
    geometry_msgs::Twist move_cmd;
	std_msgs::String base_rst_msg;
    ros::Subscriber controller_sub;
    ros::Publisher twist_pub;
	ros::Publisher base_rst_pub;
    ros::NodeHandle nh;
    ros::Rate loop_rate;
};

int main(int argc, char **argv) {
    cout<<"start"<<endl;
    ros::init(argc,argv,"calibrate2");
    Calibrate2 myCalibrate2;
    cout<<"end"<<endl;
    return 0;
}

Calibrate2::Calibrate2():
                        odom_angle(0.0),
                        start_test(false),
                        is_start(false),
                        test_angle(180.0),
                        speed(0.7),
                        tolerance(0.05),
                        odom_angular_scale_correction(1.0),
                        loop_rate(10)
{
    ros::param::get("angular_s",angular_s);
    cout<<"angular_s: "<<angular_s<<endl;

    stringstream ss;
    ss << angular_s;
    ss >> test_angle;
    ss >> speed;
    ss >> tolerance;
    ss >> odom_angular_scale_correction;

    cout<<"test_angle: "<<test_angle<<endl;
    cout<<"speed: "<<speed<<endl;
    cout<<"tolerance: "<<tolerance<<endl;
    cout<<"odom_angular_scale_correction: "<<odom_angular_scale_correction<<endl;

    test_angle = radians(test_angle);
    cout<<"test_radians: "<<test_angle<<endl;
    tolerance = radians(tolerance);
    cout<<"tolerance_radians: "<<tolerance<<endl;
	
	// Reset the odometry data
    base_rst_pub = nh.advertise<std_msgs::String>("Reset",10);
    base_rst_msg.data = "1";
	cout<<base_rst_msg.data<<endl;
	
	cout<<"reset end"<<endl;

    controller_sub = nh.subscribe("odom",10,&Calibrate2::Callback,this);
    twist_pub = nh.advertise<geometry_msgs::Twist>("Twist",1);
    double delta_angle(0.0);
    double turn_angle(0.0);
	move_cmd = Twist_(); 
	move_cmd.angular.z = (double)speed;   
	while (ros::ok()) {
        if (start_test) {
			delta_angle = odom_angular_scale_correction * normalize_angle(odom_angle - last_angle);
			turn_angle += delta_angle;
			double error = abs(test_angle)-abs(turn_angle);
			cout<<"error: "<<error<<endl;
			if(error>tolerance){
				last_angle = odom_angle;
			}
			else{
				for(int i=0;i<500;i++) twist_pub.publish(Twist_());
				cout<<"final angle:"<<odom_angle<<endl;
            	start_test = false;
				break;
			}
        }
		twist_pub.publish(move_cmd);
        loop_rate.sleep();
		ros::spinOnce();
    }
    
}

geometry_msgs::Twist Calibrate2::Twist_()
{
    geometry_msgs::Twist new_cmd;
    new_cmd.linear.x = 0.0;
    new_cmd.linear.y = 0.0;
    new_cmd.linear.z = 0.0;
    new_cmd.angular.x = 0.0;
    new_cmd.angular.y = 0.0;
    new_cmd.angular.z = 0.0;
    return new_cmd;
}

double Calibrate2::normalize_angle(double angle)
{
    while (angle > pi) {
        angle -= 2.0 * pi;
    }
    while (angle < -1*pi) {
        angle += 2.0 * pi;
    }
    return angle;
}

double Calibrate2::radians(double angle)
{
    return (angle*pi)/180.0;
}


double Calibrate2::getAngle(double oritation)
{
    return asin(oritation)*2.0;
}

void Calibrate2::Callback(const nav_msgs::Odometry odom)
{
    if (!is_start) {
		base_rst_pub.publish(base_rst_msg);
		loop_rate.sleep();
        last_angle = getAngle(odom.pose.pose.orientation.z);
        is_start = true;
		start_test = true;
		cout<<"callback last_angle: "<<last_angle<<endl;
    }
    else
    {
        odom_angle = getAngle(odom.pose.pose.orientation.z);
		cout<<"odom_angle: "<<odom_angle<<endl;
    }
}
