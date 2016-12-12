#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <math>
#include <iostream>
#include <string>
#include <sstream>

using namespace std;

const double pi = 180.0;

class Calibrate2
{
public:
    Calibrate2();
    bool start_test;
private:
    void Callback(const nav_msgs::Odometry odom);
    void Stop();
    double normalize_angle(double angle);
    double getAngle(double oritation);
    geometry_msgs::Twist Twist_();
    bool is_start;
    double test_angle;
    double speed;
    double tolerance;
    double odom_angular_scale_correction;
    double odom_angle;
    string angular_s;
    geometry_msgs::Twist move_cmd;
    ros::Subscriber controller_sub;
    ros::publisher twist_pub;
    ros::NodeHandle nh;
    ros::Rate loop_rate;
}

int int main(int argc, char const *argv[]) {
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

    controller_sub = nh.subscriber("odom",10,&Calibrate2::Calibrate2,this);
    twist_pub = nh.advertise<geometry_msgs::Twist>("Twist",1);
    int reverse(1.0);
    double last_angle(0.0);
    double delta_angle(0.0);
    double turn_angle(0.0);
    double angular_speed(0.0);
    while (ros::ok()) {
        if (start_test) {
            last_angle = odom_angle;
            reverse = -1 * reverse;
            angular_speed = reverse * speed;

            while (abs(turn_angle) < abs(test_angle)) {
                if (ros::is_shutdown()) {
                    return;
                }
                move_cmd = Twist_();
                move_cmd.angular.z = angular_speed;
                twist_pub.publish(move_cmd);
                loop_rate.sleep();

                // Get the current rotation angle from base controller
                // odom_angle = get_odom_angle();
                // I use callback funtion to get this value

                delta_angle = odom_angular_scale_correction * (
                    odom_angle - last_angle);

                turn_angle += delta_angle;
                last_angle = odom_angle;
            }

            // Stop the robot
            for (int i=0; i < 10; i++,) {
                twist_pub.publish(Twist_());
            }
            start_test = false;
        }
	ros::spinOnce();
    }
}

geometry_msgs::Twist Calibrate2::Twist_()
{
    move_cmd.linear.x = 0.0;
    move_cmd.angular.z = 0.0;
}

double::Calibrate2 normalize_angle(double angle)
{
    while (angle > pi) {
        angle -= 2.0 * pi;
    }
    while (angle < -1*pi) {
        angle += 2.0 * pi;
    }
    return angle;
}

double Calibrate2::getAngle(double oritation)
{
    return asin(oritaion);
}

void Calibrate2::Calibrate2(const nav_msgs::Odometry:: odom)
{
    if (!is_start) {
        odom_angle = getAngle(odom.pose.pose.orientation.z);
        is_start = true;
	start_test = true;
    }
    else
    {
        odom_angle = getAngle(odom.pose.pose.orientation.z);
    }
}
