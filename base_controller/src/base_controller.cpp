#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include <stdio.h>
#include <math.h>
#include <string.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include "Serial.h"
#include <time.h>

using namespace NS_NaviCommon ;
using namespace std ;

class Base_controller{
  public:
    Base_controller():x_(0),th_(0),odom_x_(0),odom_y_(0),odom_th_(0),odom_ts_(0) {
			serial = new Serial() ;
			
			ros::NodeHandle n ;
			n.param("port_name",port_name_,string("/dev/ttyS0")) ;
			n.param("baud_rate",baud_rate_,115200) ;
			bas_sub_ = n.subscribe("Base_params",10,&Base_controller::basCallback,this) ;
			pid_sub_ = n.subscribe("PID",10,&Base_controller::pidCallback,this) ;
			spd_sub_ = n.subscribe("Twist",10,&Base_controller::spdCallback,this) ;
			pub_ = n.advertise<nav_msgs::Odometry>("odom",10) ;
			timer_ = n.createTimer(ros::Duration(0.1), &Base_controller::spin, this) ;
    }
		~Base_controller(){
			delete serial ;
		}

    bool initialize() {
			serial->bind(port_name_.c_str(),115200) ;
			serial->open() ;

      if(!serial->isOpened()) {
				ROS_INFO("serial port open failure!") ;
        return false;
			}

			char bas_rst[50] = "bas rst(1);\r\n" ;
			int len = serial->senddata((unsigned char*)bas_rst,strlen(bas_rst)) ;
			ROS_INFO("len=%d",len) ;
			ROS_INFO("%s",bas_rst) ;
			getAckLine(getOdomMsg_) ;
			return true;
		}

  private:
    double x_ ;
    double th_ ;
		double odom_x_ ;
		double odom_y_ ;
		double odom_th_ ;
		int odom_ts_ ;
		double flen_per_tick_ ;
		double fdist_wheel_track_ ;
		double fcontrol_rate_ ;
		int iKp_ ;
		int iKi_ ;
		int iKd_ ;
		int iKo_ ;
		char getOdomMsg_[300] ;
		char setBaseBasMsg_[50] ;
		char setBaseVelMsg_[50] ;
		char setBasePidMsg_[50] ;
		Serial *serial ;

		string port_name_ ;
		int baud_rate_ ;
		tf::TransformBroadcaster odom_broadcaster_ ;
		ros::Subscriber bas_sub_ ;
		ros::Subscriber pid_sub_ ;
    ros::Subscriber spd_sub_ ;
    ros::Publisher pub_ ;
		ros::Timer timer_ ;

		void spin(const ros::TimerEvent& e){
			getOdom() ;
			nav_msgs::Odometry odom ;
			odom.header.frame_id = "odom" ;
			odom.header.stamp = ros::Time(odom_ts_/1000.0) ;
			odom.pose.pose.position.x = odom_x_ ;
			odom.pose.pose.position.y = odom_y_ ;
			odom.pose.pose.orientation.x = 0.0 ;
			odom.pose.pose.orientation.y = 0.0 ;
      odom.pose.pose.orientation.z = sin(odom_th_/2.0) ;
      odom.pose.pose.orientation.w = cos(odom_th_/2.0) ;

			pub_.publish(odom) ;

			tf::Transform tmp ;
			tmp.setOrigin(tf::Vector3(odom_x_,odom_y_,0.0)) ;
			tmp.setRotation(tf::Quaternion(0.0,0.0,sin(odom_th_/2.0),cos(odom_th_/2.0))) ;
			odom_broadcaster_.sendTransform(tf::StampedTransform(tmp,ros::Time::now(),"base_link","odom")) ;
		}

		void spdCallback(const geometry_msgs::Twist::ConstPtr& msg){
      x_ = msg->linear.x ;
      th_ = msg->angular.z ;
			sprintf(setBaseVelMsg_,"spd set(%.6lf,%.6lf);\r\n",x_,th_) ;
			// send the msg via serial port
			serial->senddata((unsigned char*)setBaseVelMsg_,strlen(setBaseVelMsg_)) ;

			ROS_INFO("%s",setBaseVelMsg_) ;
			getAckLine(getOdomMsg_) ;
    }

		void pidCallback(const std_msgs::String::ConstPtr& msg){
			const char* msg_data = msg->data.c_str() ;
			sscanf(msg_data,"%d %d %d %d",&iKp_,&iKi_,&iKd_,&iKo_) ;
			sprintf(setBasePidMsg_,"pid set(%d,%d,%d,%d);\r\n",iKp_,iKi_,iKd_,iKo_) ;
			serial->senddata((unsigned char*)setBasePidMsg_,strlen(setBasePidMsg_)) ;
			ROS_INFO("%s",setBasePidMsg_) ;
			getAckLine(getOdomMsg_) ;
		}

		void basCallback(const std_msgs::String::ConstPtr& msg){
			const char* msg_data = msg->data.c_str() ;
			sscanf(msg_data,"%lf %lf %lf",&flen_per_tick_,&fdist_wheel_track_,&fcontrol_rate_) ;
			sprintf(setBaseBasMsg_,"bas set(%.6lf,%.6lf,%.6lf);\r\n",flen_per_tick_,fdist_wheel_track_,fcontrol_rate_) ;
			serial->senddata((unsigned char*)setBaseBasMsg_,strlen(setBaseBasMsg_)) ;
			ROS_INFO("%s",setBaseBasMsg_) ;
			getAckLine(getOdomMsg_) ;
		}

		void getOdom(){
			char mot_get[50] = "mot get(0);\r\n" ;
			//serial->open() ;
			int len = serial->senddata((unsigned char*)mot_get,strlen(mot_get)) ;
			ROS_INFO("len=%d",len) ;
			getAckLine(getOdomMsg_) ;
			sscanf(getOdomMsg_,"ack(%lf,%lf,%lf,%d);",&odom_x_,&odom_y_,&odom_th_,&odom_ts_) ;
		}

		void getAckLine(char* str){
			strcpy(str,"") ;
			int begin = time(0) ;
			while(strncmp(str,"ack",3)){
				int x = 0 ;
				unsigned char c = 0 ;
				while(c!='\n'){
					if(serial->recvdata(&c,sizeof(c))&&c){
						str[x++] = c ;
					}
				}
				str[x] = 0 ;
				int end = time(0) ;
				if(end-begin>=2){
					ROS_INFO("break!") ;
					break ;
				}
			}
			ROS_INFO("%s",str) ;
		}

} ;

int main(int argc, char **argv) {

  ros::init(argc, argv, "base_controller");

	Base_controller base_controller ;
  base_controller.initialize();

	ROS_INFO("before spin") ;
	ros::spin() ;
	ROS_INFO("after spin") ;

  return 0;
}
