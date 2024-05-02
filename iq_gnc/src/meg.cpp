#include <gnc_functions.hpp>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/TwistStamped.h>

double ow;
double ox;
double oy;
double oz;
double siny_cosp;
double cosy_cosp;
double yaw;
double yawd = yaw * 180 / 3.1415;
double lat;
double lon;
double x ;
double y ;
double dist;
double sdist;
double angle;
int flag = 0;
int flag2 = 0;
int flag3 =0;
double so;
double start_lon;
double start_lat;
double hdist;
double size;
int obstacle_angle;
double fdist;
double l_dist;
double lb_dist;
bool object_ahead = false;
bool object_left = false;
bool object_right = false;
bool object_back_left = false;
bool wall_following = false;
double min_left_dist = 100;
double left_dist;
bool parallel = false;
bool reorient = false;
//include API 

void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ow = msg->orientation.w;
    ox = msg->orientation.x;
    oy = msg->orientation.y;
    oz = msg->orientation.z;
    siny_cosp = 2 * (ow * oz + ox * oy);
    cosy_cosp = 1 - 2 * (oy * oy + oz * oz);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    yawd = yaw * 180 / 3.1415;
    if(yawd < 0){
    yawd = 360 + yawd;
   }
   ROS_INFO("Yaw = %f", yawd);

   }
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    geometry_msgs::Point current_pos = get_current_location();
	ROS_INFO("In laser callback");
    if (reorient) {
        left_dist = msg->ranges[360];
        if (left_dist < 30) {
            if (min_left_dist > left_dist) {
                min_left_dist = left_dist;
            }
            else if (min_left_dist < left_dist){
                reorient = false;
                min_left_dist = 100;
            }
        }
        return;
    }
	for(int i = 150; i< 210;i++){
            obstacle_angle = i/2;
            fdist = msg->ranges[i];
            if(fdist<1){
            object_ahead  = true;
            reorient = true;
			ROS_INFO("Obstacle ahead");
            //ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
			geometry_msgs::TwistStamped cmd_vel_msg;
			break;
            }
            else{
            object_ahead  = false;
           }
           }
	for(int i = 330; i< 390 ;i++){          
            obstacle_angle = i/2;
            lb_dist = msg->ranges[i];
            if(lb_dist<30){
            //ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
            if(lb_dist<1){
            object_left = true;
			ROS_INFO("Obstacle left");
            //ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
            //ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);

            break;
            }
            else{
            object_left = false;

           }
       }
    }
    	for(int i = 400; i< 500;i++){

            obstacle_angle = i/2;
            l_dist = msg-> ranges[i];
        
            //ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);
            if(l_dist<1.5){
            object_back_left = true;
            ROS_INFO("Corner");
            //ROS_INFO("angle = %d, distance = %f",obstacle_angle,msg->ranges[i]);

            break;
            }
            else{
            object_back_left= false;

           }   
        }
    }


int main(int argc, char** argv)
{
	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");


	//initialize control publisher/subscribers

	init_publisher_subscriber(gnc_node);
	ros::Subscriber lasersub = gnc_node.subscribe("/spur/laser/scan", 1000, laserCallback);
	ros::Subscriber imusub = gnc_node.subscribe("/mavros/imu/data", 1000, imuCallback);
	ros::Publisher cmd_vel_pub = gnc_node.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 1);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(1.5);

	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	while(ros::ok())
	{	
	geometry_msgs::Point current_pos = get_current_location();
	//ROS_INFO("X = %f, Y = %f, Z= %f ",current_pos.x,current_pos.y,current_pos.z);
	//ROS_INFO("Current Heading = %f",get_current_heading());
	angle = atan2(current_pos.y - 0 ,current_pos.x - 3)* 180 / 3.1415;
	//ROS_INFO("Desired angle = %f",angle);
	geometry_msgs::TwistStamped cmd_vel_msg;
    cmd_vel_msg.header.stamp = ros::Time::now();

	if(!object_ahead && !reorient){
    ROS_INFO("Obstacle not ahead");
	cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = 0.5*cos(yaw);  
    cmd_vel_msg.twist.linear.y = 0.5 * sin(yaw);  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.0; 
    cmd_vel_pub.publish(cmd_vel_msg);
    ros::spinOnce();
    }

     if( object_left == false && object_ahead == false && object_back_left == true && reorient == false){
        ROS_INFO("Corner");
        geometry_msgs::TwistStamped cmd_vel_msg;
	 cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = 0.0;  
    cmd_vel_msg.twist.linear.y = 0.0;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.1;
    cmd_vel_pub.publish(cmd_vel_msg);
    ros::spinOnce();
    }
	
	if(object_ahead || reorient){

		ROS_INFO("Obstacle ahead");
        geometry_msgs::TwistStamped cmd_vel_msg;
	cmd_vel_msg.header.stamp = ros::Time::now();	
    cmd_vel_msg.twist.linear.x = 0.0;  
    cmd_vel_msg.twist.linear.y = 0.0;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = -0.1; 
    cmd_vel_pub.publish(cmd_vel_msg);
	ros::spinOnce();
	}
	if(object_left && !object_ahead && !reorient){
		ROS_INFO("Obstacle Left");
		
	geometry_msgs::TwistStamped cmd_vel_msg;
	 cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = 0.5 * cos(yaw);  
    cmd_vel_msg.twist.linear.y = 0.5 * sin(yaw);  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.0; 
    cmd_vel_pub.publish(cmd_vel_msg);
    ros::spinOnce();}
	}
	rate.sleep();
    ros::spinOnce();
	
	return 0;
}