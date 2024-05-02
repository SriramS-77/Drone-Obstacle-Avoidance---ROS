#include <gnc_functions.hpp>
#include <geometry_msgs/Point.h>
#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/LaserScan.h>
#include "std_msgs/String.h"
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/TwistStamped.h>
#include <stack>
#include <cmath>
#include <math.h>

#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/NavSatStatus.h>

struct coordinate{
	float x; 
	float y; 
	bool visited;
};

coordinate target;
coordinate current_dest;
coordinate next_coordinate;

std::vector<coordinate> tangent_points;
std::vector<coordinate> seen_coordinates;
std::stack<coordinate> stak;

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
double alt;
double x ;
double y ;
double dist, prev_dist;
double angle;
int flag = 0;
int obstacle_angle;
bool object_ahead = false;
bool target_going;
bool tangent_going=false;
bool reached;
bool hover;
bool near = false;


void stop_velocity(ros::Publisher cmd_vel_pub) {
	geometry_msgs::TwistStamped cmd_vel_msg;
	cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = 0.0;  
    cmd_vel_msg.twist.linear.y = 0.0;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.0; 
    cmd_vel_pub.publish(cmd_vel_msg);
	return;
}

void set_velocity(coordinate dest, ros::Publisher cmd_vel_pub, double velocity) {
	geometry_msgs::Point current_pos = get_current_location();
	geometry_msgs::TwistStamped cmd_vel_msg;
	double x_diff = dest.x - current_pos.x, y_diff = dest.y - current_pos.y;
	double mag = sqrt( pow(x_diff, 2) + pow(y_diff, 2) );
	cmd_vel_msg.header.stamp = ros::Time::now();
    cmd_vel_msg.twist.linear.x = x_diff / mag * velocity;  
    cmd_vel_msg.twist.linear.y = y_diff / mag * velocity;  
    cmd_vel_msg.twist.linear.z = 0.0;  
    cmd_vel_msg.twist.angular.x = 0.0; 
    cmd_vel_msg.twist.angular.y = 0.0; 
    cmd_vel_msg.twist.angular.z = 0.0; 
    cmd_vel_pub.publish(cmd_vel_msg);
	return;
}


void imuCallback(const sensor_msgs::Imu::ConstPtr& msg) {
    ow = msg->orientation.w;
    ox = msg->orientation.x;
    oy = msg->orientation.y;
    oz = msg->orientation.z;
    siny_cosp = 2 * (ow * oz + ox * oy);
    cosy_cosp = 1 - 2 * (oy * oy + oz * oz);
    yaw = std::atan2(siny_cosp, cosy_cosp);

    yawd = yaw * 180 / M_PI;
    if(yawd < 0){
    yawd = 360 + yawd;
   }
   // ROS_INFO("Yaw = %f", yawd);
}


void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg){
    geometry_msgs::Point current_pos = get_current_location();
	double heading = get_current_heading();
	ROS_INFO("X = %f, Y = %f, Z= %f, heading = %lf ",current_pos.x,current_pos.y,current_pos.z, heading);
	bool in_object_ahead = false;
	int target_ray, current_ray;
	double target_angle, target_angle_rad, current_angle, current_angle_rad;
	double x_diff = target.x - current_pos.x, y_diff = target.y - current_pos.y;

	// ROS_INFO("");
	double angle_increment = 0.5;

	target_angle_rad = atan(abs(x_diff) / abs(y_diff));
	target_angle = target_angle_rad * (180 / M_PI);


	if (x_diff > 0 && y_diff > 0)
		target_angle = fmod((-target_angle + 90), 360);
	else if (x_diff < 0 && y_diff > 0)
		target_angle = fmod((target_angle + 90), 360);
	else if (x_diff > 0 && y_diff < 0)
		target_angle = fmod((target_angle + 270), 360);
	else if (x_diff < 0 && y_diff < 0)
		target_angle = fmod((-target_angle + 270), 360);

	ROS_INFO("X_Diff ---> %lf, Y_Diff ---> %lf, target_rad ---> %lf", x_diff, y_diff, target_angle_rad);
	
	target_ray = target_angle / angle_increment;
/*
	for (int i=0; i<720; i++){
		dist = msg->ranges[i];
		ROS_INFO("%d laser  =  %f", i, dist);
		if (dist < msg->range_max){
			ROS_INFO("True");
		}
	}
	return;
*/	
	ROS_INFO("Target Ray ---> %d", target_ray);
	for (int i=target_ray-60; i<=target_ray+60; i++){
		current_ray = i % 720;
		if (current_ray < 0)
			current_ray += 720;
		dist = msg->ranges[current_ray];
		if (dist < 10 && dist > 0.1){
			ROS_INFO("Obstacle detected between position and target at %lf metres at %d degrees.", dist, current_ray/2);
			in_object_ahead = true;
			target_going = false;
			break;
		}
	}
	object_ahead = in_object_ahead;
	if (!object_ahead){
		ROS_INFO("No obstacle to target.");
		return;
	}

	tangent_points.clear();
	coordinate possible_coordinate;

	double max_range = msg->range_max, min_range = msg->range_min;

	prev_dist = msg->ranges[719];
	for(int i=0; i<720; i++){
		dist = msg->ranges[i];

		if (prev_dist > max_range && dist < max_range || prev_dist < max_range && dist > max_range){
			current_angle = i * angle_increment;
			possible_coordinate.x = current_pos.x + std::min(dist, prev_dist) * cos(current_angle);
			possible_coordinate.y = current_pos.y + std::min(dist, prev_dist) * sin(current_angle);
			possible_coordinate.visited = false;
			tangent_points.push_back(possible_coordinate);
			// check_and_insert(possible_coordinate);
		}

		prev_dist = dist;
	} 
}

/*
ros::Subscriber global_pos_sub = gnc_node.subscribe("/mavros/global_position/global", 1000, global_pos_callback);
void global_pos_callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
	ROS_INFO("Global position global callback: %f, %f, %f\n", msg->latitude, msg->longitude, msg->altitude);
}
void global_pos_raw_callback(const sensor_msgs::NavSatFix::ConstPtr& msg){
	//ROS_INFO("Global position raw fix callback: %f, %f, %f\n", msg->latitude, msg->longitude, msg->altitude);
}
void alt_callback(const std_msgs::Float64::ConstPtr& msg){
	//ROS_INFO("Global position rel alt: %f\n", msg->data);
}
*/

bool check_if_equal(coordinate a, coordinate b, double pos_tolerance) {
	float deltaX = abs(a.x - b.x);
    float deltaY = abs(a.y - b.y);
	float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) );

	if (dMag < pos_tolerance)
		return true;
	else
		return false;
}

bool check_if_waypoint_reached(coordinate dest_waypoint, double pos_tolerance) {
	geometry_msgs::Point current_pos = get_current_location();
	coordinate current_coordinate;
	current_coordinate.x = current_pos.x;
	current_coordinate.y = current_pos.y;
	return check_if_equal(current_coordinate, dest_waypoint, pos_tolerance);
}

coordinate get_next_coordinate_from_tangents() {
	coordinate popped = tangent_points.back();
	tangent_points.pop_back();
	return popped;
}

bool insert_in_seen_coordinates (coordinate c) {
	for (int i=0; i<seen_coordinates.size(); i++) {
		if (check_if_equal(c, seen_coordinates[i], 0.8)) {
			return false;
		}
	}
	seen_coordinates.push_back(c);
	return true;
}


int main(int argc, char** argv)
{
	target.x = 25;
	target.y = 50;
	target.visited = false;

	//initialize ros 
	ros::init(argc, argv, "gnc_node");
	ros::NodeHandle gnc_node("~");

	//initialize control publisher/subscribers

	init_publisher_subscriber(gnc_node);
	ros::Subscriber lasersub = gnc_node.subscribe("/spur/laser/scan", 1000, laserCallback);
	ros::Subscriber imusub = gnc_node.subscribe("/mavros/imu/data", 1000, imuCallback);
	
	// ros::Subscriber global_pos_raw_sub = gnc_node.subscribe("/mavros/global_position/raw/fix", 1000, global_pos_raw_callback);
	// ros::Subscriber global_alt_sub = gnc_node.subscribe("/mavros/global_position/rel_alt", 1000, alt_callback);

	ros::Publisher cmd_vel_pub = gnc_node.advertise<geometry_msgs::TwistStamped>("/mavros/setpoint_velocity/cmd_vel", 0.5);
	//ros::Publisher cmd_pos_pub = gnc_node.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local", 1);

  	// wait for FCU connection
	wait4connect();

	//wait for used to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(1.5);

	// set_destination(target.x, target.y, 1.5, 0);
	
	set_velocity(target, cmd_vel_pub, 1);
	target_going = true;

	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();

		geometry_msgs::Point current_pos = get_current_location();
		//ROS_INFO("Current Heading = %f",get_current_heading());
		angle = atan2(current_pos.y - 0 ,current_pos.x - 3)* 180 / 3.1415;
		//ROS_INFO("Desired angle = %f",angle);

		if(! object_ahead){
			set_velocity(target, cmd_vel_pub, 2);
			tangent_going = false;
			continue;
		}

		if(check_if_waypoint_reached(target, 3)){
			ROS_INFO("GOAL REACHED!!!");
			land();
			break;
		}

		if (object_ahead && tangent_going) {
			if (check_if_waypoint_reached(next_coordinate, 3)){
				ROS_INFO("\n\nReached intermediate tangent point!\n\n");
				tangent_going = false;
			}
			else{
				set_velocity(next_coordinate, cmd_vel_pub, 2);
			}
		}
		if (! tangent_going && object_ahead){
			next_coordinate = get_next_coordinate_from_tangents();
			stak.push(next_coordinate);
			set_velocity(next_coordinate, cmd_vel_pub, 2);   // set_destination(next_coordinate.x, next_coordinate.y, 1.5, 0);
			ROS_INFO("\n\nMoving towards intermediate, %lf, %lf\n\n", next_coordinate.x, next_coordinate.y);
			tangent_going = true;
		}
	}
	return 0;
}