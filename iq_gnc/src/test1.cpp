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

struct coordinate{
	float x; 
	float y; 
	bool visited;
};

coordinate target;
// coordinate current_dest;
coordinate next_coordinate;

std::vector<coordinate> tangent_points;
std::vector<coordinate> deadend_coordinates;
std::stack<coordinate> stak;

double ow;
double ox;
double oy;
double oz;
double siny_cosp;
double cosy_cosp;
double yaw;
double yawd = yaw * 180 / 3.1415;

double dist, prev_dist;
double angle;
double current_going_angle = -1;
bool object_ahead = false;
bool target_going=true, tangent_going=false;
int tangent_points_number = 0;

bool check_if_equal(coordinate a, coordinate b, double pos_tolerance) {
	float deltaX = abs(a.x - b.x);
    float deltaY = abs(a.y - b.y);
	float dMag = sqrt( pow(deltaX, 2) + pow(deltaY, 2) );

	if (dMag < pos_tolerance)
		return true;
	else
		return false;
}

bool check_if_coordinate_in_vector (coordinate c, std::vector<coordinate>& v, double pos_tolerance) {
	for (int i=0; i<v.size(); i++) {
		if (check_if_equal(c, v[i], pos_tolerance)) {
			return true;
		}
	}
	return false;
}

double get_angle(geometry_msgs::Point cur_pt, coordinate dest_pt) {
    double x_diff = dest_pt.x - cur_pt.x, y_diff = dest_pt.y - cur_pt.y;
	double angle_rad = atan2(y_diff, x_diff);
    ROS_INFO("\n\n\nCalculating for x-diff = %lf and y-diff = %lf.\nAngle in rad: %lf\n\n\n\n", x_diff, y_diff, angle_rad);
	double angle = angle_rad * (180 / M_PI);
	if (angle < 0) {
		angle += 360;
	}
	return angle;
}

void set_velocity(coordinate dest, ros::Publisher cmd_vel_pub, double velocity) {
	geometry_msgs::Point current_pos = get_current_location();
	geometry_msgs::TwistStamped cmd_vel_msg;
	double x_diff = dest.x - current_pos.x, y_diff = dest.y - current_pos.y;
	double mag = sqrt( pow(x_diff, 2) + pow(y_diff, 2) );
	current_going_angle = fmod(atan2(y_diff, x_diff)*180/M_PI + 360, 360);
	ROS_INFO("\n\nSetting velocity as %lf, %lf at angle %lf.\n\n", x_diff / mag, y_diff / mag, current_going_angle);
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

	double angle_increment = 0.5;

	target_angle_rad = atan(abs(x_diff) / abs(y_diff));
	target_angle = target_angle_rad * (180 / M_PI);

	if (x_diff > 0 && y_diff > 0)
		target_angle = fmod((-target_angle), 360);
	else if (x_diff < 0 && y_diff > 0)
		target_angle = fmod(target_angle, 360);
	else if (x_diff > 0 && y_diff < 0)
		target_angle = fmod((target_angle + 180), 360);
	else if (x_diff < 0 && y_diff < 0)
		target_angle = fmod((-target_angle + 180), 360);
	
	target_angle = fmod((target_angle + yawd + 360), 360);

	ROS_INFO("X_Diff ---> %lf, Y_Diff ---> %lf, target_rad ---> %lf", x_diff, y_diff, target_angle_rad);
	
	target_ray = target_angle / angle_increment;
	
	ROS_INFO("Target Ray ---> %d", target_ray);
	for (int i=target_ray-60; i<=target_ray+60; i++){
		current_ray = i % 720;
		if (current_ray < 0)
			current_ray += 720;
		dist = msg->ranges[current_ray];
		if (dist < 10 && dist > 0.1){
			ROS_INFO("Obstacle detected between position and target at %lf metres at %d degrees.", dist, current_ray/2);
			in_object_ahead = true;
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
	double another, current_going_ray;

	current_going_ray = ( fmod((current_going_angle - 270 - yawd + 720), 360) * 2 );

	prev_dist = msg->ranges[719];
	for(int i=0; i<720; i++){
		dist = msg->ranges[i];

		// if (prev_dist > max_range && dist < max_range || prev_dist < max_range && dist > max_range){
		if (prev_dist > max_range && msg->ranges[(i-60+720)%720] > max_range && dist < max_range || prev_dist < max_range && dist > max_range && msg->ranges[(i+60)%720] > max_range){
			current_angle = i * angle_increment;
			// start
			if (prev_dist > max_range)
				current_angle = fmod((current_angle - 20 + 360), 360);
			else
				current_angle = fmod((current_angle + 20 + 360), 360); 
			current_angle = fmod((yawd + 270 + current_angle), 360);
			// end
		
			possible_coordinate.x = current_pos.x + std::min(dist, prev_dist) * cos(current_angle * M_PI / 180);
			possible_coordinate.y = current_pos.y + std::min(dist, prev_dist) * sin(current_angle * M_PI / 180);

			if (! check_if_coordinate_in_vector(possible_coordinate, deadend_coordinates, 5)) {
				tangent_points.emplace_back(possible_coordinate);
			}
			else
				ROS_INFO("Encountered a deadend tangent point");

			// check_and_insert(possible_coordinate);
			another = atan2(possible_coordinate.y - current_pos.y, possible_coordinate.x - current_pos.x) * 180 / M_PI;
			another = fmod((another + 360), 360);
			ROS_INFO("\nTangent point %ld detected at angle %d and %lf\n", tangent_points.size(), (int)current_angle, another);
		}
		prev_dist = dist;

		if (i == int(current_going_ray) && tangent_points.size() == 0) {
			possible_coordinate.x = current_pos.x;
			possible_coordinate.y = current_pos.y;
			if (! check_if_coordinate_in_vector(possible_coordinate, deadend_coordinates, 5))
				deadend_coordinates.emplace_back(possible_coordinate);
		}
	} 
	ROS_INFO("%ld tangent points detected", tangent_points.size());
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

bool check_if_waypoint_reached(coordinate dest_waypoint, double pos_tolerance) {
	geometry_msgs::Point current_pos = get_current_location();
	coordinate current_coordinate;
	current_coordinate.x = current_pos.x;
	current_coordinate.y = current_pos.y;
	return check_if_equal(current_coordinate, dest_waypoint, pos_tolerance);
}

coordinate get_next_coordinate_from_tangents() {
	geometry_msgs::Point current_pos = get_current_location();
	int small = 0;
	double deltaX = abs(current_pos.x - tangent_points[small].x), deltaY = abs(current_pos.y - tangent_points[small].y);
	double cost = sqrt( pow(deltaX, 2) + pow(deltaY, 2) ), new_cost;
	double h_deltaX = abs(target.x - tangent_points[small].x), h_deltaY = abs(target.y - tangent_points[small].y);
	double heuristic = sqrt( pow(h_deltaX, 2) + pow(h_deltaY, 2) ), new_heuristic;

	for (int i=1; i<tangent_points.size(); i++) {
		deltaX = abs(current_pos.x - tangent_points[i].x), deltaY = abs(current_pos.y - tangent_points[i].y);
		h_deltaX = abs(target.x - tangent_points[i].x), h_deltaY = abs(target.y - tangent_points[i].y);
		new_cost = sqrt(pow(deltaX, 2) + pow(deltaY, 2)), new_heuristic = sqrt(pow(h_deltaX, 2) + pow(h_deltaY, 2));
		if (cost + heuristic > new_cost + new_heuristic) {
			small = i;
			cost = new_cost;
			heuristic = new_heuristic;
		}
	}
	coordinate popped = tangent_points[small];
	// tangent_points.pop_back();
	ROS_INFO("\nPopping tangent point %d\n", small+1);
	return popped;
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
	takeoff(3.0);
	
	set_velocity(target, cmd_vel_pub, 2);
	target_going = true;
	tangent_going = false;

	//specify control loop rate. We recommend a low frequency to not over load the FCU with messages. Too many messages will cause the drone to be sluggish
	ros::Rate rate(2.0);
	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();

		geometry_msgs::Point current_pos = get_current_location();
		//ROS_INFO("Current Heading = %f",get_current_heading());
		//angle = atan2(current_pos.y - 0 ,current_pos.x - 3)* 180 / 3.1415;
		//ROS_INFO("Desired angle = %f",angle);

		if(check_if_waypoint_reached(target, 3)){
			ROS_INFO("\n\n\nGOAL REACHED!!!");
			ROS_INFO("\n\nNumber of intermediate tangent points: %d\n", tangent_points_number);
			land();
			break;
		}

		if(! object_ahead){
			ROS_INFO("Going to goal...");
			set_velocity(target, cmd_vel_pub, 2);
			target_going = true;
			tangent_going = false;
			continue;
		}

		if (object_ahead && tangent_going) {
			if (check_if_waypoint_reached(next_coordinate, 3)){
				ROS_INFO("\n\nReached intermediate tangent point!\n\n");
				tangent_going = false;
			}
			else{
				ROS_INFO("Going to intermediate...");
				// next_coordinate = get_next_coordinate_from_tangents(); ////////////////
				set_velocity(next_coordinate, cmd_vel_pub, 2);
			}
		}
		if (! tangent_going && object_ahead){
			next_coordinate = get_next_coordinate_from_tangents();
			stak.push(next_coordinate);
			set_velocity(next_coordinate, cmd_vel_pub, 2);
			ROS_INFO("\n\nMoving towards NEW intermediate, %lf, %lf, at %lf degrees.\n\n", next_coordinate.x, next_coordinate.y, get_angle(current_pos, next_coordinate));
			tangent_going = true;
			tangent_points_number++;
		}
	}
	return 0;
}