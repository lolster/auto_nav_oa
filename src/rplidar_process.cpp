/** Authors
 * Anush S Kumar (anushkumar27@gmail.com)
 * Sushrith Arkal (sushrith.arkal@gmail.com)
 * 
 * Preprocesses the RPLidar raw data and cuts off a radius of max_obstacle_range
 * for distances.
**/

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <algorithm>
#include <cmath>

// "Obstacle map" message
// Returned to client (published for use)
std_msgs::Float32MultiArray ob_map;
float max_range;
float ob_safety_buffer;
void rplidar_sub_cb(const sensor_msgs::LaserScan::ConstPtr& msg) {
	if(!msg->ranges.empty()) {
		std::vector<float> data(msg->ranges.size());
		std::copy(msg->ranges.begin(), msg->ranges.end(), data.begin());
		ob_map.data = std::move(data);
		std::transform(
			ob_map.data.begin(), 
			ob_map.data.end(), 
			ob_map.data.begin(), 
			[](float dist) -> float {
				return (dist > 0 && dist <= max_range) ? std::fdim(dist, ob_safety_buffer)  : std::numeric_limits<float>::infinity();
			}
		);
	}
}

int main(int argc, char **argv) {   
	// Initialise ros node and advertise to roscore
	ros::init(argc, argv, "rplidar_process");

	// NodeHandle is the main access point to communications with the ROS system.
	ros::NodeHandle nh;

	// Get params for max range considered for obstacles
	nh.param<float>("max_obstacle_range", max_range, 5.0f);
	nh.param<float>("obstacle_safety_buffer", ob_safety_buffer, 0.3f);
	// PUBLISHER
	// To publish the pre processed RPLidar value float32[]
	ros::Publisher oa_pub = nh.advertise<std_msgs::Float32MultiArray>
			("oa/data", 10);
	ros::Subscriber rplidar_sub = nh.subscribe<sensor_msgs::LaserScan>
			("/laser/scan", 10, rplidar_sub_cb);

	ros::Rate rate(20.0);

	std::vector<float> initial_data(360);
	std::fill(initial_data.begin(), initial_data.end(), -1);

	ob_map.data = initial_data;

	while(ros::ok()) {
		// Publish the current pose
		oa_pub.publish(ob_map);
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}
