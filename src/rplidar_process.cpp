/** Authors
 * Anush S Kumar (anushkumar27@gmail.com)
 * Sushrith Arkal (sushrith.arkal@gmail.com)
**/

// Note: This is currently a dummy node publishing simulated RPLidar values for development

#include <ros/ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <vector>

int main(int argc, char **argv) {   
    // Initialise ros node and advertise to roscore
    ros::init(argc, argv, "rplidar_process");

    // NodeHandle is the main access point to communications with the ROS system.
    ros::NodeHandle nh;

    // PUBLISHER
    // To publish the pre processed RPLidar value float32[]
    ros::Publisher oa_pub = nh.advertise<std_msgs::Float32MultiArray>
            ("oa/data", 10);

    // The setpoint publishing rate MUST be faster than 2Hz
    ros::Rate rate(20.0);

    std_msgs::Float32MultiArray ob_map;

    std::vector<float> dummy_data(360);
    std::fill(dummy_data.begin(), dummy_data.end(), -1);

    ob_map.data = dummy_data;

    while(ros::ok()) {
	// Publish the current pose
	oa_pub.publish(ob_map);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
