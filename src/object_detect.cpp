#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <limits>

#define RAD2DEG(x) ((x)*180./M_PI)
#define SURROUNDING_RANGE 0.05 // if the distance between LIDAR and next detected pixel is less than this, treat the pixel and the one before it as connected tgt
#define MIN_CURVE_LENGTH 10 // min. amt of pixels that can be treated as a ball
#define MAX_CURVE_LENGTH 20 // max. amt of pixels that can be treated as a ball

int ball_present = 1;

void obj_detect(const sensor_msgs::LaserScan::ConstPtr& scan) {
    int count = scan->scan_time / scan->time_increment;

    for (int i = 0; i < count; i++) {
        // LIDAR max range is set to 5m (subj to change)
        if (scan->ranges[i] != std::numeric_limits<float>::infinity()) { // if a pixel is detected within the set range
            int length_of_curve = 1;
            int j = i + length_of_curve;
            // while the pixels on the right are in the vicinity 
            while (scan->ranges[j] <= scan->ranges[i] + SURROUNDING_RANGE && scan->ranges[j] >= scan->ranges[i] - SURROUNDING_RANGE) {
                length_of_curve++;
                j++;
                if (j > sizeof(scan->ranges)) {
                    j = 0;
                }
            }
            if (length_of_curve >= MIN_CURVE_LENGTH && length_of_curve <= MAX_CURVE_LENGTH) {
                ROS_INFO("ball at %d, %d,   length: %d", i, j, length_of_curve);
            }
        }
    }    

}

void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
{
    int count = scan->scan_time / scan->time_increment;
    // ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    // ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
  
    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        // ROS_INFO("[i: %d]: [%f, %f]", i, degree, scan->ranges[i]);
    }
    obj_detect(scan);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_client");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan_filtered", 1000, scanCallback);

    ros::spin();

    return 0;
}
