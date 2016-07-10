#ifndef FAKELIDAR_H
#define FAKELIDAR_H

#include <math.h>
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class FakeLidar {
public:
	int message_buffer_size;
	double update_frequency;
    float angle_min;  // [rad]
    float angle_max;  // [rad]
    float angle_increment;  // [rad]
    float time_increment; // [s]
    float range_min; // [m]
    float range_max; // [m]
    unsigned int num_readings;  // readings per scan
    std::vector<float> ranges;  // [m]
    std::vector<float> intensities;  // device-specific 
	std::string node_name;

	ros::Rate* loop_rate;  // [s]
	ros::NodeHandle node;
	ros::Publisher publisher;

	FakeLidar(int argc, char **argv);
	~FakeLidar();
	void update();
private:

};

#endif