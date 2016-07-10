#ifndef FAKELIDAR_H
#define FAKELIDAR_H

#include <math.h>
#include <random>
#include <chrono>
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class FakeLidar {
public:
	int message_buffer_size;
	double update_frequency;
    float angle_min;
    float angle_max;
    float angle_increment;
    float time_increment;
    float range_min;
    float range_max;
    unsigned int num_readings;
    std::vector<float> ranges;
    std::vector<float> intensities; 
	std::string node_name;

	float means[2];
	float stds[2];
	std::default_random_engine* generator;
	std::normal_distribution<float>* range_noise;
	std::normal_distribution<float>* intensity_noise;

	ros::Rate* loop_rate;  // Doesn't have default constructor
	ros::NodeHandle node;
	ros::Publisher publisher;

	FakeLidar();
	~FakeLidar();
	void update();
	void update_ranges();
	void update_intensities();
private:

};

#endif