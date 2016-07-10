#include "FakeLidar.h"

FakeLidar::FakeLidar(int argc, char **argv){
    // Get publish_rate from args
    message_buffer_size = 1000;
    update_frequency = 10;

    // Set up attributes
    angle_min = -2 * M_PI / 3;  // -120 degrees in [rad]
    angle_max = 2 * M_PI / 3;  // 120 degrees in [rad]
    angle_increment = 1 * M_PI / 180;  // 1 degree in [rad]
    time_increment = 1 / 10; // [s]
    range_min = 0.2; // [m]
    range_max = 1.0; // [m]
    num_readings = 1 + ((angle_max - angle_min) / angle_increment);  // 241 readings per scan
    ranges.reserve(num_readings);  // [m]
    intensities.reserve(num_readings);  // device-specific 

    publisher = node.advertise<sensor_msgs::LaserScan>("fake_lidar_node", 1000);
    loop_rate = new ros::Rate(10);  // Doesn't have default constructor
    return;
}

FakeLidar::~FakeLidar(){
    delete loop_rate;
    return;
}

void FakeLidar::update(){
    ros::Time scan_time = ros::Time::now();

    // Generate fake ranges and intensities
    for(unsigned int i = 0; i < num_readings; ++i){
      ranges[i] = 0.5;
      intensities[i] = 10000;
    }

    // Load msg with fake data
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "fake_lidar_frame";
    scan.angle_min = angle_min;
    scan.angle_max = angle_max;
    scan.angle_increment = angle_increment;
    scan.time_increment = (1 / 10) / num_readings;
    scan.range_min = range_min;
    scan.range_max = range_max;

    scan.ranges.resize(num_readings);
    scan.intensities.resize(num_readings);
    for (unsigned int i=0; i < num_readings; i++){
        scan.ranges[i] = ranges[i];
        scan.intensities[i] = intensities[i];
    }   

    // Publish message
    publisher.publish(scan);
    ros::spinOnce();  // unnecessary because no callbacks, but kept as reminder
    loop_rate->sleep();

    return;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "FakeLidar");
    FakeLidar fake_lidar(argc, argv);

    while(ros::ok()){
        fake_lidar.update();
    }
    return 0;
}