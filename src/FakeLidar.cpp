#include "FakeLidar.h"

FakeLidar::FakeLidar(){
    // Set up attributes
    message_buffer_size = 1000;
    update_frequency = 10; // []
    angle_min = 0;  // [rad]
    angle_max = 2 * M_PI;  // [rad]
    angle_increment = 2 * M_PI / 360;  // 1 degree in [rad]
    time_increment = 1 / update_frequency; // [s]
    range_min = 0.2; // [m]
    range_max = 1.5; // [m]
    num_readings = 1 + ((angle_max - angle_min) / angle_increment);  // readings per scan
    ranges.reserve(num_readings);  // [m]
    intensities.reserve(num_readings);  // device-specific units

    // Set up fake noise
    means[0] = 0.0; means[1] = 0.0;
    stds[0] = 0.01; stds[1] = 0.01;
    unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
    generator = new std::default_random_engine(seed);
    range_noise = new std::normal_distribution<float>(means[0], stds[0]);
    intensity_noise = new std::normal_distribution<float>(means[1], stds[1]);

    publisher = node.advertise<sensor_msgs::LaserScan>("fake_lidar_node", message_buffer_size);
    loop_rate = new ros::Rate(update_frequency);
    return;
}

FakeLidar::~FakeLidar(){
    delete loop_rate;
    delete generator;
    delete range_noise;
    delete intensity_noise;
    return;
}

void FakeLidar::update(){
    ros::Time scan_time = ros::Time::now();

    // Generate fake ranges and intensities
    update_ranges();
    update_intensities();

    // Load msg with fake data
    sensor_msgs::LaserScan scan;
    scan.header.stamp = scan_time;
    scan.header.frame_id = "fake_lidar_frame";
    scan.angle_min = angle_min;
    scan.angle_max = angle_max;
    scan.angle_increment = angle_increment;
    scan.time_increment = (1 / update_frequency) / num_readings;
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

void FakeLidar::update_ranges(){
    for(unsigned int i = 0; i < num_readings; ++i){
        ranges[i] = 1 + (*range_noise)(*generator);
    }
}

void FakeLidar::update_intensities(){
    for(unsigned int i = 0; i < num_readings; ++i){
        intensities[i] = 1000 + (*intensity_noise)(*generator);
    }
}


int main(int argc, char **argv){
    ros::init(argc, argv, "FakeLidar");
    FakeLidar fake_lidar;

    while(ros::ok()){
        fake_lidar.update();
    }
    return 0;
}