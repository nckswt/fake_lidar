/*
 * Copyright (c) 2016, University of Colorado, Boulder.
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * 
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 * 
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/** \author Nick Sweet */

#include "fake_lidar.h"

FakeLidar::FakeLidar()
{
  // Set up attributes
  message_buffer_size = 1000;
  update_frequency = 10;                                           // []
  angle_min = 0;                                                   // [rad]
  angle_max = 2 * M_PI;                                            // [rad]
  angle_increment = 2 * M_PI / 360;                                // 1 degree in [rad]
  time_increment = 1 / update_frequency;                           // [s]
  range_min = 0.2;                                                 // [m]
  range_max = 1.5;                                                 // [m]
  num_readings = 1 + ((angle_max - angle_min) / angle_increment);  // readings per scan
  ranges.reserve(num_readings);                                    // [m]
  intensities.reserve(num_readings);                               // device-specific units

  // Set up fake noise
  means[0] = 0.0;
  means[1] = 0.0;
  stds[0] = 0.01;
  stds[1] = 0.01;
  unsigned seed = std::chrono::system_clock::now().time_since_epoch().count();
  generator = new std::default_random_engine(seed);
  range_noise = new std::normal_distribution<float>(means[0], stds[0]);
  intensity_noise = new std::normal_distribution<float>(means[1], stds[1]);

  publisher = node.advertise<sensor_msgs::LaserScan>("fake_lidar_node", message_buffer_size);
  loop_rate = new ros::Rate(update_frequency);
  return;
}

FakeLidar::~FakeLidar()
{
  delete loop_rate;
  delete generator;
  delete range_noise;
  delete intensity_noise;
  return;
}

void FakeLidar::update()
{
  ros::Time scan_time = ros::Time::now();

  // Generate fake ranges and intensities
  updateRanges();
  updateIntensities();

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
  for (unsigned int i = 0; i < num_readings; i++)
  {
    scan.ranges[i] = ranges[i];
    scan.intensities[i] = intensities[i];
  }

  // Publish message
  publisher.publish(scan);
  ros::spinOnce();  // unnecessary because no callbacks, but kept as reminder
  loop_rate->sleep();

  return;
}

void FakeLidar::updateRanges()
{
  for (unsigned int i = 0; i < num_readings; ++i)
    ranges[i] = 1 + (*range_noise)(*generator);
}

void FakeLidar::updateIntensities()
{
  for (unsigned int i = 0; i < num_readings; ++i)
    intensities[i] = 1000 + (*intensity_noise)(*generator);
}

/** This is a simple program that generates fake lidar data with small
* amounts of gaussian noise. It is meant simply to demonstate some of
* the underpinnings of ROS - specifically, the publisher/subscriber model,
* standard message types, and ROS node architecture.
*/
int main(int argc, char **argv)
{
  ros::init(argc, argv, "FakeLidar");
  FakeLidar fake_lidar;

  while (ros::ok())
  {
    fake_lidar.update();
  }
  return 0;
}