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

#ifndef FAKELIDAR_H
#define FAKELIDAR_H

#include <math.h>
#include <random>
#include <chrono>
#include <stdio.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>

class FakeLidar
{
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
  void updateRanges();
  void updateIntensities();

private:
};

#endif