/*
 * Copyright (c) 2015, The Regents of the University of California (Regents).
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are
 * met:
 *
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *
 *    2. Redistributions in binary form must reproduce the above
 *       copyright notice, this list of conditions and the following
 *       disclaimer in the documentation and/or other materials provided
 *       with the distribution.
 *
 *    3. Neither the name of the copyright holder nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS AS IS
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Please contact the author(s) of this library if you have any questions.
 * Author: James Smith   ( james.smith@berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// This defines the DepthCloudProjector class.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef GUIDANCE_DRIVER_H
#define GUIDANCE_DRIVER_H

#include <memory>
#include <ros/ros.h>
#include <dji/DJI_guidance.h>
#include <dji/DJI_utility.h>
#include <opencv/cv.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/LaserScan.h>

/**
 * Allows us to configure camera directions to specific indices
 * This essentially defines the configuration of the physical cameras and allows
 * us to write code that is index agnostic (ie, if we swap two camera indices, we can
 * just change the indices here)
 */
enum camera_direction
{
  cam_front = e_vbus1,
  cam_right = e_vbus2,
  cam_back = e_vbus3,
  cam_left = e_vbus4,
  cam_down = e_vbus5
};

class GuidanceDriver {
 public:
  explicit GuidanceDriver();
  ~GuidanceDriver();

  bool Initialize(const ros::NodeHandle& n);

 private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // SDK Wrappers
  bool InitGuidanceSDK();
  bool InitGuidanceData();

  // Data selectors
  bool SelectCameraPair(camera_direction dir);
  bool SelectIMU();
  bool SelectUltrasonic();
  bool SelectVelocity();
  bool SelectObstacleDistance();

  // SDK data polling
  bool UpdateOnlineStatus();
  bool UpdateStereoCalibration();

  void PrintOnlineStatus();
  void PrintStereoCalibration();

  // SDK Callback
public:
  int OnSDKEvent(int data_type, int data_len, char* content);
private:
  static int OnSDKEventCallback(int data_type, int data_len, char* content);
  bool ProcessImageData(char* content);
  bool ProcessIMUData(char* content);
  bool ProcessUltrasonicData(char* content);
  bool ProcessVelocityData(char* content);
  bool ProcessObstacleDistanceData(char* content);

  bool ProcessGreyscaleLeftImage(cv::Mat img);
  bool ProcessGreyscaleRightImage(cv::Mat img);
  bool ProcessStereoPair(cv::Mat left, cv::Mat right, int index);
  bool ProcessDepthImage(cv::Mat img);
  bool ProcessDisparityImage(cv::Mat img);

  // Callbacks.
  //void DepthMapCallback(const sensor_msgs::Image& map);

  // Publishers/subscribers.
  //ros::Publisher cloud_pub_;
  //ros::Subscriber depth_sub_;
  ros::Publisher depth_image_pub_;
  ros::Publisher disparity_image_pub_;
  ros::Publisher left_image_pub_;
  ros::Publisher right_image_pub_;
  ros::Publisher imu_pub_;
  ros::Publisher obstacle_distance_pub_;
  ros::Publisher velocity_pub_;
  ros::Publisher ultrasonic_pub_;
  ros::Publisher stereo_pair_pub_;

  // Time stamp.
  ros::Time stamp_;

  bool initialized_;
  std::string name_;

  // Store data from the sdk
  stereo_cali stereo_calibration_[CAMERA_PAIR_NUM];
  int online_status_[CAMERA_PAIR_NUM];
};

#endif
