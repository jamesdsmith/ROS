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
 * Author: David Fridovich-Keil   ( dfk@eecs.berkeley.edu )
 */

///////////////////////////////////////////////////////////////////////////////
//
// This defines the UAVSlam class.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef UAV_SLAM_H
#define UAV_SLAM_H

#include <ros/ros.h>
#include <message_synchronizer/message_synchronizer.h>
#include <point_cloud_filter/point_cloud_filter.h>
#include <utils/math/transform_3d.h>
#include <uav_odometry/uav_odometry.h>
#include <uav_mapper/uav_mapper.h>
#include <uav_localization/uav_localization.h>

#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_broadcaster.h>
#include <Eigen/Dense>
#include <cmath>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
using namespace math;

class UAVSlam {
 public:
  explicit UAVSlam();
  ~UAVSlam();

  bool Initialize(const ros::NodeHandle& n);

 private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callbacks.
  void AddPointCloudCallback(const PointCloud::ConstPtr& cloud);
  void TimerCallback(const ros::TimerEvent& event);

  // Publish.
  void PublishPose(const Transform3D& transform,
                   const std::string& child_frame_id);
  void PublishFullScan(const PointCloud::ConstPtr& cloud);
  void PublishFilteredScan(const PointCloud::Ptr& cloud);

  // Member variables.
  UAVOdometry odometry_;
  UAVMapper mapper_;
  UAVLocalization localization_;
  PointCloudFilter filter_;

  // Subscribers.
  ros::Subscriber point_cloud_subscriber_;
  ros::Timer timer_;
  MessageSynchronizer<PointCloud::ConstPtr> synchronizer_;

  // Publishers.
  ros::Publisher scan_publisher_full_;
  ros::Publisher scan_publisher_filtered_;
  tf2_ros::TransformBroadcaster transform_broadcaster_;

  // Time stamp.
  ros::Time stamp_;

  std::string scanner_topic_, filtered_topic_, unfiltered_topic_,
    world_frame_, odometry_frame_, localized_frame_;
  bool first_step_;
  bool initialized_;
  std::string name_;
};

#endif
