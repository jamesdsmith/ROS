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
// This defines the uav_mapper node.
//
///////////////////////////////////////////////////////////////////////////////

#ifndef UAV_MAPPER_H
#define UAV_MAPPER_H

#include <ros/ros.h>
#include <message_synchronizer/message_synchronizer.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf2_ros/transform_broadcaster.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class UAVMapper {
 public:
  explicit UAVMapper();
  ~UAVMapper();

  bool Initialize(const ros::NodeHandle& n);

 private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Helpers.
  Eigen::Matrix4f PointCloudOdometry(const PointCloud::ConstPtr& cloud);

  // Callbacks.
  void AddPointCloudCallback(const PointCloud::ConstPtr& cloud);

  // Communication.
  MessageSynchronizer<PointCloud> synchronizer_;
  ros::Subscriber point_cloud_subscriber_;
  ros::Publisher point_cloud_publisher_;
  ros::Publisher point_cloud_publisher_filtered_;
  ros::Publisher point_cloud_publisher_aligned_;

  tf2_ros::TransformBroadcaster transform_broadcaster_;

  // Integrated transform.
  Eigen::Quaterniond integrated_rotation_;
  Eigen::Vector3d integrated_translation_;

  // Last point cloud.
  PointCloud::Ptr previous_cloud_;

  // Name.
  std::string name_;
};

#endif
