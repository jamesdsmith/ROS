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
// Definitions for the DepthCloudProjector class.
//
///////////////////////////////////////////////////////////////////////////////

#include <depth_cloud_projector/depth_cloud_projector.h>
#include <mapper/mapper.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <opencv/cv.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/transforms.h>
#include <utils/math/transform_3d.h>
#include <utils/math/rotation.h>
#include <math.h>
#include <chrono>

// Constructor/destructor.
DepthCloudProjector::DepthCloudProjector() : initialized_(false) {}
DepthCloudProjector::~DepthCloudProjector() {}

// Initialize.
bool DepthCloudProjector::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "depth_cloud_projector");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  initialized_ = true;
  return true;
}

// Load parameters.
bool DepthCloudProjector::LoadParameters(const ros::NodeHandle& n) {
  return true;
}

// Register callbacks.
bool DepthCloudProjector::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle node(n);

  // Subscriber.
  depth_sub_ =
    node.subscribe("/guidance/depth_image", 10,
                   &DepthCloudProjector::DepthMapCallback, this);
  multi_img_sub_ =
    node.subscribe("/guidance/depth_images", 1,
                   &DepthCloudProjector::MultiImageCallback, this);

  // Publishers.
  cloud_pub_ = node.advertise<PointCloud>("/mapper/cloud", 10, false);

  return true;
}

// Point cloud callback.
void DepthCloudProjector::DepthMapCallback(const sensor_msgs::Image& msg) {
  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  DepthMap dm(cv_ptr->image);
  dm.SetInverted(false);
  Mapper m(true);

  PointCloud cl = m.ProjectDepthMap(dm);

  //std::cout << "Projecting " << cl.size() << " points" << std::endl;

  cl.header.frame_id = "guidance";
  cl.header.stamp = cv_ptr->header.stamp.toNSec() / 1000;
  cl.header.seq = cv_ptr->header.seq;

  cloud_pub_.publish(cl.makeShared());
}

using namespace std::chrono;
milliseconds start = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
int frames = 0;

void DepthCloudProjector::MultiImageCallback(const dji_guidance::multi_image::ConstPtr& msg) {
  PointCloud cl;
  for (auto const& img : msg->images) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
      sensor_msgs::Image im = img.image;
      cv_ptr = cv_bridge::toCvCopy(im, sensor_msgs::image_encodings::MONO16);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // cv::Mat depth8(320, 240, CV_8UC1);
    // cv_ptr->image.convertTo(depth8, CV_8UC1);
    
    DepthMap dm(cv_ptr->image);
    dm.SetInverted(false);
    Mapper m(true);

    PointCloud projection = m.ProjectDepthMap(dm);

    math::Transform3D tform = GetRotationFor(img.vbus_index) * GetOffsetFor(img.vbus_index);
    PointCloud transformed;
    pcl::transformPointCloud(projection, transformed, tform.GetTransform());

    // Filter the point cloud...
    // pcl::PointCloud<pcl::PointXYZ>::Ptr transformedPtr(&transformed);
    // PointCloud filtered;
    // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    // sor.setInputCloud(transformedPtr);
    // sor.setMeanK(50);
    // sor.setStddevMulThresh(1.0);
    // sor.filter(filtered);
    // cl = cl + filtered;

    cl = cl + transformed;
    cl.header.frame_id = "guidance";
    cl.header.stamp = cv_ptr->header.stamp.toNSec() / 1000;
    cl.header.seq = cv_ptr->header.seq;
  }
  //std::cout << "Projected " << cl.size() << " points" << std::endl;
  cloud_pub_.publish(cl.makeShared());

  frames++;
  milliseconds now = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
  if ((now - start).count() > 1000) {
      std::cout << "SDK pull rate: " << ((float)frames / (now - start).count()) * 1000.f << std::endl;
      start = now;
      frames = 0;
  }
}

/*
  1 front on M100
  2 right on M100
  3 rear  on M100
  4 left  on M100
  0 down  on M100
  */

math::Transform3D DepthCloudProjector::GetOffsetFor(int index) {
  Eigen::Matrix3d zero = math::EulerAnglesToMatrix(0,0,0);

  // Translation spec:
  // Units: meters?
  // Eigen::Vector3d(x, y, z)
  //    +x = backward
  //    +y = down
  //    +z = right

  // Matrice 100 camera position description:
  // Each of the side cameras are all 10 cm from the middle
  // Each side camera is 3.5 cm offset to the left (looking in the camera direction)
  // bottom is about 3.5 cm down
  switch (index) {
    case 2: return math::Transform3D(zero, Eigen::Vector3d(-0.035, 0, 0.1));
    case 3: return math::Transform3D(zero, Eigen::Vector3d(0.035, 0, -0.1));
    case 4: return math::Transform3D(zero, Eigen::Vector3d(0.1, 0, 0.035));
    case 0: return math::Transform3D(zero, Eigen::Vector3d(0.02, 0.035, -0.035));
    case 1:
    default:
      return math::Transform3D(zero, Eigen::Vector3d(0, 0, 0));
  }
}

math::Transform3D DepthCloudProjector::GetRotationFor(int index) {
  // PITCH, YAW, ROLL
  // Rotation examples:
  //   90 degree pitch forward
  //     math::EulerAnglesToMatrix(math::D2R(-90), 0, 0);
  //   90 degree yaw to the right
  //     math::EulerAnglesToMatrix(0, math::D2R(90), 0);
  //   90 degree roll right (CW)
  //     math::EulerAnglesToMatrix(0, 0, math::D2R(90));

  math::Transform3D tform;
  // pitch forward
  Eigen::Matrix3d temp = math::EulerAnglesToMatrix(math::D2R(-90), 0, 0);
  math::Transform3D pitch;
  pitch.SetRotation(temp);

  // yaw right
  temp = math::EulerAnglesToMatrix(0, math::D2R(90), 0);
  math::Transform3D yaw;
  yaw.SetRotation(temp);

  // roll right
  temp = math::EulerAnglesToMatrix(0, 0, math::D2R(90));
  math::Transform3D roll;
  roll.SetRotation(temp);

  
  switch (index) {
    // This was an experimental way to find the right rotations for each direction
    // going forward we should just calculate the explicit values so we dont need to 
    // be building these every time
    case 2: return tform * pitch * yaw;
    case 3: return tform * pitch * yaw * yaw;
    case 4: return tform * pitch * yaw * yaw * yaw;
    case 0: return tform * pitch * pitch;
    case 1:
    default:
      return tform;
  }
}
