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
// Start up a new UAVMapper node.
//
///////////////////////////////////////////////////////////////////////////////

#include <uav_mapper/uav_mapper.h>
#include <message_synchronizer/message_synchronizer.h>

// Constructor/destructor.
UAVMapper::UAVMapper() : initialized_(false) {
  map_cloud_.reset(new PointCloud);
}

UAVMapper::~UAVMapper() {}

// Initialize.
bool UAVMapper::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "uav_mapper");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  // Octree holds references to points in map_cloud_.
  map_octree_.reset(new Octree(octree_resolution_));
  map_octree_->setInputCloud(map_cloud_);

  initialized_ = true;
  return true;
}

// Load parameters.
bool UAVMapper::LoadParameters(const ros::NodeHandle& n) {
  if (!ros::param::get("/uav_slam/octree/octree_res", octree_resolution_))
    return false;

  return true;
}

// Register callbacks.
bool UAVMapper::RegisterCallbacks(const ros::NodeHandle& n) {
  return true;
}

// Find nearest neighbors.
bool UAVMapper::NearestNeighbors(const PointCloud::Ptr cloud,
                                 PointCloud::Ptr neighbors) {
  neighbors->points.clear();

  // For each point in input cloud, append nearest neighbor to neighbors.
  for (size_t ii = 0; ii < cloud->points.size(); ii++) {
    float nn_distance = -1.0;
    int nn_index = -1;

    // Check valid point.
    if (std::isnan(cloud->points[ii].x) ||
        std::isnan(cloud->points[ii].y) ||
        std::isnan(cloud->points[ii].z)) {
      ROS_WARN("%s: Skipping nan point.", name_.c_str());
      continue;
    }

    map_octree_->approxNearestSearch(cloud->points[ii], nn_index, nn_distance);
    if (nn_index >= 0) {
      neighbors->push_back(map_cloud_->points[nn_index]);
    }
  }

  return neighbors->points.size() > 0;
}

// K-nearest neighbor search.
bool UAVMapper::KNearestNeighbors(const pcl::PointXYZ& point, int k,
                                  std::vector<pcl::PointXYZ>& neighbors) {
  neighbors.clear();

  // Get neighbbors from octree.
  std::vector<int> nn_indices;
  std::vector<float> nn_distances;
  if (map_octree_->nearestKSearch(point, k, nn_indices, nn_distances)) {
    if (nn_indices.size() <= 0) {
      ROS_ERROR("%s: Did not find any nearest neighbors.", name_.c_str());
      return false;
    }
    if (nn_indices.size() < k) {
      ROS_WARN("%s: Found only %d of %d requested nearest neighbors.",
               name_.c_str(), nn_indices.size(), k);
      return false;
    }

    // Query into map cloud and get actual points back.
    for (size_t ii = 0; ii < nn_indices.size(); ii++)
      neighbors.push_back(map_cloud_->points[ nn_indices[ii] ]);
  }

  return true;
}


// Add points to map.
void UAVMapper::InsertPoints(const PointCloud& cloud) {
  for (size_t ii = 0; ii < cloud.points.size(); ii++) {
    const pcl::PointXYZ point = cloud.points[ii];

    // Add all points to map_cloud_, but only add to octree if voxel is empty.
    if (!map_octree_->isVoxelOccupiedAtPoint(point))
      map_octree_->addPointToCloud(point, map_cloud_);
#if 0
    else
      map_cloud_->push_back(point);
#endif
  }
}

// Size.
size_t UAVMapper::Size() {
  return map_cloud_->points.size();
}
