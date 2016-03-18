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
// This defines the surface_fitting node.
//
///////////////////////////////////////////////////////////////////////////////

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <surface_fitting/gp_surface_estimator.h>
#include <utils/math/random_generator.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

int main(int argc, char** argv) {
  // Generate a new node.
  ros::init(argc, argv, "surface_fitting");
  ros::NodeHandle n("~");

  // Create a new GPSurfaceEstimator.
  GPSurfaceEstimator gp;

  // Pick 30 random points on the unit sphere.
  math::RandomGenerator rng(0);
  PointCloud cloud;
  for (size_t ii = 0; ii < 100; ii++) {
    double theta = rng.DoubleUniform(0.0, 2.0*M_PI);
    double phi = rng.DoubleUniform(0.0, M_PI);
    double x = std::sin(phi) * std::cos(theta);
    double y = std::sin(phi) * std::sin(theta);
    double z = std::cos(phi);

    pcl::PointXYZ p;
    p.x = x; p.y = y; p.z = z;
    cloud.push_back(p);
  }

  // Initialize a mapper.
  UAVMapper mapper;
  if (!mapper.Initialize(n)) {
    ROS_ERROR("%s: Failed to initialize UAVMapper.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  // Initialize GP.
  if (!gp.Initialize(n, &mapper)) {
    ROS_ERROR("%s: Failed to initialize GPSurfaceEstimator.",
              ros::this_node::getName().c_str());
    return EXIT_FAILURE;
  }

  ros::spin();
  return EXIT_SUCCESS;
}
