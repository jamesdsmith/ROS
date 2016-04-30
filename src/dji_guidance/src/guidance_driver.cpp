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
// Definitions for the GuidanceDriver class.
//
///////////////////////////////////////////////////////////////////////////////

#include <guidance_driver/guidance_driver.h>

// @TODO high priority, figure something out for this
#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

// This is the unfortunate result of not allowing us to pass an object along with
// our SDK callback function...
static GuidanceDriver* guidance_driver;

// Constructor/destructor.
GuidanceDriver::GuidanceDriver() : initialized_(false) {}
GuidanceDriver::~GuidanceDriver() {}

// Initialize.
bool GuidanceDriver::Initialize(const ros::NodeHandle& n) {
  name_ = ros::names::append(n.getNamespace(), "guidance");

  if (!LoadParameters(n)) {
    ROS_ERROR("%s: Failed to load parameters.", name_.c_str());
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("%s: Failed to register callbacks.", name_.c_str());
    return false;
  }

  if (!InitGuidanceSDK()) {
    ROS_ERROR("%s: Failed to initialize DJI Guidance SDK. Aborting startup.", name_.c_str());
    return false;
  }

  initialized_ = true;
  return true;
}

// Load parameters.
bool GuidanceDriver::LoadParameters(const ros::NodeHandle& n) {
  return true;
}

// Register callbacks.
bool GuidanceDriver::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle node(n);

  // Subscriber.
  // depth_sub_ =
  //   node.subscribe("/guidance/depth_image", 10,
  //                  &DepthCloudProjector::DepthMapCallback, this);

  // Publishers.
  depth_image_pub_       = my_node.advertise<sensor_msgs::Image>("/guidance/depth_image",1);
  disparity_image_pub_   = my_node.advertise<sensor_msgs::Image>("/guidance/disparity_image",1);
  left_image_pub_        = my_node.advertise<sensor_msgs::Image>("/guidance/left_image",5);
  right_image_pub_       = my_node.advertise<sensor_msgs::Image>("/guidance/right_image",5);
  imu_pub_               = my_node.advertise<geometry_msgs::TransformStamped>("/guidance/imu",1);
  velocity_pub_          = my_node.advertise<geometry_msgs::Vector3Stamped>("/guidance/velocity",1);
  obstacle_distance_pub_ = my_node.advertise<sensor_msgs::LaserScan>("/guidance/obstacle_distance",1);
  ultrasonic_pub_        = my_node.advertise<sensor_msgs::LaserScan>("/guidance/ultrasonic",1);
  multi_image_pub_       = my_node.advertise<dji_guidance::multi_image>("/guidance/depth_images",1);

  return true;
}

bool GuidanceDriver::InitGuidanceSDK() {
  reset_config();

  if (init_transfer() != 0) {
    ROS_ERROR("%s: There was an error initializing data transfer. Aborting startup.", name_.c_str());
    return false;
  }

  if (!UpdateOnlineStatus()) {
    ROS_ERROR("%s: Could not detect online status of Guidance sensors. Aborting startup.", name_.c_str());
    return false;
  } else {
    PrintOnlineStatus();
  }

  // @NOTE We may not actually need to pull calibration data from the guidance unit if we decide to
  // use the ROS camera calibration stuff! Stick with this for now
  // @TODO We should actually use this calibration data, for real
  if (!UpdateStereoCalibration()) {
    ROS_ERROR("%s: Could not get stereo camera calibration data from Guidance. Aborting startup.", name_.c_str());
    return false;
  } else {
    PrintStereoCalibration();
  }
  
  // Just let it ride at max frequency, we should see if it produces more consistent results
  // at a lower frequency @TODO
  if (set_image_frequecy(e_frequecy_20) != 0) {
    ROS_ERROR("%s: Could not set the image frequency in the Guidance SDK. Not aborting startup, but this could be a cause for concern.", name_.c_str());
  }

  if (!InitGuidanceData()) {
    ROS_ERROR("%s: There was an error initializing data transfer. Aborting startup.", name_.c_str());
    return false;
  }

  if (guidance_driver != NULL) {
    ROS_ERROR("%s: Tried to initialize two drivers! Aborting startup.", name_.c_str());
    return false;
  }

  if (set_sdk_event_handler(OnSDKEvent) != 0) {
    ROS_ERROR("%s: Failure while setting Guidance SDK event handler. Aborting startup.", name_.c_str());
    return false;
  } else {
    guiance_driver = this;
  }

  /* start data transfer */
  if (start_transfer() != 0) {
    ROS_ERROR("%s: Failure when starting data transfer.", name_.c_str());
    return false;
  }
  
  return true;
}

/**
 * Select which data from the SDK we want the driver to pull
 * We dont want to select everything the SDK has to save on USB bandwidth
 */
bool GuidanceDriver::InitGuidanceData() {
  // @TODO Load these from parameters!!
  return
    SelectCameraPair(cam_front) && 
    SelectCameraPair(cam_left) &&
    SelectCameraPair(cam_right) &&
    SelectCameraPair(cam_down) &&
    // SelectIMU() &&
    // SelectUltrasonic() &&
    // SelectObstacleDistance() &&
    // SelectVelocity() &&
    true;
}

/**
 * This is the function that the Guidance SDK calls back
 * All this should do is forward the call on to the GuidanceDriver object
 * so that the callback can have access to the GuidanceDriver member functions, etc
 */
int OnSDKEvent(int data_type, int data_len, char* content) {
  // It should NOT be possible for guidance_driver to be null, but you never know!
  if (guidance_driver != NULL) {
    return guidance_driver->OnSDKEvent(data_type, data_len, content);
  } else {
    ROS_ERROR("%s: Recieved SDK callback from DJI Guidance device, but there is no active guidance driver!", name_.c_str());
    return -1;
  }
}

int GuidanceDriver::OnSDKEvent(int data_type, int data_len, char* content) {
  g_lock.enter();

  if (data_type == e_image && content != NULL) {        
    ProcessImageData(content);
  }

  if (data_type == e_imu && content != NULL) {
    ProcessIMUData(content);
  }

  if (data_type == e_velocity && content != NULL) {
    ProcessVelocityData(content);
  }

  if (data_type == e_obstacle_distance && content != NULL) {
    ProcessObstacleDistanceData(content);
  }

  if (data_type == e_ultrasonic && content != NULL) {
    ProcessUltrasonicData(content);
  }

  g_lock.leave();
  g_event.set_event();

  return 0;
}

bool GuidanceDriver::ProcessImageData(char* content) {
  image_data* data = (image_data*)content;
  
  Cv::Mat greyscale_left(HEIGHT, WIDTH, CV_8UC1);
  Cv::Mat greyscale_right(HEIGHT, WIDTH, CV_8UC1);
  Cv::Mat disparity(HEIGHT, WIDTH, CV_16SC1);
  Cv::Mat depth(HEIGHT, WIDTH, CV_16SC1);

  for (int i = 0; i < CAMERA_PAIR_NUM; ++i) {
    if (data->m_greyscale_image_left[i]) {
      memcpy(greyscale_left.data, data->m_greyscale_image_left[i], IMAGE_SIZE);
      ProcessGreyscaleLeftImage(greyscale_left);
    }
    if (data->m_greyscale_image_right[i]) {
      memcpy(greyscale_right.data, data->m_greyscale_image_right[i], IMAGE_SIZE);
      ProcessGreyscaleRightImage(greyscale_right);
    }
    if (data->m_depth_image[i]) {
      memcpy(depth.data, data->m_depth_image[i], IMAGE_SIZE * 2);
      ProcessDepthImage(depth);
    }
    if (data->m_disparity_image[i]) {
      memcpy(disparity.data, data->m_disparity_image[i], IMAGE_SIZE * 2);
      ProcessDisparityImage(disparity);
    }
  }
}

bool GuidanceDriver::ProcessGreyscaleLeftImage(Cv::Math img) {
  cv_bridge::CvImage left;
  g_greyscale_image_left.copyTo(left.image);
  left.header.frame_id = "guidance";
  left.header.stamp    = ros::Time::now();
  left.encoding        = sensor_msgs::image_encodings::MONO8;
  left_image_pub.publish(left.toImageMsg());
}

bool GuidanceDriver::ProcessGreyscaleRightImage(Cv::Math img) {
  cv_bridge::CvImage right;
  g_greyscale_image_right.copyTo(right.image);
  right.header.frame_id = "guidance";
  right.header.stamp    = ros::Time::now();
  right.encoding        = sensor_msgs::image_encodings::MONO8;
  right_image_pub.publish(right.toImageMsg());
}

bool GuidanceDriver::ProcessDepthImage(Cv::Mat img) {
  cv_bridge::CvImage depth;
  g_depth.copyTo(depth.image);
  depth.header.frame_id = "guidance";
  depth.header.stamp    = ros::Time::now();
  depth.encoding        = sensor_msgs::image_encodings::MONO16;
  depth_image_pub.publish(depth.toImageMsg());
}

bool GuidanceDriver::ProcessDisparityImage(Cv::Mat img) {
  cv_bridge::CvImage disparity;
  img.copyTo(disparity.image);
  disparity.header.frame_id = "guidance";
  disparity.header.stamp    = ros::Time::now();
  disparity.encoding        = sensor_msgs::image_encodings::MONO16;
  disparity_image_pub_.publish(disparity.toImageMsg());
  return true;
}

bool GuidanceDriver::ProcessIMUData(char* content) {
  imu* imu_data = (imu*)content;

#if DEBUG_MSG
  printf( "frame index: %d, stamp: %d\n", imu_data->frame_index, imu_data->time_stamp );
  printf( "imu: [%f %f %f %f %f %f %f]\n", imu_data->acc_x, imu_data->acc_y, imu_data->acc_z, imu_data->q[0], imu_data->q[1], imu_data->q[2], imu_data->q[3] );
#endif

  // publish imu data
  geometry_msgs::TransformStamped g_imu;
  g_imu.header.frame_id = "guidance";
  g_imu.header.stamp    = ros::Time::now();
  g_imu.transform.translation.x = imu_data->acc_x;
  g_imu.transform.translation.y = imu_data->acc_y;
  g_imu.transform.translation.z = imu_data->acc_z;
  g_imu.transform.rotation.w = imu_data->q[0];
  g_imu.transform.rotation.x = imu_data->q[1];
  g_imu.transform.rotation.y = imu_data->q[2];
  g_imu.transform.rotation.z = imu_data->q[3];
  imu_pub.publish(g_imu);

  return true;
}

bool GuidanceDriver::ProcessUltrasonicData(char* content) {
  ultrasonic_data* ultrasonic = (ultrasonic_data*)content;

#if DEBUG_MSG
  printf( "frame index: %d, stamp: %d\n", ultrasonic->frame_index, ultrasonic->time_stamp );
  for ( int d = 0; d < CAMERA_PAIR_NUM; ++d ) {
      printf( "ultrasonic distance: %f, reliability: %d\n", ultrasonic->ultrasonic[d] * 0.001f, (int)ultrasonic->reliability[d] );
  }
#endif

  // publish ultrasonic data
  sensor_msgs::LaserScan g_ul;
  g_ul.ranges.resize(CAMERA_PAIR_NUM);
  g_ul.intensities.resize(CAMERA_PAIR_NUM);
  g_ul.header.frame_id = "guidance";
  g_ul.header.stamp    = ros::Time::now();

  for (int d = 0; d < CAMERA_PAIR_NUM; ++d) {
    g_ul.ranges[d] = 0.001f * ultrasonic->ultrasonic[d];
    g_ul.intensities[d] = 1.0 * ultrasonic->reliability[d];
  }
  ultrasonic_pub.publish(g_ul);

  return true;
}

bool GuidanceDriver::ProcessVelocityData(char* content) {
  velocity* vo = (velocity*)content;

#if DEBUG_MSG
  printf( "frame index: %d, stamp: %d\n", vo->frame_index, vo->time_stamp );
  printf( "vx:%f vy:%f vz:%f\n", 0.001f * vo->vx, 0.001f * vo->vy, 0.001f * vo->vz );
#endif

  // publish velocity
  geometry_msgs::Vector3Stamped g_vo;
  g_vo.header.frame_id = "guidance";
  g_vo.header.stamp    = ros::Time::now();
  g_vo.vector.x = 0.001f * vo->vx;
  g_vo.vector.y = 0.001f * vo->vy;
  g_vo.vector.z = 0.001f * vo->vz;
  velocity_pub.publish(g_vo);

  return true;
}

bool GuidanceDriver::ProcessObstacleDistanceData(char* content) {
  obstacle_distance* oa = (obstacle_distance*)content;

#if DEBUG_MSG
  printf("frame index: %d, stamp: %d\n", oa->frame_index, oa->time_stamp);
  printf("obstacle distance:");
  for (int i = 0; i < CAMERA_PAIR_NUM; ++i) {
      printf(" %f ", 0.01f * oa->distance[i]);
  }
  printf("\n");
#endif

  // publish obstacle distance
  sensor_msgs::LaserScan g_oa;
  g_oa.ranges.resize(CAMERA_PAIR_NUM);
  g_oa.header.frame_id = "guidance";
  g_oa.header.stamp    = ros::Time::now();
  for (int i = 0; i < CAMERA_PAIR_NUM; ++i) {
    g_oa.ranges[i] = 0.01f * oa->distance[i];
  }
  obstacle_distance_pub.publish(g_oa);

  return true;
}

bool GuidanceDriver::SelectCameraPair(camera_direction dir) {
  // Select right
  if (select_greyscale_image(dir, false)) {
    return false;
  }
  // Select left
  if (select_greyscale_image(dir, true)) {
    return false;
  }
}

bool GuidanceDriver::SelectIMU() {
  select_imu();
  return true;
}

bool GuidanceDriver::SelectUltrasonic() {
  select_ultrasonic();
  return true;
}

bool GuidanceDriver::SelectVelocity() {
  select_obstacle_distance();
  return true;
}

bool GuidanceDriver::SelectObstacleDistance() {
  select_velocity();
  return true;
}

bool GuidanceDriver::UpdateOnlineStatus() {
  return get_online_status(online_status_) == 0;
}

bool GuidanceDriver::UpdateStereoCalibration() {
  return get_stereo_cali(stereo_calibration_) == 0;
}

void GuidanceDriver::PrintOnlineStatus() {
  std::cout << "Sensor online status: ";
  for (int i = 0; i < CAMERA_PAIR_NUM; ++i) {
    std::cout << online_status[i] << " ";
  }
  std::cout << std::endl;
}

void GuidanceDriver::PrintStereoCalibration() {
  std::cout << "cu\tcv\tfocal\tbaseline" << std::endl;
  for (int i = 0; i < CAMERA_PAIR_NUM; ++i) {
    std::cout << cali[i].cu << "\t" << cali[i].cv << "\t" << cali[i].focal << "\t" << cali[i].baseline << std::endl;
  }
}
