/*
 * main_sdk0428.cpp
 *
 *  Created on: Apr 29, 2015
 *      Author: craig
 */

#include <stdio.h>
#include <string.h>
#include <iostream>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include "DJI_guidance.h"
#include "DJI_utility.h"
#include "guidance_helpers.h"
#include "dji_guidance/set_camera_id.h"
#include "dji_guidance/set_exposure_param.h"

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic
#include "dji_guidance/multi_image.h"
#include "dji_guidance/stereo_pair.h"

using namespace cv;

char            key       = 0;
e_vbus_index    CAMERA_ID = get_default_camera_id();
exposure_param  exposure_para = get_default_exposure_param(CAMERA_ID);
DJI_lock        g_lock;
DJI_event       g_event;

ros::Subscriber multi_image_sub;
ros::Subscriber stereo_pair_sub;

using namespace cv;
#define WIDTH 320
#define HEIGHT 240

/* depth greyscale image */
void multi_image_callback(const dji_guidance::multi_image::ConstPtr& msg) {
    for (auto const& img : msg->images) {
        cv_bridge::CvImagePtr cv_ptr;
        try {
          sensor_msgs::Image im = img.image;
          cv_ptr = cv_bridge::toCvCopy(im, sensor_msgs::image_encodings::MONO16);
        } catch (cv_bridge::Exception& e) {
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;
        }

        if (cv_ptr) {
            cv::Mat depth8(WIDTH, HEIGHT, CV_8UC1);
            cv_ptr->image.convertTo(depth8, CV_8UC1);
            //cv::imshow("depth_image " + std::to_string(img.vbus_index), cv_ptr->image);
            cv::imshow("depth_image " + std::to_string(img.vbus_index), depth8);
            waitKey(1);
        }
    }
}

void stereo_pair_callback(const dji_guidance::stereo_pair::ConstPtr& msg) {
    cv_bridge::CvImagePtr left_ptr;
    try {
        left_ptr = cv_bridge::toCvCopy(msg->left, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("left_image " + std::to_string(msg->vbus_index), left_ptr->image);
    cv::waitKey(1);

    cv_bridge::CvImagePtr right_ptr;
    try {
        right_ptr = cv_bridge::toCvCopy(msg->right, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("right_image " + std::to_string(msg->vbus_index), right_ptr->image);
    cv::waitKey(1);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "GuidanceDepthNodeViewer");
    ros::NodeHandle my_node;

    //multi_image_sub = my_node.subscribe("/guidance/depth_images",  1, multi_image_callback);
    stereo_pair_sub = my_node.subscribe("/guidance/stereo_pair",  5, stereo_pair_callback);
    
    int err_code = 0;

    std::cout << "Starting GuidanceNodeDepthViewer..." << std::endl;

    ros::spin();

    return 0;
}
