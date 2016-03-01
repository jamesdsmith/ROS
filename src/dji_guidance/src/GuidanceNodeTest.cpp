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
#include "guidance/set_camera_id.h"
#include "guidance/set_exposure_param.h"

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance && ultrasonic

using namespace cv;

char            key       = 0;
e_vbus_index    CAMERA_ID = get_default_camera_id();
exposure_param  exposure_para = get_default_exposure_param(CAMERA_ID);
DJI_lock        g_lock;
DJI_event       g_event;

ros::Subscriber left_image_sub;
ros::Subscriber right_image_sub;
ros::Subscriber depth_image_sub;
ros::Subscriber disparity_image_sub;
ros::Subscriber imu_sub;
ros::Subscriber velocity_sub;
ros::Subscriber obstacle_distance_sub;
ros::Subscriber ultrasonic_sub;

ros::Publisher set_camera_id_pub;
ros::Publisher set_exposure_param_pub;

using namespace cv;
#define WIDTH 320
#define HEIGHT 240

/* left greyscale image */
void left_image_callback(const sensor_msgs::ImageConstPtr& left_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(left_img, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("left_image", cv_ptr->image);
    cv::waitKey(1);
}

/* right greyscale image */
void right_image_callback(const sensor_msgs::ImageConstPtr& right_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(right_img, sensor_msgs::image_encodings::MONO8);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::imshow("right_image", cv_ptr->image);
    cv::waitKey(1);
}

/* depth greyscale image */
void depth_image_callback(const sensor_msgs::ImageConstPtr& depth_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(depth_img, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat depth8(HEIGHT, WIDTH, CV_8UC1);
    cv_ptr->image.convertTo(depth8, CV_8UC1);
    cv::imshow("depth_image", depth8);
    cv::waitKey(1);
}

/* depth greyscale image */
void disparity_image_callback(const sensor_msgs::ImageConstPtr& disparity_img)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(disparity_img, sensor_msgs::image_encodings::MONO16);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv::Mat disparity8(HEIGHT, WIDTH, CV_8UC1);
    cv_ptr->image.convertTo(disparity8, CV_8UC1);
    cv::imshow("disparity_image", disparity8);
    cv::waitKey(1);
}

/* imu */
void imu_callback(const geometry_msgs::TransformStamped& g_imu)
{ 
    printf( "frame_id: %s stamp: %d\n", g_imu.header.frame_id.c_str(), g_imu.header.stamp.sec );
    printf( "imu: [%f %f %f %f %f %f %f]\n", g_imu.transform.translation.x, g_imu.transform.translation.y, g_imu.transform.translation.z, 
						g_imu.transform.rotation.x, g_imu.transform.rotation.y, g_imu.transform.rotation.z, g_imu.transform.rotation.w );
}

/* velocity */
void velocity_callback(const geometry_msgs::Vector3Stamped& g_vo)
{ 
    printf( "frame_id: %s stamp: %d\n", g_vo.header.frame_id.c_str(), g_vo.header.stamp.sec );
    printf( "velocity: [%f %f %f]\n", g_vo.vector.x, g_vo.vector.y, g_vo.vector.z );
}

/* obstacle distance */
void obstacle_distance_callback(const sensor_msgs::LaserScan& g_oa)
{ 
    printf( "frame_id: %s stamp: %d\n", g_oa.header.frame_id.c_str(), g_oa.header.stamp.sec );
    printf( "obstacle distance: [%f %f %f %f %f]\n", g_oa.ranges[0], g_oa.ranges[1], g_oa.ranges[2], g_oa.ranges[3], g_oa.ranges[4] );
}

/* ultrasonic */
void ultrasonic_callback(const sensor_msgs::LaserScan& g_ul)
{ 
    printf( "frame_id: %s stamp: %d\n", g_ul.header.frame_id.c_str(), g_ul.header.stamp.sec );
    for (int i = 0; i < 5; i++)
        printf( "ultrasonic distance: [%f]  reliability: [%d]\n", g_ul.ranges[i], (int)g_ul.intensities[i] );
}

int main(int argc, char** argv)
{
    if(argc > 1) {
        printf("This is demo program showing data from Guidance.\n\t" 
            " 'a','d','w','s','x' to select sensor direction.\n\t"
            " 'j','k' to change the exposure parameters.\n\t"
            " 'm' to switch between AEC and constant exposure modes.\n\t"
            " 'n' to return to default exposure mode and parameters.\n\t"
            " 'q' to quit.\n");
        return 0;
    }

    ros::init(argc, argv, "GuidanceNodeTest");
    ros::NodeHandle my_node;

    left_image_sub         = my_node.subscribe("/guidance/left_image",  10, left_image_callback);
    right_image_sub        = my_node.subscribe("/guidance/right_image", 10, right_image_callback);
    depth_image_sub        = my_node.subscribe("/guidance/depth_image", 10, depth_image_callback);
    disparity_image_sub    = my_node.subscribe("/guidance/depth_image", 10, disparity_image_callback);
    imu_sub                = my_node.subscribe("/guidance/imu", 1, imu_callback);
    velocity_sub           = my_node.subscribe("/guidance/velocity", 1, velocity_callback);
    obstacle_distance_sub  = my_node.subscribe("/guidance/obstacle_distance", 1, obstacle_distance_callback);
    ultrasonic_sub         = my_node.subscribe("/guidance/ultrasonic", 1, ultrasonic_callback);

    set_camera_id_pub      = my_node.advertise<guidance::set_camera_id>(guidance::SET_CAMERA_ID, 10);
    set_exposure_param_pub = my_node.advertise<guidance::set_exposure_param>(guidance::SET_EXPOSURE_PARAM, 10);
    
    int err_code = 0;

    std::cout << "Starting GuidanceNodeTest..." << std::endl;
    // Open windows at startup so we can process wait_event
    cv::Mat disparity8(HEIGHT, WIDTH, CV_8UC1);
    cv::imshow("disparity_image", disparity8);

    while (ros::ok()) {
        //g_event.wait_event();
        key = waitKey(1);
        if (key > 0) {
            if(key == 'q') {
                // quit the loop
                std::cout << "Quitting..." << std::endl;
                break;
            } else if(key == 'j' || key == 'k' || key == 'm' || key == 'n') {
                // set exposure parameters
                if(key == 'j') {
                    if(exposure_para.m_is_auto_exposure) {
                        exposure_para.m_expected_brightness += 20;
                    } else {
                        exposure_para.m_exposure_time += 3;
                    }
                } else if(key == 'k') {
                    if(exposure_para.m_is_auto_exposure) {
                        exposure_para.m_expected_brightness -= 20;
                    } else {
                        exposure_para.m_exposure_time -= 3;
                    }
                } else if(key == 'm') {
                    exposure_para.m_is_auto_exposure = !exposure_para.m_is_auto_exposure;
                    std::cout << "exposure is " << exposure_para.m_is_auto_exposure << std::endl;
                } else if(key == 'n') {
                    //return to default
                    exposure_para.m_expected_brightness = exposure_para.m_exposure_time = 0;
                }

                std::cout << "Setting exposure parameters....SensorId=" << CAMERA_ID << std::endl;
                exposure_para.m_camera_pair_index = CAMERA_ID;

                // Send change exposure message over to GuidanceNode
                set_exposure_param_pub.publish(get_exposure_msg(exposure_para));
                key = 0;
            } else if(key == 'w' || key == 'd' || key == 'x' || key == 'a' || key == 's') {
                // switch image direction
                if (key == 'w') {
                    CAMERA_ID = e_vbus1;
                } else if (key == 'd') {
                    CAMERA_ID = e_vbus2;
                } else if (key == 'x') {
                    CAMERA_ID = e_vbus3;
                } else if (key == 'a') {
                    CAMERA_ID = e_vbus4;
                } else if (key == 's') {
                    CAMERA_ID = e_vbus5;
                }

                std::cout << "Change camera: " << CAMERA_ID << std::endl;

                // Send change camera id message over to GuidanceNode
                guidance::set_camera_id msg;
                msg.cameraID = CAMERA_ID;
                set_camera_id_pub.publish(msg);
                key = 0;
            }
        }
        ros::spinOnce();
    }

    return 0;
}
