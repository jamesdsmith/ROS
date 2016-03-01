/*
 * GuidanceNode.cpp
 *
 *  Created on: Apr 29, 2015
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
#include "dji_guidance/multi_image.h"

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic

ros::Publisher image_pub;
ros::Publisher imu_pub;
ros::Publisher velocity_pub;

using namespace cv;

// JDS: Some of this should be moved to parameters on the rewrite
#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

// for setting exposure
char            key       = 0;
e_vbus_index    CAMERA_ID = get_default_camera_id();
exposure_param  exposure_para = get_default_exposure_param(CAMERA_ID);
DJI_lock        g_lock;
DJI_event       g_event;
Mat             g_depth(HEIGHT,WIDTH,CV_16SC1);

std::ostream& operator<<(std::ostream& out, const e_sdk_err_code value){
    const char* s = 0;
    static char str[100]={0};
#define PROCESS_VAL(p) case(p): s = #p; break;
    switch(value){
        PROCESS_VAL(e_OK);     
        PROCESS_VAL(e_load_libusb_err);     
        PROCESS_VAL(e_sdk_not_inited);
        PROCESS_VAL(e_disparity_not_allowed);
        PROCESS_VAL(e_image_frequency_not_allowed);
        PROCESS_VAL(e_config_not_ready);
        PROCESS_VAL(e_online_flag_not_ready);
        PROCESS_VAL(e_stereo_cali_not_ready);
        PROCESS_VAL(e_libusb_io_err);
        PROCESS_VAL(e_timeout);
    default:
        strcpy(str, "Unknown error");
        s = str;
        break;
    }
#undef PROCESS_VAL

    return out << s;
}

dji_guidance::guidance_image create_image_message(image_data* data, e_vbus_index idx) {
    dji_guidance::guidance_image img;
    if (data->m_depth_image[idx]) {
        img.vbus_index = idx;

        memcpy(g_depth.data, data->m_depth_image[idx], IMAGE_SIZE * 2);
        cv_bridge::CvImage depth_16;
        g_depth.copyTo(depth_16.image);
        depth_16.header.frame_id = "guidance";
        depth_16.header.stamp = ros::Time::now();
        depth_16.encoding = sensor_msgs::image_encodings::MONO16;

        img.image = *(depth_16.toImageMsg());

        return img;
    } else {
        std::cout << "Attempted to build an image message for an unsubscribed image channel " << idx << std::endl;
        return img;
    }
}

int my_callback(int data_type, int data_len, char *content)
{
    g_lock.enter();

    /* image data */
    if (e_image == data_type && NULL != content)
    {        
        image_data* data = (image_data*)content;
        dji_guidance::multi_image msg;

        // forward facing guidance sensor is disabled for now...
        //msg.images.push_back(create_image_message(data, e_vbus1));
        msg.images.push_back(create_image_message(data, e_vbus2));
        msg.images.push_back(create_image_message(data, e_vbus3));
        msg.images.push_back(create_image_message(data, e_vbus4));
        //msg.images.push_back(create_image_message(data, e_vbus5));

        image_pub.publish(msg);
        std::cout << "published " << msg.images.size() << " images" << std::endl;
    }

    /* imu */
    if ( e_imu == data_type && NULL != content )
    {
        imu *imu_data = (imu*)content;
        // printf( "frame index: %d, stamp: %d\n", imu_data->frame_index, imu_data->time_stamp );
        // printf( "imu: [%f %f %f %f %f %f %f]\n", imu_data->acc_x, imu_data->acc_y, imu_data->acc_z, imu_data->q[0], imu_data->q[1], imu_data->q[2], imu_data->q[3] );
    
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
    }
    /* velocity */
    if ( e_velocity == data_type && NULL != content )
    {
        velocity *vo = (velocity*)content;
        // printf( "frame index: %d, stamp: %d\n", vo->frame_index, vo->time_stamp );
        // printf( "vx:%f vy:%f vz:%f\n", 0.001f * vo->vx, 0.001f * vo->vy, 0.001f * vo->vz );
    
        // publish velocity
        geometry_msgs::Vector3Stamped g_vo;
        g_vo.header.frame_id = "guidance";
        g_vo.header.stamp    = ros::Time::now();
        g_vo.vector.x = 0.001f * vo->vx;
        g_vo.vector.y = 0.001f * vo->vy;
        g_vo.vector.z = 0.001f * vo->vz;
        velocity_pub.publish(g_vo);
    }

    g_lock.leave();
    g_event.set_event();

    return 0;
}

int main(int argc, char** argv)
{
    /* initialize ros */
    ros::init(argc, argv, "GuidanceDepthNode");
    ros::NodeHandle my_node;
    
    image_pub    = my_node.advertise<dji_guidance::multi_image>("/guidance/depth/images",1);
    imu_pub      = my_node.advertise<geometry_msgs::TransformStamped>("/guidance/depth/imu",1);
    velocity_pub = my_node.advertise<geometry_msgs::Vector3Stamped>("/guidance/depth/velocity",1);

    /* initialize guidance */
    reset_config();
    int err_code = init_transfer();
    RETURN_IF_ERR(err_code);

    int online_status[CAMERA_PAIR_NUM];
    err_code = get_online_status(online_status);
    RETURN_IF_ERR(err_code);
    std::cout << "Sensor online status: ";
    for (int i = 0; i < CAMERA_PAIR_NUM; i++) {
        std::cout << online_status[i] << " ";
    }
    std::cout << std::endl;

    // get cali param
    stereo_cali cali[CAMERA_PAIR_NUM];
    err_code = get_stereo_cali(cali);
    RETURN_IF_ERR(err_code);
    std::cout << "cu\tcv\tfocal\tbaseline\n";
    for (int i=0; i < CAMERA_PAIR_NUM; i++) {
        std::cout << cali[i].cu << "\t" << cali[i].cv << "\t" << cali[i].focal << "\t" << cali[i].baseline << std::endl;
    }
    
    /* select data */
    //err_code = select_depth_image(e_vbus1);
    //RETURN_IF_ERR(err_code);
    err_code = select_depth_image(e_vbus2);
    RETURN_IF_ERR(err_code);
    err_code = select_depth_image(e_vbus3);
    RETURN_IF_ERR(err_code);
    err_code = select_depth_image(e_vbus4);
    RETURN_IF_ERR(err_code);
    //err_code = select_depth_image(e_vbus5);
    //RETURN_IF_ERR(err_code);
    select_imu();
    select_velocity();

    /* start data transfer */
    err_code = set_sdk_event_handler(my_callback);
    RETURN_IF_ERR(err_code);
    err_code = start_transfer();
    RETURN_IF_ERR(err_code);
    
    std::cout << "start_transfer" << std::endl;

    ros::spin();

    /* release data transfer */
    err_code = stop_transfer();
    RETURN_IF_ERR(err_code);
    std::cout << std::endl;

    //make sure the ack packet from GUIDANCE is received
    sleep(1);
    std::cout << "release_transfer" << std::endl;
    err_code = release_transfer();
    RETURN_IF_ERR(err_code);

    return 0;
}
