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
#include "dji_guidance/set_camera_id.h"
#include "dji_guidance/set_exposure_param.h"
#include "dji_guidance/multi_image.h"

#include <geometry_msgs/TransformStamped.h> //IMU
#include <geometry_msgs/Vector3Stamped.h> //velocity
#include <sensor_msgs/LaserScan.h> //obstacle distance & ultrasonic
#include <time.h>
#include <chrono>

ros::Publisher depth_image_pub;
ros::Publisher disparity_image_pub;
ros::Publisher left_image_pub;
ros::Publisher right_image_pub;
ros::Publisher imu_pub;
ros::Publisher obstacle_distance_pub;
ros::Publisher velocity_pub;
ros::Publisher ultrasonic_pub;
ros::Publisher image_pub;

ros::Subscriber set_camera_id_sub;
ros::Subscriber set_exposure_param_sub;

using namespace cv;

// JDS: Some of this should be moved to parameters on the rewrite
#define WIDTH 320
#define HEIGHT 240
#define IMAGE_SIZE (HEIGHT * WIDTH)

//#define DEBUG_MSG

// for setting exposure
char            key       = 0;
e_vbus_index    CAMERA_ID = get_default_camera_id();
exposure_param  exposure_para = get_default_exposure_param(CAMERA_ID);
DJI_lock        g_lock;
DJI_event       g_event;
Mat             g_greyscale_image_left(HEIGHT, WIDTH, CV_8UC1);
Mat             g_greyscale_image_right(HEIGHT, WIDTH, CV_8UC1);
Mat             g_depth(HEIGHT,WIDTH,CV_16SC1);
Mat             depth8(HEIGHT, WIDTH, CV_8UC1);
Mat             g_disparity(HEIGHT,WIDTH,CV_16SC1);
Mat             disparity8(HEIGHT, WIDTH, CV_8UC1);

// Avoid looking from the same camera twice in a row if possible (it isnt)
e_vbus_index camera_pair_sequence[][2] = {
    {e_vbus2, e_vbus3},
    {e_vbus4, e_vbus5},
    {e_vbus2, e_vbus4},
    {e_vbus3, e_vbus5},
    {e_vbus2, e_vbus5},
    {e_vbus3, e_vbus4},
};
// e_vbus_index camera_pair_sequence[][2] = {
//     {e_vbus4, e_vbus3},
// };

int sequence_index = 0;
int sequence_count = (sizeof(camera_pair_sequence) / sizeof(camera_pair_sequence[0]));

int frame_count = 0;
int frame_delay = 2;
int frame_cycle = 3;

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

dji_guidance::depth_image create_image_message(image_data* data, e_vbus_index idx) {
    dji_guidance::depth_image img;
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

// make sure that the data we are looking for in the current sequence is in the image_data
bool check_sequence_data(image_data* data) {
    return data->m_depth_image[camera_pair_sequence[sequence_index][0]]
        && data->m_depth_image[camera_pair_sequence[sequence_index][1]];
}

void select_camera_pair(int index) {
    int err_code = select_depth_image(camera_pair_sequence[index][0]);
    if (err_code) {
        std::cout << "Could not select camera " << camera_pair_sequence[index][0] << std::endl;
    }
    err_code = select_depth_image(camera_pair_sequence[index][1]);
    if (err_code) {
        std::cout << "Could not select camera " << camera_pair_sequence[index][1] << std::endl;
    }
}

bool check_sequence_validity(image_data* data, e_vbus_index index) {
    return data->m_depth_image[index] && !(camera_pair_sequence[sequence_index][0] == index || camera_pair_sequence[sequence_index][1] == index);
}


using namespace std::chrono;
milliseconds start = duration_cast< milliseconds >(system_clock::now().time_since_epoch());
int frames = 0;

int my_callback(int data_type, int data_len, char *content)
{
    g_lock.enter();

    /* image data */
    if (e_image == data_type && NULL != content)
    {        
        image_data* data = (image_data*)content;

        //std::cout << "Recieved images: ";
        bool gotImages = false;
        for (int i = 0; i < 5; i++) {
            if ( data->m_greyscale_image_left[i] ){
                memcpy(g_greyscale_image_left.data, data->m_greyscale_image_left[i], IMAGE_SIZE);
                //imshow("left",  g_greyscale_image_left);
                // publish left greyscale image
                cv_bridge::CvImage left_8;
                g_greyscale_image_left.copyTo(left_8.image);
                left_8.header.frame_id  = "guidance";
                left_8.header.stamp = ros::Time::now();
                left_8.encoding     = sensor_msgs::image_encodings::MONO8;
                left_image_pub.publish(left_8.toImageMsg());
                //std::cout << i << "L, ";
                gotImages = true;
            }
            if ( data->m_greyscale_image_right[i] ){
                memcpy(g_greyscale_image_right.data, data->m_greyscale_image_right[i], IMAGE_SIZE);
                //imshow("right", g_greyscale_image_right);
                // publish right greyscale image
                cv_bridge::CvImage right_8;
                g_greyscale_image_right.copyTo(right_8.image);
                right_8.header.frame_id  = "guidance";
                right_8.header.stamp     = ros::Time::now();
                right_8.encoding     = sensor_msgs::image_encodings::MONO8;
                right_image_pub.publish(right_8.toImageMsg());
                //std::cout << i << "R, ";
                gotImages = true;
            }
        }
        //std::cout << std::endl;

        if (gotImages) {
            frames += 1;
        }

        milliseconds now = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
        if ((now - start).count() > 1000) {
            std::cout << "SDK pull rate: " << ((float)frames / (now - start).count()) * 1000.f << std::endl;
            start = now;
            frames = 0;
        }

        // if ( data->m_greyscale_image_left[CAMERA_ID] ){
        //     memcpy(g_greyscale_image_left.data, data->m_greyscale_image_left[CAMERA_ID], IMAGE_SIZE);
        //     //imshow("left",  g_greyscale_image_left);
        //     // publish left greyscale image
        //     cv_bridge::CvImage left_8;
        //     g_greyscale_image_left.copyTo(left_8.image);
        //     left_8.header.frame_id  = "guidance";
        //     left_8.header.stamp = ros::Time::now();
        //     left_8.encoding     = sensor_msgs::image_encodings::MONO8;
        //     left_image_pub.publish(left_8.toImageMsg());
        // }
        // if ( data->m_greyscale_image_right[CAMERA_ID] ){
        //     memcpy(g_greyscale_image_right.data, data->m_greyscale_image_right[CAMERA_ID], IMAGE_SIZE);
        //     //imshow("right", g_greyscale_image_right);
        //     // publish right greyscale image
        //     cv_bridge::CvImage right_8;
        //     g_greyscale_image_right.copyTo(right_8.image);
        //     right_8.header.frame_id  = "guidance";
        //     right_8.header.stamp     = ros::Time::now();
        //     right_8.encoding     = sensor_msgs::image_encodings::MONO8;
        //     right_image_pub.publish(right_8.toImageMsg());
        // }

        // disabling the old depth image because its going to be too hard to sync CAMERA_ID with the
        // new sequence system. I am leaving this here until we do the node rewrite so that I can 
        // reference it during the rewrite if we want to change it to publish single depth images
        // if ( data->m_depth_image[CAMERA_ID] ){
        //     memcpy(g_depth.data, data->m_depth_image[CAMERA_ID], IMAGE_SIZE * 2);
        //     //g_depth.convertTo(depth8, CV_8UC1);
        //     //imshow("depth", depth8);
        //     //publish depth image
        //     cv_bridge::CvImage depth_16;
        //     g_depth.copyTo(depth_16.image);
        //     depth_16.header.frame_id  = "guidance";
        //     depth_16.header.stamp     = ros::Time::now();
        //     depth_16.encoding     = sensor_msgs::image_encodings::MONO16;
        //     depth_image_pub.publish(depth_16.toImageMsg());
        // }

        if ( false && data->m_disparity_image[CAMERA_ID] ){
            memcpy(g_disparity.data, data->m_disparity_image[CAMERA_ID], IMAGE_SIZE * 2);
            //g_disparity.convertTo(disparity8, CV_8UC1);
            //imshow("disparity", disparity8);
            //publish disparity image
            cv_bridge::CvImage disparity_16;
            g_disparity.copyTo(disparity_16.image);
            disparity_16.header.frame_id  = "guidance";
            disparity_16.header.stamp     = ros::Time::now();
            disparity_16.encoding         = sensor_msgs::image_encodings::MONO16;
            disparity_image_pub.publish(disparity_16.toImageMsg());
        }

        // Publish a multi_image of depth data        
        if (false && frame_count >= frame_delay) {
            if (false) {
                std::cout << "Image data pointers: " 
                          << (data->m_depth_image[0] != 0)
                          << ", " << (data->m_depth_image[1] != 0)
                          << ", " << (data->m_depth_image[2] != 0)
                          << ", " << (data->m_depth_image[3] != 0)
                          << ", " << (data->m_depth_image[4] != 0)
                          << ". Camera indices: " 
                          << camera_pair_sequence[sequence_index][0] 
                          << ", " << camera_pair_sequence[sequence_index][1] 
                          << std::endl;
            }

            if (check_sequence_data(data)) {
                dji_guidance::multi_image msg;

                msg.images.push_back(create_image_message(data, camera_pair_sequence[sequence_index][0]));
                msg.images.push_back(create_image_message(data, camera_pair_sequence[sequence_index][1]));

                //std::cout << "publishing images for cameras: " << camera_pair_sequence[sequence_index][0] << ", " << camera_pair_sequence[sequence_index][1] << std::endl;
                image_pub.publish(msg);

                frames++;
                milliseconds now = duration_cast<milliseconds>(system_clock::now().time_since_epoch());
                if ((now - start).count() > 1000) {
                    std::cout << "SDK pull rate: " << ((float)frames / (now - start).count()) * 1000.f << std::endl;
                    start = now;
                    frames = 0;
                }
            }
        }

        if (false && frame_count >= frame_cycle) {
            // select next camera pairs in the sequence
            sequence_index = (sequence_index + 1) % sequence_count;

            //std::cout << "selecting these new indices: " << camera_pair_sequence[sequence_index][0] << ", " << camera_pair_sequence[sequence_index][1] << std::endl;
            int err_code = stop_transfer();
            if (err_code) {
                std::cout << "Error when stopping transfer while trying to switch cameras " << err_code << std::endl;
            }
            reset_config();
            select_camera_pair(sequence_index);
            err_code = start_transfer();
            if (err_code) {
                std::cout << "Error when restarting transfer while trying to switch cameras " << err_code << std::endl;
            }
            frame_count = 0;
        } else {
            frame_count++;
        }
    }

    /* imu */
    if ( e_imu == data_type && NULL != content )
    {
        imu *imu_data = (imu*)content;
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
    }
    /* velocity */
    if ( e_velocity == data_type && NULL != content )
    {
        velocity *vo = (velocity*)content;
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
    }

    /* obstacle distance */
    if ( e_obstacle_distance == data_type && NULL != content )
    {
        obstacle_distance *oa = (obstacle_distance*)content;
#if DEBUG_MSG
        printf( "frame index: %d, stamp: %d\n", oa->frame_index, oa->time_stamp );
        printf( "obstacle distance:" );
        for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
        {
            printf( " %f ", 0.01f * oa->distance[i] );
        }
        printf( "\n" );
#endif

        // publish obstacle distance
        sensor_msgs::LaserScan g_oa;
        g_oa.ranges.resize(CAMERA_PAIR_NUM);
        g_oa.header.frame_id = "guidance";
        g_oa.header.stamp    = ros::Time::now();
        for ( int i = 0; i < CAMERA_PAIR_NUM; ++i )
            g_oa.ranges[i] = 0.01f * oa->distance[i];
        obstacle_distance_pub.publish(g_oa);
    }

    /* ultrasonic */
    if ( e_ultrasonic == data_type && NULL != content )
    {
        ultrasonic_data *ultrasonic = (ultrasonic_data*)content;
#if DEBUG_MSG
        printf( "frame index: %d, stamp: %d\n", ultrasonic->frame_index, ultrasonic->time_stamp );
        for ( int d = 0; d < CAMERA_PAIR_NUM; ++d )
        {
            printf( "ultrasonic distance: %f, reliability: %d\n", ultrasonic->ultrasonic[d] * 0.001f, (int)ultrasonic->reliability[d] );
        }
#endif
    
        // publish ultrasonic data
        sensor_msgs::LaserScan g_ul;
        g_ul.ranges.resize(CAMERA_PAIR_NUM);
        g_ul.intensities.resize(CAMERA_PAIR_NUM);
        g_ul.header.frame_id = "guidance";
        g_ul.header.stamp    = ros::Time::now();
        for ( int d = 0; d < CAMERA_PAIR_NUM; ++d ){
            g_ul.ranges[d] = 0.001f * ultrasonic->ultrasonic[d];
            g_ul.intensities[d] = 1.0 * ultrasonic->reliability[d];
        }
        ultrasonic_pub.publish(g_ul);
    }

    g_lock.leave();
    g_event.set_event();

    return 0;
}

void set_camera_id_callback(const dji_guidance::set_camera_idConstPtr& msg) {
    e_vbus_index idx = static_cast<e_vbus_index>(msg->cameraID);
    if(idx >= 0 && idx < CAMERA_PAIR_NUM) {
        std::cout << "Changing camera to: " << idx << std::endl;
        int err_code = stop_transfer();
        //RETURN_IF_ERR(err_code);
        reset_config();
        
        CAMERA_ID = idx;
        select_greyscale_image(CAMERA_ID, true);
        select_greyscale_image(CAMERA_ID, false);
        select_disparity_image(CAMERA_ID);

        err_code = start_transfer();
        //RETURN_IF_ERR(err_code);
    }
}

void set_exposure_param_callback(const dji_guidance::set_exposure_paramConstPtr& msg) {
    if(msg->camera_pair_index != CAMERA_ID) {
        std::cout << "Setting camera exposure parameters for an unselected camera! Setting: " << msg->camera_pair_index << ", Selected: " << CAMERA_ID << std::endl;
    }
    exposure_para = get_exposure_param(msg->step, msg->exposure_time, msg->expected_brightness, msg->is_auto_exposure, (int)msg->camera_pair_index);
    std::cout << "Setting exposure parameters....SensorId=" << CAMERA_ID << std::endl;
    set_exposure_param(&exposure_para);
}

int main(int argc, char** argv)
{
    /* initialize ros */
    ros::init(argc, argv, "GuidanceNode");
    ros::NodeHandle my_node;
    
    depth_image_pub         = my_node.advertise<sensor_msgs::Image>("/guidance/depth_image",1);
    disparity_image_pub     = my_node.advertise<sensor_msgs::Image>("/guidance/disparity_image",1);
    left_image_pub          = my_node.advertise<sensor_msgs::Image>("/guidance/left_image",5);
    right_image_pub         = my_node.advertise<sensor_msgs::Image>("/guidance/right_image",5);
    imu_pub                 = my_node.advertise<geometry_msgs::TransformStamped>("/guidance/imu",1);
    velocity_pub            = my_node.advertise<geometry_msgs::Vector3Stamped>("/guidance/velocity",1);
    obstacle_distance_pub   = my_node.advertise<sensor_msgs::LaserScan>("/guidance/obstacle_distance",1);
    ultrasonic_pub          = my_node.advertise<sensor_msgs::LaserScan>("/guidance/ultrasonic",1);
    image_pub               = my_node.advertise<dji_guidance::multi_image>("/guidance/depth_images",1);

    set_camera_id_sub       = my_node.subscribe(guidance::SET_CAMERA_ID, 10, set_camera_id_callback);
    set_exposure_param_sub  = my_node.subscribe(guidance::SET_EXPOSURE_PARAM, 10, set_exposure_param_callback);

    /* initialize guidance */
    reset_config();
    std::cout << "Pre  init_transfer" << std::endl;
    int err_code = init_transfer();
    std::cout << "Post init_transfer" << std::endl;
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
    err_code = select_greyscale_image(e_vbus2, false);
    err_code = select_greyscale_image(e_vbus2, true);
    
    err_code = select_greyscale_image(e_vbus3, false);
    err_code = select_greyscale_image(e_vbus3, true);
    
    err_code = select_greyscale_image(e_vbus4, false);
    err_code = select_greyscale_image(e_vbus4, true);

    err_code = select_greyscale_image(e_vbus5, false);
    err_code = select_greyscale_image(e_vbus5, true);
    
    //err_code = select_greyscale_image(CAMERA_ID, true);
    RETURN_IF_ERR(err_code);
    //err_code = select_greyscale_image(CAMERA_ID, false);
    RETURN_IF_ERR(err_code);
    // err_code = select_depth_image(CAMERA_ID);
    // RETURN_IF_ERR(err_code);
    //err_code = select_disparity_image(CAMERA_ID);
    RETURN_IF_ERR(err_code);

    set_image_frequecy(e_frequecy_20);

    //select_camera_pair(sequence_index);

    // select_imu();
    // select_ultrasonic();
    // select_obstacle_distance();
    // select_velocity();
    /* start data transfer */
    err_code = set_sdk_event_handler(my_callback);
    RETURN_IF_ERR(err_code);
    err_code = start_transfer();
    RETURN_IF_ERR(err_code);
    
    std::cout << "start_transfer" << std::endl;

    while (ros::ok()) {
        ros::spinOnce();
    }

    /* release data transfer */
    err_code = stop_transfer();
    RETURN_IF_ERR(err_code);
    //make sure the ack packet from GUIDANCE is received
    sleep(1);
    std::cout << "release_transfer" << std::endl;
    err_code = release_transfer();
    RETURN_IF_ERR(err_code);

    return 0;
}
