#include <iostream>
#include <exception>
#include <sstream>
#include <math.h>
#include <algorithm>
#include <string>
#include <vector>
   
#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>

#include <dead_reckoning/EspMeasurements.h>
#include <dead_reckoning/WheelSpeedMeasurements.h>
#include <dead_reckoning/GearState.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/types_c.h> 
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"
 
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// ------------------------------ DEFINES -------------------------------
// Dead Reckoning 
#define DEG2RAD M_PI/180
#define KpH2MpS 0.277778
#define Mps2KpH 3.6

// AVM 
// #define IMG_WIDTH 200
// #define IMG_HEIGHT 200

// #define REAL_OCCUPANCY_SIZE_X (double)17.5  // AVM_IMG_WIDTH 400PIX == 25Meter // 35*13/18  
// #define REAL_OCCUPANCY_SIZE_Y (double)17.5 // AVM_IMG_HEIGHT 400PIX == 25Meter

// #define REAL_OCCUPANCY_SIZE_X (double)8.5 * 4.555 / 5.2  // AVM_IMG_WIDTH 400PIX == 25Meter // 35*13/18  
// #define REAL_OCCUPANCY_SIZE_Y (double)8.5 * 4.555 / 5.2 // AVM_IMG_HEIGHT 400PIX == 25Meter

// #define PIXEL_PER_METER  (double)IMG_WIDTH/REAL_OCCUPANCY_SIZE_X           //400PIX / 25M
// #define METER_PER_PIXEL  (double)(1.0/(double)(PIXEL_PER_METER)) // 0.0625    
// ------------------------------------------------------------------------

int m_AVM_IMG_WIDTH = 0, m_AVM_IMG_HEIGHT = 0, m_SLAM_IMG_WIDTH = 0;
double m_REAL_OCCUPANCY_SIZE_X = 0.0, m_REAL_OCCUPANCY_SIZE_Y = 0.0;
double m_SLAM_OCCUPANCY_SIZE_X = 0.0, m_SLAM_OCCUPANCY_SIZE_Y = 0.0;
double m_METER_PER_PIXEL = 0.0;

// ------------------------------ CONSTANTS -------------------------------
// Dead Reckoning 
nav_msgs::Path dr_path_msg;
geometry_msgs::PoseStamped pose_dr;

double yaw_gain = 1.0;
double m_prev_can_hw_timestamp = 0.0;

bool m_parkingStartFlag = true;
bool m_forward = true;

double m_center_to_rear = 0.0;
double m_rear_wheel_distance = 1.652;
double m_backward_yaw_gain = 1.0;

int cnt = 0;

struct CARPOSE {
    double x, y, th, vel, dir;
};
CARPOSE m_car;

// AVM
cv::Mat Temp_SrcImg = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3);
cv::Mat SrcImg= cv::Mat::zeros( m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3);
// ------------------------------------------------------------------------


// ------------------------------ CALLBACKS -------------------------------
// Dead Reckoning 
void CallbackParkingStart(const nav_msgs::Path::ConstPtr& msg);    // JW add

void CallbackWheelSpeedDataCan(const std_msgs::Float32MultiArray::ConstPtr& msg);
void CallbackWheelSpeedData(const dead_reckoning::WheelSpeedMeasurements::ConstPtr& msg);
void CallbackESP_Data(const dead_reckoning::EspMeasurements::ConstPtr& msg);
void CallbackGearState(const dead_reckoning::GearState::ConstPtr& msg);
void CallbackReplanning(const std_msgs::Bool::ConstPtr& msg);

void UpdatePosition(double c_yaw_rate, double c_velo, double can_hw_timestamp);
void InitializeDR();
void PubDRpath();

// AVM 
void CallbackAVM(const sensor_msgs::ImageConstPtr &msg);
void skeletonize_parkingline(cv::Mat src, cv::Mat& dst);
void FeatureExtractor(cv::Mat input_img, cv::Mat& output_img_gray);
// ------------------------------------------------------------------------

// ------------------------ SUBSCRIBERS & PUBLISHERS -------------------------
// Dead Reckoning 
ros::Subscriber sub_gear, sub_wheel_speed, sub_esp, sub_parkingStart, sub_rePlanning;
ros::Publisher pub_localization_data, pub_canVelData, pub_dr_path;

// AVM 
ros::Subscriber sub_AVM;
ros::Publisher* pub_Segimg_Pointer;
ros::Publisher pub_segAVM;
// -------------------------------------------------------------------------

// ------------------------ FUNCTIONS -------------------------
// Dead Reckoning 
void InitializeDR() {
    // cout << "Initialize DeadReckoning!!!" << endl;
    m_car.x = 0.0;
    m_car.y = 0.0;
    m_car.th = 0.0;
    m_car.vel = 0.0;
    m_car.dir = 1.0;
}

void PubDRpath() {
    dr_path_msg.header.frame_id = "map";
    dr_path_msg.header.stamp = ros::Time::now();
    pose_dr.header = dr_path_msg.header;
    pose_dr.pose.position.x = m_car.x ;
    pose_dr.pose.position.y = m_car.y ;
    pose_dr.pose.position.z = 0;
    dr_path_msg.poses.push_back(pose_dr);

    pub_dr_path.publish(dr_path_msg);
}

//Based on kinematic bicycle model.
void UpdatePosition(double c_yaw_rate, double c_velo, double can_hw_timestamp) {
    m_car.dir = (m_forward) ? 1.0:-1.0;

    //Calculate dt
    double dt = can_hw_timestamp - m_prev_can_hw_timestamp;
    
    if (cnt >0)
        // Kinematic Bicycle model
        m_car.x += m_car.dir * c_velo * cos(m_car.th) * dt;
        m_car.y -= m_car.dir * c_velo * sin(m_car.th) * dt;
    
        // // New Kinematic Bicycle model ???
        // m_car.x += m_car.dir * m_c_velo * cos(m_car.th + 0.5*m_c_yaw_rate*dt) * dt;
        // m_car.y -= m_car.dir * m_c_velo * sin(m_car.th + 0.5*m_c_yaw_rate*dt) * dt;

        m_car.th += m_car.dir * c_yaw_rate * dt;
        m_car.vel = m_car.dir * c_velo;
        PubDRpath(); // YW add
    cnt += 1;
}

// AVM 
void skeletonize_parkingline(cv::Mat src, cv::Mat& dst){ //1 ch, 1 ch
    cv::Mat eroded;
    cv::Mat temp(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC1);
    cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));
    bool done;
    do
    {
        cv::erode(src, eroded, element);
        cv::dilate(eroded, temp, element);   // temp = open(img)
        cv::subtract(src, temp, temp);
        cv::bitwise_or(dst, temp, dst);
        eroded.copyTo(src);
        done = (cv::countNonZero(src) == 0);
    } while (!done);
}


void FeatureExtractor(cv::Mat input_img, cv::Mat& output_img_gray)
{   

    // make binary image
    for(int i = 0; i<input_img.size().width;i++){
        for(int j=0; j<input_img.size().height; j++){
            if((input_img.at<cv::Vec3b>(j, i)[2] >0) || (input_img.at<cv::Vec3b>(j, i)[1] > 0)  || (input_img.at<cv::Vec3b>(j, i)[0] > 0)){
                output_img_gray.at<uchar>(j, i) = 255;
            }
            else {
                output_img_gray.at<uchar>(j, i) = 0;
            }
        }
    }

    cv::Mat dst = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC1);
    cv::Mat dst1 = cv::Mat::zeros(m_SLAM_IMG_WIDTH, m_SLAM_IMG_WIDTH, CV_8UC1);
    // dst = output_img_gray.clone();
    skeletonize_parkingline(output_img_gray,dst);

    cv::Rect rect(m_AVM_IMG_WIDTH/2 - m_SLAM_IMG_WIDTH/2, m_AVM_IMG_WIDTH/2 - m_SLAM_IMG_WIDTH/2, m_SLAM_IMG_WIDTH, m_SLAM_IMG_WIDTH);
    dst1 = dst(rect);

    sensor_msgs::Image Segimg;
    Segimg.header.stamp = ros::Time::now();
    cv_bridge::CvImage img_bridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, dst1);
    // cv_bridge::CvImage img_bridge = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, img_blur);
    img_bridge.toImageMsg(Segimg);
    pub_Segimg_Pointer->publish(Segimg);
}
// -------------------------------------------------------------------------


// ------------------------ CALLBACK FUNCTIONS -------------------------
// Dead Reckoning
void CallbackReplanning(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        m_parkingStartFlag = false;
        // cout << "REPLANNING !!! [Dead Reckoning]" << endl;
    }
}

// JW add
void CallbackParkingStart(const nav_msgs::Path::ConstPtr& msg) {
    m_parkingStartFlag = true;
}

void CallbackGearState(const dead_reckoning::GearState::ConstPtr& msg) {
    m_forward = (msg->gear_state == 5) ? true : false;
}

void CallbackESP_Data(const dead_reckoning::EspMeasurements::ConstPtr& msg) {
    float lat_accel = msg->lat_accel; 
    float long_accel = msg->long_accel;
    float yaw_rate = msg->yaw_rate;
    bool yaw_rate_stat = msg->yaw_rate_stat;
    bool yaw_rate_diag = msg->yaw_rate_diag;
}

void CallbackWheelSpeedData(const dead_reckoning::WheelSpeedMeasurements::ConstPtr& msg) {
    float rear_left_wheel = msg->rear_left;
    float rear_right_wheel = msg->rear_right;
    float front_left_wheel = msg->front_left;
    float front_right_wheel = msg->front_right;
    double can_hw_timestamp = msg->hw_timestamp;
    
    // if (m_car.dir < 0) 
    //     yaw_gain = m_backward_yaw_gain;  // backward 

    double c_yaw_rate = yaw_gain*(rear_left_wheel - rear_right_wheel) * KpH2MpS / m_rear_wheel_distance; // [rad/s]
    double c_veloFront = 0.5 * (front_left_wheel + front_right_wheel) * KpH2MpS;
    double c_veloRear =  0.5 * (rear_left_wheel + rear_right_wheel) * KpH2MpS;
    double c_velo = 1.0*0.5*(c_veloFront + c_veloRear);
    // m_c_velo = 0.5 * (rear_left_wheel + rear_right_wheel) * KpH2MpS;
    // m_c_velo *= 1.03;

    if (m_parkingStartFlag) UpdatePosition(c_yaw_rate, c_velo, can_hw_timestamp);
    else                    InitializeDR();

    std_msgs::Float32MultiArray msgs;
    if (cnt >0)
        msgs.data.push_back(m_car.x);
        msgs.data.push_back(m_car.y);
        msgs.data.push_back(-m_car.th);
        msgs.data.push_back(m_car.vel*Mps2KpH); // [km/h]
        msgs.data.push_back(m_car.dir);     
        pub_localization_data.publish(msgs);
    
    m_prev_can_hw_timestamp = can_hw_timestamp;
}

// AVM
void CallbackAVM(const sensor_msgs::ImageConstPtr &msg)
{
    SrcImg = cv_bridge::toCvShare(msg, "bgr8")->image;
    Temp_SrcImg = SrcImg.clone();
    cv::Mat binaryMat= cv::Mat::zeros( m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC1);
    FeatureExtractor(Temp_SrcImg, binaryMat);
}
// -------------------------------------------------------------------------

int main(int argc, char *argv[]){
    ros::init(argc, argv, "preprocess_node");
    ros::NodeHandle node;
    ros::NodeHandle priv_nh("~");

    node.getParam("AVM_IMG_WIDTH",              m_AVM_IMG_WIDTH);
    node.getParam("AVM_IMG_HEIGHT",             m_AVM_IMG_HEIGHT);
    node.getParam("REAL_OCCUPANCY_SIZE_X",      m_REAL_OCCUPANCY_SIZE_X);
    node.getParam("REAL_OCCUPANCY_SIZE_Y",      m_REAL_OCCUPANCY_SIZE_Y);
    node.getParam("SLAM_OCCUPANCY_SIZE_X",      m_SLAM_OCCUPANCY_SIZE_X);
    node.getParam("SLAM_OCCUPANCY_SIZE_Y",      m_SLAM_OCCUPANCY_SIZE_Y);
    
    sub_AVM = node.subscribe("/AVM_ICP_image", 1, CallbackAVM);
    sub_wheel_speed = node.subscribe("/vehicle_state/wheel_speed_measurements", 1, &CallbackWheelSpeedData);
    // sub_wheel_speed = node.subscribe("/CAN/WHL_SPD", 10, &CallbackWheelSpeedDataCan);
    
    sub_esp = node.subscribe("/vehicle_state/esp_measurements", 1, &CallbackESP_Data);
    sub_gear = node.subscribe("/vehicle_state/gear_state", 1, &CallbackGearState);
    // sub_parkingStart = node.subscribe("/parkingPath", 1, &CallbackParkingStart);
    sub_rePlanning = node.subscribe("replanning", 1, &CallbackReplanning);
    // sub_rePlanning = node.subscribe("really_replanning", 1, &CallbackReplanning);

    // pub_localization_data = node.advertise<std_msgs::Float32MultiArray>("/LocalizationData", 1);
    pub_localization_data = node.advertise<std_msgs::Float32MultiArray>("/DRData", 1);
    // pub_canVelData = node.advertise<std_msgs::Float32MultiArray>("/CanVelData2", 1);    // JW add
    pub_dr_path = node.advertise<nav_msgs::Path>("DR_path", 1);
    pub_segAVM = node.advertise<sensor_msgs::Image>("/_cam/image_seg", 1);

    node.getParam("center_to_rear",         m_center_to_rear);
    node.getParam("rear_wheel_distance",    m_rear_wheel_distance);
    node.getParam("backward_yaw_gain",      m_backward_yaw_gain);
    InitializeDR();
    
    pub_Segimg_Pointer = &pub_segAVM;

    double PIXEL_PER_METER = (double)(m_AVM_IMG_WIDTH)/m_REAL_OCCUPANCY_SIZE_X;
    m_METER_PER_PIXEL = (double)(1.0/(double)(PIXEL_PER_METER));
    m_SLAM_IMG_WIDTH = (int)((double)(m_AVM_IMG_WIDTH * m_SLAM_OCCUPANCY_SIZE_X / m_REAL_OCCUPANCY_SIZE_X));


    ros::spin();
}
