//YW created in 2022.12.27
 
#include <iostream>
#include <string>
#include <exception>
#include <math.h>
#include <sstream>
   
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/types_c.h> 

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"

#include "opencv2/imgcodecs.hpp"
#include "opencv2/imgproc.hpp"

#include <std_msgs/Float32MultiArray.h>
#include <pcl_ros/point_cloud.h>
#include <algorithm>
#include <vector>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

// #define IMG_WIDTH 200
// #define IMG_HEIGHT 200

// #define RESIZE_IMG_WIDTH 200
// #define RESIZE_IMG_HEIGHT 200

// #define REAL_OCCUPANCY_SIZE_X (double)17.5  // AVM_IMG_WIDTH 400PIX == 25Meter // 35*13/18  
// #define REAL_OCCUPANCY_SIZE_Y (double)17.5 // AVM_IMG_HEIGHT 400PIX == 25Meter

// double METER_PER_PIXEL = 0.129999; //0.061634; // 0.060763 / 0.126388; // (13/18)*(35.0/IMG_WIDTH) 25.277408m, 25.639744m 0.12999999987 0.126388
// #define REAL_OCCUPANCY_SIZE_X (double) 8.5 * 4.555 / 5.2 // AVM_IMG_WIDTH 400PIX == 25Meter // 35*13/18  
// #define REAL_OCCUPANCY_SIZE_Y (double) 8.5 * 4.555 / 5.2 // AVM_IMG_HEIGHT 400PIX == 25Meter

// #define PIXEL_PER_METER  (double)IMG_WIDTH/REAL_OCCUPANCY_SIZE_X           //400PIX / 25M
// #define METER_PER_PIXEL  (double)(1.0/(double)(PIXEL_PER_METER)) // 0.0625  

int m_AVM_IMG_WIDTH = 0, m_AVM_IMG_HEIGHT = 0, m_SLAM_IMG_WIDTH = 0;
double m_REAL_OCCUPANCY_SIZE_X = 0.0, m_REAL_OCCUPANCY_SIZE_Y = 0.0;
double m_SLAM_OCCUPANCY_SIZE_X = 0.0, m_SLAM_OCCUPANCY_SIZE_Y = 0.0;
double m_METER_PER_PIXEL = 0.0;

struct CARPOSE {
    double x, y, th;
};
CARPOSE m_car;

pcl::PointCloud<pcl::PointXYZ>::Ptr avmpointcloud(new pcl::PointCloud<pcl::PointXYZ>());

void CallbackDRData(const std_msgs::Float32MultiArray::ConstPtr& msg);

ros::Publisher* pub_avmpointcloud_Pointer;

ros::Publisher pub_localization_data, pub_avmpointcloud, pubCurrentPose;

cv::Mat SrcImg= cv::Mat::zeros( m_SLAM_IMG_WIDTH, m_SLAM_IMG_WIDTH, CV_8UC1);
cv::Mat img= cv::Mat::zeros( m_SLAM_IMG_WIDTH, m_SLAM_IMG_WIDTH, CV_8UC1);

void CallbackDRData(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    m_car.x = msg->data[0];
    m_car.y = msg->data[1];
    m_car.th = msg->data[2];

    std_msgs::Float32MultiArray msgs;
    msgs.data.push_back(m_car.x);
    msgs.data.push_back(m_car.y);
    msgs.data.push_back(m_car.th);
    
    pub_localization_data.publish(msgs);
}
 
void AVM2PointCloud(cv::Mat input_img)
{   
    img = input_img.clone();

    int img_height = input_img.size().height;
    int img_width = input_img.size().width;

    pcl::PointXYZ point;

    // make AVM image to point cloud
    for(int j=0; j<img_height;j++){
        for(int i=0;i<img_width ;i++){
                if(int(img.at<uchar>(j,i))>0){
                    // point.x = -(j-(img_height/2+int(200 * 1.5248 / m_REAL_OCCUPANCY_SIZE_X)))*m_METER_PER_PIXEL;
                    point.x = -(j-img_height/2)*m_METER_PER_PIXEL;
                    point.y = -(i-img_width/2)*m_METER_PER_PIXEL;
                    point.z = 0.0;
                    

                    // std::cout<<  point.y << "//" << point.x << std::endl;

                    avmpointcloud->push_back(point);
                }
        }
    }

    // tf::TransformBroadcaster tfMap2AVM;
    // // tf::Transform mapToAVM = tf::Transform(tf::createQuaternionFromRPY(0.0,0.0,-m_car.th), tf::Vector3(-m_car.y,m_car.x,0.0));
    // tf::Transform mapToAVM = tf::Transform(tf::createQuaternionFromRPY(0.0,0.0,m_car.th), tf::Vector3(m_car.x,m_car.y,0.0));
    // tfMap2AVM.sendTransform(tf::StampedTransform(mapToAVM, ros::Time::now(), "map", "avm_link"));

    sensor_msgs::PointCloud2 avmpointcloud2;
    pcl::toROSMsg(*avmpointcloud, avmpointcloud2);
    avmpointcloud2.header.stamp = ros::Time::now();
    avmpointcloud2.header.frame_id = "map";
    pub_avmpointcloud_Pointer->publish(avmpointcloud2);

    avmpointcloud->points.clear();

    // // broacast pose of AVM (vehicle)
    // geometry_msgs::Quaternion AVMPoseQuat=tf::createQuaternionMsgFromRollPitchYaw(0.0,0.0,m_car.th);
    // nav_msgs::Odometry odomAftMapped;
    // odomAftMapped.header.frame_id = "map";
    // odomAftMapped.child_frame_id = "avm_link";
    // odomAftMapped.header.stamp = ros::Time::now();
    // odomAftMapped.pose.pose.orientation.x = AVMPoseQuat.x;
    // odomAftMapped.pose.pose.orientation.y = AVMPoseQuat.y;
    // odomAftMapped.pose.pose.orientation.z = AVMPoseQuat.z;
    // odomAftMapped.pose.pose.orientation.w = AVMPoseQuat.w;
    // // odomAftMapped.pose.pose.position.x = -m_car.y;
    // // odomAftMapped.pose.pose.position.y = m_car.x;
    // odomAftMapped.pose.pose.position.x = m_car.x;
    // odomAftMapped.pose.pose.position.y = m_car.y;
    // odomAftMapped.pose.pose.position.z = 0.0;
    // pubCurrentPose.publish(odomAftMapped);
}

void imageCallback(const sensor_msgs::ImageConstPtr &msg)
{
    SrcImg = cv_bridge::toCvShare(msg, "mono8")->image;
    AVM2PointCloud(SrcImg);
}

int main(int argc, char **argv)
{   
    ros::init(argc, argv, "avm2pointcloud_node");
    ros::NodeHandle nh;

    std::string OCAM_CALIB_FILE_PATH;
    OCAM_CALIB_FILE_PATH = "/home/joonwooahn/catkin_ws/src/undistorted_top_view/include";
    nh.getParam("OCAM_CALIB_FILE_PATH",       OCAM_CALIB_FILE_PATH);
    nh.getParam("AVM_IMG_WIDTH",              m_AVM_IMG_WIDTH);
    nh.getParam("AVM_IMG_HEIGHT",             m_AVM_IMG_HEIGHT);
    nh.getParam("REAL_OCCUPANCY_SIZE_X",      m_REAL_OCCUPANCY_SIZE_X);
    nh.getParam("REAL_OCCUPANCY_SIZE_Y",      m_REAL_OCCUPANCY_SIZE_Y);
    nh.getParam("SLAM_OCCUPANCY_SIZE_X",      m_SLAM_OCCUPANCY_SIZE_X);
    nh.getParam("SLAM_OCCUPANCY_SIZE_Y",      m_SLAM_OCCUPANCY_SIZE_Y);

    // ros::Subscriber sub_raw = nh.subscribe("/_cam/image_seg", 1, imageCallback); // for carla data
    ros::Subscriber sub_raw = nh.subscribe("/_cam/image_seg", 1, imageCallback); // for real bag data
    // ros::Subscriber sub_raw = nh.subscribe("/avm_usb_cam/image_seg", 1, imageCallback); // for carla data
    ros::Subscriber subDRData = nh.subscribe("/DRData", 100, CallbackDRData);

    pub_avmpointcloud = nh.advertise<sensor_msgs::PointCloud2>("/AVM_PointCloud", 100);
    pub_localization_data = nh.advertise<std_msgs::Float32MultiArray>("/DRData_aft_pointcloud", 100);
    // pubCurrentPose = nh.advertise<nav_msgs::Odometry>("/currentPose", 100);
    pub_avmpointcloud_Pointer = &pub_avmpointcloud;

    double PIXEL_PER_METER = (double)(m_AVM_IMG_WIDTH)/m_REAL_OCCUPANCY_SIZE_X;
    m_METER_PER_PIXEL = (double)(1.0/(double)(PIXEL_PER_METER));
    m_SLAM_IMG_WIDTH = (int)((double)(m_AVM_IMG_WIDTH * m_SLAM_OCCUPANCY_SIZE_X / m_REAL_OCCUPANCY_SIZE_X));

    ros::spin();

    return 0;
}