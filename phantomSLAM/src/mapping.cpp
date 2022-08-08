//YW created in 2022.12.27
#include <stdio.h>
#include <time.h>

#include <iostream>
#include <string>
#include <exception>
#define _USE_MATH_DEFINES
#include <math.h>
#include <sstream>
   
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>

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
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <fast_gicp/gicp/fast_gicp.hpp>

ros::Publisher pubCurrentFeature;
ros::Publisher pubCurrentFeatureInWorld;
ros::Publisher pubGlobalFeature;
ros::Publisher pubCurrentPose;

pcl::PointCloud<pcl::PointXYZ>::Ptr currentFeatureCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr currentFeatureCloudInWorld(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr globalFeatureCloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr moment_featurecloud(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr mapcloud(new pcl::PointCloud<pcl::PointXYZ>());


struct CARPOSE {
    double x, y, th;
};
CARPOSE m_car;

struct GOALPOSE {
    double x, y, th;
};
GOALPOSE m_goal;

int m_AVM_IMG_WIDTH = 0, m_AVM_IMG_HEIGHT = 0, m_SLAM_IMG_WIDTH = 0;
double m_REAL_OCCUPANCY_SIZE_X = 0.0, m_REAL_OCCUPANCY_SIZE_Y = 0.0;
double m_SLAM_OCCUPANCY_SIZE_X = 0.0, m_SLAM_OCCUPANCY_SIZE_Y = 0.0;
double m_METER_PER_PIXEL = 0.0;

double m_center_to_rear = 0.0;
double gicp_dist_thresh = 10;

int flag_int = 0;
int start_int = 0;

bool systemInitial=false;
bool gicp_flag = false;
bool gicp_intrp = false;

float currentX=0;
float currentY=0;
float currentZ=0;
float currentRoll=0;
float currentPitch=0;
float currentYaw=0;

float m_currentX=0;
float m_currentY=0;
float m_currentZ=0;
float m_currentRoll=0;
float m_currentPitch=0;
float m_currentYaw=0;

float prevX=0;
float prevY=0;
float prevZ=0;
float prevYaw = 0;

float delX=0;
float delY=0;
float delYaw = 0;

float cumulX=0;
float cumulY=0;
float cumulYaw=0;

float cumulodoX=0;
float cumulodoY=0;
float cumulodoYaw=0;

void CallbackAVM_Data(const sensor_msgs::PointCloud2ConstPtr &AVM_CloudMsg);
void Callbackmap_Data(const sensor_msgs::PointCloud2ConstPtr &map_CloudMsg);
void CallbackDRData(const std_msgs::Float32MultiArray::ConstPtr& msg);
void CallbackParkingGoal(const geometry_msgs::PoseArray::ConstPtr& end);

void InitializeDR();


Eigen::Affine3f transWorldCurrent;

//   transform point cloud according to pose
pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, Eigen::Affine3f& transCur)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZ>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
            cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
            cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);

        }
        return cloudOut;
    }

void CallbackDRData(const std_msgs::Float32MultiArray::ConstPtr& msg)
{
    m_car.x = msg->data[0];
    m_car.y = msg->data[1];
    m_car.th = msg->data[2];
}

void InitializeDR() {
    m_car.x = 0.0;
    m_car.y = 0.0;
    m_car.th = 0.0;

    m_goal.x = 1.3125;
    m_goal.y = -5.775;
    m_goal.th = -0.5 * M_PI;
}


void Callbackmap_Data(const sensor_msgs::PointCloud2ConstPtr &map_CloudMsg)
{
    pcl::PointCloud<pcl::PointXYZ> map_CloudIn;
    pcl::fromROSMsg(*map_CloudMsg, map_CloudIn);
    mapcloud->clear();
    for(size_t i=0;i<map_CloudIn.points.size();i++){
         pcl::PointXYZ pi=map_CloudIn.points[i];

        mapcloud->push_back(pi);
    }
}

void CallbackParkingGoal(const geometry_msgs::PoseArray::ConstPtr& end) {    //[end] which is the coordinates of the goal
    m_goal.x = end->poses[0].position.x;
    m_goal.y = end->poses[0].position.y;
    m_goal.th = tf::getYaw(end->poses[0].orientation);
}

void CallbackAVM_Data(const sensor_msgs::PointCloud2ConstPtr &AVM_CloudMsg)
{
    pcl::PointCloud<pcl::PointXYZ> AVM_CloudIn;
    pcl::fromROSMsg(*AVM_CloudMsg, AVM_CloudIn);
    if (AVM_CloudIn.points.size() > 0){
        
        currentFeatureCloud->clear();
        for(size_t i=0;i<AVM_CloudIn.points.size();i++){
            pcl::PointXYZ pi=AVM_CloudIn.points[i];

            currentFeatureCloud->push_back(pi);
        }
        

        // for slam real data 
        currentYaw = m_car.th;
        currentX = m_car.x;
        currentY = m_car.y;

        if(!systemInitial){

            // filtered
            pcl::PointCloud<pcl::PointXYZ>::Ptr init_cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> init_sor;
            init_sor.setInputCloud (currentFeatureCloud);            
            init_sor.setMeanK (20);               
            init_sor.setStddevMulThresh (1);         
            init_sor.filter (*init_cloud_filtered);
            *currentFeatureCloud = *init_cloud_filtered;

            // Eigen::Matrix4f guess = Eigen::Matrix4f::Identity(); 
            // guess(0,0) = cos(-cumulodoYaw);
            // guess(0,1) = sin(-cumulodoYaw);
            // guess(1,0) = -sin(-cumulodoYaw);
            // guess(1,1) = cos(-cumulodoYaw);
            // guess(0,3) = cumulodoX;
            // guess(1,3) = cumulodoY;

            // int cloudSize = currentFeatureCloud->size();

            // pcl::PointCloud<pcl::PointXYZ>::Ptr init_moment(new pcl::PointCloud<pcl::PointXYZ>());
            // init_moment->resize(cloudSize);
            // for (int i = 0; i < cloudSize; ++i){
            //     const auto &pointFrom = currentFeatureCloud->points[i];
            //     init_moment->points[i].x = guess(0,0) * pointFrom.x + guess(0,1) * pointFrom.y + guess(0,2) * pointFrom.z + guess(0,3);
            //     init_moment->points[i].y = guess(1,0) * pointFrom.x + guess(1,1) * pointFrom.y + guess(1,2) * pointFrom.z + guess(1,3);
            //     init_moment->points[i].z = guess(2,0) * pointFrom.x + guess(2,1) * pointFrom.y + guess(2,2) * pointFrom.z + guess(2,3);
            // }

            // *currentFeatureCloud = *init_moment;


            // *currentFeatureCloud = *transformPointCloud(currentFeatureCloud, guess);

            // //  global map initialize
            // pcl::PointCloud<pcl::PointXYZ> init_cloud_voxeled;
            // pcl::VoxelGrid<pcl::PointXYZ> init_downSizeFilter;
            // init_downSizeFilter.setInputCloud(currentFeatureCloud);
            // init_downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            // init_downSizeFilter.filter(init_cloud_voxeled);
            // *currentFeatureCloud=init_cloud_voxeled;

            *globalFeatureCloud=*globalFeatureCloud+*currentFeatureCloud;


            if (m_AVM_IMG_WIDTH == 200 ){
                pcl::PointCloud<pcl::PointXYZ> init_global_cloud_voxeled;
                pcl::VoxelGrid<pcl::PointXYZ> init_global_downSizeFilter;
                init_global_downSizeFilter.setInputCloud(globalFeatureCloud);
                init_global_downSizeFilter.setLeafSize(0.0075, 0.0075, 0.0075); // 200 by 200
                init_global_downSizeFilter.filter(init_global_cloud_voxeled);
                *globalFeatureCloud=init_global_cloud_voxeled;
            }

            *currentFeatureCloudInWorld = *currentFeatureCloud;
            *moment_featurecloud = *currentFeatureCloud;
            *mapcloud = *currentFeatureCloud;

            m_currentYaw = currentYaw;
            m_currentX = currentX;
            m_currentY = currentY;

            cumulodoX = m_currentX;
            cumulodoY = m_currentY;
            cumulodoYaw = m_currentYaw;
            if (start_int > 10){
                systemInitial=true;
            }
            else {
                start_int += 1;
            }
            // std::cout << systemInitial << std::endl;

            return ;
        }

        tf::TransformBroadcaster tfMap2AVM;
        tf::Transform mapToAVM = tf::Transform(tf::createQuaternionFromRPY(currentRoll,currentPitch,currentYaw), tf::Vector3(currentX,currentY,currentZ));
        tfMap2AVM.sendTransform(tf::StampedTransform(mapToAVM, ros::Time::now(), "map", "avm_link"));

        delX = currentX - prevX;
        delY = currentY - prevY;
        delYaw = currentYaw - prevYaw;

        cumulX += delX;
        cumulY += delY;
        cumulYaw += delYaw;

        cumulodoX += delX;
        cumulodoY += delY;
        cumulodoYaw += delYaw;


        double dis=sqrt(cumulX*cumulX+cumulY*cumulY);

        if(dis>0.0075){
            *moment_featurecloud = *currentFeatureCloud;

            // filtered
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud (moment_featurecloud);            
            sor.setMeanK (20);               
            sor.setStddevMulThresh (1);         
            sor.filter (*cloud_filtered);
            *moment_featurecloud = *cloud_filtered;

            // // voxeled
            // pcl::PointCloud<pcl::PointXYZ> cloud_voxeled;
            // pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
            // downSizeFilter.setInputCloud(moment_featurecloud);
            // downSizeFilter.setLeafSize(0.2, 0.2, 0.2);
            // downSizeFilter.filter(cloud_voxeled);
            // *moment_featurecloud=cloud_voxeled;

            if (gicp_flag) {
                // std::cout << "registration: FAST_GICP" << std::endl;
                static fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>::Ptr gicp(new fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>());
                gicp->setNumThreads(8);
                gicp->setTransformationEpsilon(1e-4);
                gicp->setMaximumIterations(300);

                // Real data 8.5-----------------------------------------------------------------------------------
                // gicp->setMaxCorrespondenceDistance(0.12);
                // gicp->setCorrespondenceRandomness(40);

                // Real data 17.5-----------------------------------------------------------------------------------
                // gicp->setMaxCorrespondenceDistance(0.001);
                
                // 200 by 200
                gicp->setMaxCorrespondenceDistance(0.16* 200 / m_AVM_IMG_WIDTH);
                // gicp->setCorrespondenceRandomness(40);

                

                // Align pointclouds
                gicp->setInputSource(moment_featurecloud);
                // gicp->setInputTarget(globalFeatureCloud);

                // gicp->setInputTarget(currentFeatureCloudInWorld);
                if (mapcloud->size()) {
                    gicp->setInputTarget(mapcloud);
                }
                else {
                    gicp->setInputTarget(currentFeatureCloudInWorld);
                }
                pcl::PointCloud<pcl::PointXYZ>::Ptr transCurrentCloudInWorld(new pcl::PointCloud<pcl::PointXYZ>());
                Eigen::Matrix4f guess = Eigen::Matrix4f::Identity(); 
                // if (gicp_intrp && flag_int == 1){
                //     guess(0,0) = cos(m_goal.th);
                //     guess(0,1) = sin(m_goal.th);
                //     guess(1,0) = -sin(m_goal.th);
                //     guess(1,1) = cos(m_goal.th);
                //     flag_int = 2;
                // }
                // else?{
                guess(0,0) = cos(-cumulodoYaw);
                guess(0,1) = sin(-cumulodoYaw);
                guess(1,0) = -sin(-cumulodoYaw);
                guess(1,1) = cos(-cumulodoYaw);
                // }
                guess(0,3) = cumulodoX;
                guess(1,3) = cumulodoY;

                gicp->align(*transCurrentCloudInWorld, guess);
                gicp->swapSourceAndTarget();
                // transWorldCurrent = gicp->getFinalTransformation();

                float loopFitnessScoreThreshold = 6; // user parameter but fixed low value is safe. 
                if (gicp->hasConverged() == false || gicp->getFitnessScore() > loopFitnessScoreThreshold) {
                    // std::cout << "GICP fitness test failed (" << gicp->getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
                    transWorldCurrent = guess;
                } else {
                    // std::cout << "GICP fitness test passed (" << gicp->getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
                    transWorldCurrent = gicp->getFinalTransformation();
                    // std::cout << "but optimization is not occured for testing performance" << std::endl;
                    // return std::nullopt;
                }
                *currentFeatureCloudInWorld =*transformPointCloud(moment_featurecloud,transWorldCurrent);
                *globalFeatureCloud = *globalFeatureCloud+*currentFeatureCloudInWorld; 

                // filtered
                // pcl::PointCloud<pcl::PointXYZ>::Ptr globalcloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
                // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> glob_sor;
                // glob_sor.setInputCloud (globalFeatureCloud);            
                // glob_sor.setMeanK (20);               
                // glob_sor.setStddevMulThresh (1.5);         
                // glob_sor.filter (*globalcloud_filtered);
                // *globalFeatureCloud = *globalcloud_filtered;
                if (m_AVM_IMG_WIDTH == 200 ){
                    pcl::PointCloud<pcl::PointXYZ> global_cloud_voxeled;
                    pcl::VoxelGrid<pcl::PointXYZ> global_downSizeFilter;
                    global_downSizeFilter.setInputCloud(globalFeatureCloud);
                    global_downSizeFilter.setLeafSize(0.0075, 0.0075, 0.0075); // 200 by 200
                    global_downSizeFilter.filter(global_cloud_voxeled);
                    *globalFeatureCloud=global_cloud_voxeled;
                }


                pcl::getTranslationAndEulerAngles(transWorldCurrent,m_currentX,m_currentY,m_currentZ,m_currentRoll,m_currentPitch,m_currentYaw);
                // ##############################################################################################
                
                // pcl::PointCloud<pcl::PointXYZ> globalMapDS;
                // pcl::VoxelGrid<pcl::PointXYZ> downSizeFilter;
                // downSizeFilter.setInputCloud(globalFeatureCloud);
                // downSizeFilter.setLeafSize(0.5, 0.5, 0.5);
                // downSizeFilter.filter(globalMapDS);
                // *globalFeatureCloud=globalMapDS;
            }
            else {
                // when not using GICP while building graph
                m_currentX = currentX;
                m_currentY = currentY;
                m_currentYaw = currentYaw;
            }
            cumulodoX = m_currentX;
            cumulodoY = m_currentY;
            cumulodoYaw = m_currentYaw;

            if ((cumulodoX - m_goal.x) * (cumulodoX - m_goal.x) + (cumulodoY - m_goal.y) * (cumulodoY - m_goal.y) < gicp_dist_thresh * gicp_dist_thresh) {
                // std::cout << (m_goal.x) << "//"  << (m_goal.y) <<  std::endl;
                // std::cout << (cumulodoX - m_goal.x) * (cumulodoX - m_goal.x) + (cumulodoY - m_goal.y) * (cumulodoY - m_goal.y) <<  std::endl;
                gicp_flag = true;
                if (flag_int == 0){
                    std::cout << "GICP START! " <<  std::endl;
                    flag_int = 1;
                }
            }

            cumulX = 0.0;
            cumulY = 0.0;

        }
        // std::cout << m_currentX << "/" << m_currentY << "/" << m_currentYaw << std::endl;
        prevX=currentX;
        prevY=currentY;
        prevYaw=currentYaw;

    }
    //  broadcast prior  global map information 
    sensor_msgs::PointCloud2 AVMCloudGlobalMapMsg;
    pcl::toROSMsg(*globalFeatureCloud, AVMCloudGlobalMapMsg);
    AVMCloudGlobalMapMsg.header.stamp = ros::Time::now();
    AVMCloudGlobalMapMsg.header.frame_id = "map";
    pubGlobalFeature.publish(AVMCloudGlobalMapMsg);

    //   transform and broadcast point cloud from current  coordinate to world coordinate 
    sensor_msgs::PointCloud2 AVMCloudCurrentInWorldMsg;
    pcl::toROSMsg(*currentFeatureCloudInWorld, AVMCloudCurrentInWorldMsg);
    AVMCloudCurrentInWorldMsg.header.stamp = ros::Time::now();
    AVMCloudCurrentInWorldMsg.header.frame_id = "map";
    pubCurrentFeatureInWorld.publish(AVMCloudCurrentInWorldMsg);

    //  broadcast  feature point cloud of current frame
    sensor_msgs::PointCloud2 cameraCloudFrameMsg;
    pcl::toROSMsg(*moment_featurecloud, cameraCloudFrameMsg);
    cameraCloudFrameMsg.header.stamp = ros::Time::now();
    cameraCloudFrameMsg.header.frame_id = "avm_link";
    pubCurrentFeature.publish(cameraCloudFrameMsg);

    // broacast pose of AVM (vehicle)
    geometry_msgs::Quaternion AVMPoseQuat=tf::createQuaternionMsgFromRollPitchYaw(m_currentRoll,m_currentPitch,m_currentYaw);
    nav_msgs::Odometry odomAftMapped;
    odomAftMapped.header.frame_id = "map";
    odomAftMapped.child_frame_id = "avm_link";
    odomAftMapped.header.stamp = ros::Time::now();
    odomAftMapped.pose.pose.orientation.x = AVMPoseQuat.x;
    odomAftMapped.pose.pose.orientation.y = AVMPoseQuat.y;
    odomAftMapped.pose.pose.orientation.z = AVMPoseQuat.z;
    odomAftMapped.pose.pose.orientation.w = AVMPoseQuat.w;
    odomAftMapped.pose.pose.position.x = m_currentX;
    odomAftMapped.pose.pose.position.y = m_currentY;
    odomAftMapped.pose.pose.position.z = m_currentZ;
    pubCurrentPose.publish(odomAftMapped);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "mapping_node");
    ros::NodeHandle nh;

    nh.getParam("AVM_IMG_WIDTH",              m_AVM_IMG_WIDTH);
    nh.getParam("AVM_IMG_HEIGHT",             m_AVM_IMG_HEIGHT);
    nh.getParam("REAL_OCCUPANCY_SIZE_X",      m_REAL_OCCUPANCY_SIZE_X);
    nh.getParam("REAL_OCCUPANCY_SIZE_Y",      m_REAL_OCCUPANCY_SIZE_Y);
    nh.getParam("SLAM_OCCUPANCY_SIZE_X",      m_SLAM_OCCUPANCY_SIZE_X);
    nh.getParam("SLAM_OCCUPANCY_SIZE_Y",      m_SLAM_OCCUPANCY_SIZE_Y);
 
    ros::Subscriber subAVMCloud = nh.subscribe<sensor_msgs::PointCloud2>("/AVM_PointCloud", 1, CallbackAVM_Data);
    ros::Subscriber submapCloud = nh.subscribe<sensor_msgs::PointCloud2>("/aft_pgo_map", 1, Callbackmap_Data);
    ros::Subscriber subDRData = nh.subscribe("/DRData_aft_pointcloud", 1, CallbackDRData);
    // ros::Subscriber Sub_parkingGoal = nh.subscribe("parking_cands", 1, CallbackParkingGoal); // get from avm

    pubCurrentFeature = nh.advertise<sensor_msgs::PointCloud2>("/currentFeature", 1);
    pubCurrentFeatureInWorld= nh.advertise<sensor_msgs::PointCloud2>("/currentFeatureInWorld", 1);
    pubGlobalFeature= nh.advertise<sensor_msgs::PointCloud2>("/globalFeatureMap", 1);
    pubCurrentPose = nh.advertise<nav_msgs::Odometry>("/currentPose", 1);

    InitializeDR();
 
    ros::spin();
}