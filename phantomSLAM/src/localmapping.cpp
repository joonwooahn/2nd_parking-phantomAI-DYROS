#include <fstream>
#include <math.h>
#include <vector>
#include <mutex>
#include <queue>
#include <thread>
#include <iostream>
#include <string>
#include <optional>
#include <ctime>
#include <cstdlib>
  
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/gicp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/octree/octree_pointcloud_voxelcentroid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float32MultiArray.h>

#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Rot2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>

#include "../include/common.h"
#include "../include/tic_toc.h"

#include <fast_gicp/gicp/fast_gicp.hpp>

using namespace gtsam;

using std::cout;
using std::endl;

double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double frameMeterGap;
double frameDegGap, frameRadGap;

double keytranslationAccumulated = 1000000.0;// large value means must add the first given frame.
double keyrotaionAccumulated = 1000000.0; // large value means must add the first given frame.

double translationAccumulated = 1000000.0; // large value means must add the first given frame.
double rotaionAccumulated = 1000000.0; // large value means must add the first given frame.

bool isNowKeyFrame = false; 
bool isNowFrame = false; 

Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init 
Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero 

std::queue<std_msgs::Float32MultiArray::ConstPtr> carposeBuf;

std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf_gt;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<std::pair<int, int> > scLoopICPBuf;

std::mutex mBuf;
std::mutex mKF;

double timeLaserOdometry = 0.0;
double timeLaserOdometry_gt = 0.0;
double timeLaser = 0.0;

int cnt = 0;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

pcl::PointCloud<pcl::PointXYZ>::Ptr LoopCurCloudInWorld(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr LoopTarCloudInWorld(new pcl::PointCloud<pcl::PointXYZ>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds; 
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;
std::vector<Pose6D> keyframe_GT_Poses;
std::vector<double> keyframeTimes;
std::vector<double> keyframe_GT_Times;
int recentIdxUpdated = 0;
int prev_keyframe_node_idx = 0;

std::vector<int> keynodes;

gtsam::NonlinearFactorGraph gtSAMgraph;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustGPSNoise;

noiseModel::Base::shared_ptr robustOdomNoise;

pcl::VoxelGrid<PointType> downSizeFilterScancontext;

// pcl::VoxelGrid<PointType> downSizeFilterICP;
std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;

// pcl::PointCloud<PointType>::Ptr laserCloudMapPGO(new pcl::PointCloud<PointType>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudMapPGO(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr recentlaserCloudMapPGO(new pcl::PointCloud<pcl::PointXYZRGB>());
// pcl::VoxelGrid<PointType> downSizeFilterMapPGO;
bool laserCloudMapPGORedraw = true;

double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;

ros::Publisher pubMapAftPGO, pubOdomAftPGO, pubPathAftPGO, pubPathGT, pubOdomAftPGO_Array, recentpubMapAftPGO;
ros::Publisher pubLoopScanLocal, pubLoopSubmapLocal, pubLoopScan_aft_Local;
ros::Publisher pubOdomRepubVerifier;

std::string save_directory;
std::string pgKITTIformat, pgScansDirectory, pgSCDsDirectory, savepose_name;
std::string odomKITTIformat;
std::fstream pgG2oSaveStream, pgTimeSaveStream;

// std::vector<std::string> edges_str; // used in writeEdge

std::string padZeros(int val, int num_digits = 6) 
{
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3

void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &_laserOdometry)
{
	mBuf.lock();
	odometryBuf.push(_laserOdometry);
	mBuf.unlock();
} // laserOdometryHandler

// void CallbackDRData(const std_msgs::Float32MultiArray::ConstPtr &_msg)
// {
//     mBuf.lock();
//     carposeBuf.push(_msg);
//     mBuf.unlock();
// }// CallbackDRData


void laserCloudFullResHandler(const sensor_msgs::PointCloud2ConstPtr &_laserCloudFullRes)
{
	mBuf.lock();
	fullResBuf.push(_laserCloudFullRes);
	mBuf.unlock();
} // laserCloudFullResHandler


void initNoises( void )
{
    gtsam::Vector priorNoiseVector6(6);
    priorNoiseVector6 << 1e-12, 1e-12, 1e-12, 1e-12, 1e-12, 1e-12;
    priorNoise = noiseModel::Diagonal::Variances(priorNoiseVector6);

    gtsam::Vector odomNoiseVector6(6);
    // odomNoiseVector6 << 1e-4, 1e-4, 1e-4, 1e-4, 1e-4, 1e-4;
    odomNoiseVector6 << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4;
    odomNoise = noiseModel::Diagonal::Variances(odomNoiseVector6);
    robustOdomNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(odomNoiseVector6) );

    double loopNoiseScore = 0.5; // constant is ok...
    gtsam::Vector robustNoiseVector6(6); // gtsam::Pose3 factor has 6 elements (6D)
    robustNoiseVector6 << loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore, loopNoiseScore;
    robustLoopNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector6) );

    double bigNoiseTolerentToXY = 1000000000.0; // 1e9
    double gpsAltitudeNoiseScore = 250.0; // if height is misaligned after loop clsosing, use this value bigger
    gtsam::Vector robustNoiseVector3(3); // gps factor has 3 elements (xyz)
    robustNoiseVector3 << bigNoiseTolerentToXY, bigNoiseTolerentToXY, gpsAltitudeNoiseScore; // means only caring altitude here. (because LOAM-like-methods tends to be asymptotically flyging)
    robustGPSNoise = gtsam::noiseModel::Robust::Create(
                    gtsam::noiseModel::mEstimator::Cauchy::Create(1), // optional: replacing Cauchy by DCS or GemanMcClure is okay but Cauchy is empirically good.
                    gtsam::noiseModel::Diagonal::Variances(robustNoiseVector3) );

} // initNoises

Pose6D carOdom(std_msgs::Float32MultiArray::ConstPtr _carpose)
{
    auto tx = _carpose->data[0];
    auto ty = _carpose->data[1];
    double tz = 0.0;
    auto yaw = _carpose->data[2];
    double roll = 0.0;
    double pitch = 0.0;
    // cout << tx << " " << ty << endl;
    // for slam only && CARLA
    return Pose6D{-ty, tx, tz, roll * 180/M_PI, pitch * 180/M_PI, yaw * 180/M_PI}; 
} // carOdom

Pose6D getOdom(nav_msgs::Odometry::ConstPtr _odom)
{
    auto tx = _odom->pose.pose.position.x;
    auto ty = _odom->pose.pose.position.y;
    auto tz = _odom->pose.pose.position.z;

    double roll, pitch, yaw;
    geometry_msgs::Quaternion quat = _odom->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(quat.x, quat.y, quat.z, quat.w)).getRPY(roll, pitch, yaw);

    return Pose6D{tx, ty, tz, roll, pitch, yaw}; 
} // getOdom

Pose6D getOdom_gt(nav_msgs::Odometry::ConstPtr _odom_gt)
{   
    auto tx_gt = _odom_gt->pose.pose.position.x;
    auto ty_gt = _odom_gt->pose.pose.position.y;
    auto tz_gt = _odom_gt->pose.pose.position.z;

    double roll_gt, pitch_gt, yaw_gt;
    geometry_msgs::Quaternion quat_gt = _odom_gt->pose.pose.orientation;
    tf::Matrix3x3(tf::Quaternion(quat_gt.x, quat_gt.y, quat_gt.z, quat_gt.w)).getRPY(roll_gt, pitch_gt, yaw_gt);

    return Pose6D{tx_gt, ty_gt, tz_gt, roll_gt, pitch_gt, yaw_gt}; 
} // getOdom_gt

Pose6D diffTransformation(const Pose6D& _p1, const Pose6D& _p2)
{
    Eigen::Affine3f SE3_p1 = pcl::getTransformation(_p1.x, _p1.y, _p1.z, _p1.roll, _p1.pitch, _p1.yaw);
    Eigen::Affine3f SE3_p2 = pcl::getTransformation(_p2.x, _p2.y, _p2.z, _p2.roll, _p2.pitch, _p2.yaw);
    Eigen::Matrix4f SE3_delta0 = SE3_p1.matrix().inverse() * SE3_p2.matrix();
    Eigen::Affine3f SE3_delta; SE3_delta.matrix() = SE3_delta0;
    float dx, dy, dz, droll, dpitch, dyaw;
    pcl::getTranslationAndEulerAngles (SE3_delta, dx, dy, dz, droll, dpitch, dyaw);
    // std::cout << "delta : " << dx << ", " << dy << ", " << dz << ", " << droll << ", " << dpitch << ", " << dyaw << std::endl;

    return Pose6D{double(abs(dx)), double(abs(dy)), double(abs(dz)), double(abs(droll)), double(abs(dpitch)), double(abs(dyaw))};
} // SE3Diff

pcl::PointCloud<PointType>::Ptr local2global(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
    
    int numberOfCores = 16;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
    }

    return cloudOut;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr local2global_map(const pcl::PointCloud<PointType>::Ptr &cloudIn, const Pose6D& tf, int idx, int fin)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudOut(new pcl::PointCloud<pcl::PointXYZRGB>());

    // srand((unsigned int)time(NULL));
    int pointr = 255;
    int pointg = 255;
    int pointb = 255;

    int residual_color = idx % 6;

    if (residual_color == 0) {
        pointr = 255;
        pointg = 51;
        pointb = 255;
    }
    else if (residual_color == 1) {
        pointr = 102;
        pointg = 178;
        pointb = 255;
    }
    else if (residual_color == 2) {
        pointr = 255;
        pointg = 255;
        pointb = 51;
    }
    else if (residual_color == 3) {
        pointr = 0;
        pointg = 255;
        pointb = 0;
    }
    else if (residual_color == 4) {
        pointr = 255;
        pointg = 153;
        pointb = 51;
    }

    // std::cout << pointr << " / " << pointg << " / " << pointb << std::endl;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(tf.x, tf.y, tf.z, tf.roll, tf.pitch, tf.yaw);
    
    int numberOfCores = 16;
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        const auto &pointFrom = cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom.x + transCur(0,1) * pointFrom.y + transCur(0,2) * pointFrom.z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom.x + transCur(1,1) * pointFrom.y + transCur(1,2) * pointFrom.z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom.x + transCur(2,1) * pointFrom.y + transCur(2,2) * pointFrom.z + transCur(2,3);
        cloudOut->points[i].r = pointr;
        cloudOut->points[i].g = pointg;
        cloudOut->points[i].b = pointb;
    }

    return cloudOut;
}

void pubPath( void )
{
    // pub odom and path 
    nav_msgs::Odometry odomAftPGO;
    nav_msgs::Odometry odomAftPGO_gt;
    nav_msgs::Path pathAftPGO;
    // nav_msgs::Path pathGT;
    pathAftPGO.header.frame_id = "map";
    // pathGT.header.frame_id = "map";

    mKF.lock(); 
    double DR_MAT[2][recentIdxUpdated];
    double GT_MAT[2][recentIdxUpdated];
    double ATE = 0.0;
    double m_ATE = 0.0;

    double current_ATE = 0.0;
    double max_ATE = 0.0;
    double mean_ATE = 0.0;
    double m_mean_ATE = 0.0;
    double m_current_ATE = 0.0;

    double curr_v = 0.0;


    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()) - 1; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    {
        const Pose6D& pose_est = keyframePosesUpdated.at(node_idx); // upodated poses

        nav_msgs::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = "map";
        odomAftPGOthis.child_frame_id = "aft_pgo";
        odomAftPGOthis.header.stamp = ros::Time().fromSec(keyframeTimes.at(node_idx));
        // dr_timestamp = keyframeTimes.at(node_idx);
        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;

        DR_MAT[0][node_idx] = pose_est.x;
        DR_MAT[1][node_idx] = pose_est.y;

        odomAftPGOthis.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_est.roll, pose_est.pitch, pose_est.yaw);
        odomAftPGO = odomAftPGOthis;

        Eigen::Quaternionf qf;
        qf.x() = odomAftPGO.pose.pose.orientation.x;
        qf.y() = odomAftPGO.pose.pose.orientation.y;
        qf.z() = odomAftPGO.pose.pose.orientation.z;
        qf.w() = odomAftPGO.pose.pose.orientation.w; 

        auto qf_euler = qf.toRotationMatrix().eulerAngles(0, 1, 2);
        float qf_yaw = qf_euler[2];

        geometry_msgs::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;;
        poseStampAftPGO.pose.position.x = odomAftPGOthis.pose.pose.position.x + 1.5248 * cos(qf_yaw);
        poseStampAftPGO.pose.position.y = odomAftPGOthis.pose.pose.position.y + 1.5248 * sin(qf_yaw);

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = "map";
        pathAftPGO.poses.push_back(poseStampAftPGO);

    }

    // Eigen::Quaternionf qf;
    // qf.x() = odomAftPGO.pose.pose.orientation.x;
    // qf.y() = odomAftPGO.pose.pose.orientation.y;
    // qf.z() = odomAftPGO.pose.pose.orientation.z;
    // qf.w() = odomAftPGO.pose.pose.orientation.w; 

    // auto qf_euler = qf.toRotationMatrix().eulerAngles(0, 1, 2);
    // float qf_yaw = qf_euler[2];

    // std_msgs::Float32MultiArray odomAftPGO_array;

    // vector<double> vec1 = {-odomAftPGO.pose.pose.position.x, odomAftPGO.pose.pose.position.y, -qf_yaw, 0.0};
    // // set up dimensions
    // odomAftPGO_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
    // odomAftPGO_array.layout.dim[0].size = vec1.size();
    // odomAftPGO_array.layout.dim[0].stride = 1;
    // odomAftPGO_array.layout.dim[0].label = "pose"; // or whatever name you typically use to index vec1

    // // copy in the data
    // odomAftPGO_array.data.clear();
    // odomAftPGO_array.data.insert(odomAftPGO_array.data.end(), vec1.begin(), vec1.end());

    mKF.unlock(); 
    pubOdomAftPGO.publish(odomAftPGO); // last pose 
    pubPathAftPGO.publish(pathAftPGO); // poses 

    static tf::TransformBroadcaster br;
    tf::Transform transform;
    tf::Quaternion q;
    transform.setOrigin(tf::Vector3(odomAftPGO.pose.pose.position.x, odomAftPGO.pose.pose.position.y, odomAftPGO.pose.pose.position.z));
    q.setW(odomAftPGO.pose.pose.orientation.w);
    q.setX(odomAftPGO.pose.pose.orientation.x);
    q.setY(odomAftPGO.pose.pose.orientation.y);
    q.setZ(odomAftPGO.pose.pose.orientation.z);
    transform.setRotation(q);
    br.sendTransform(tf::StampedTransform(transform, odomAftPGO.header.stamp, "map", "aft_pgo"));
} // pubPath

void updatePoses(void)
{
    mKF.lock(); 
    for (int node_idx=0; node_idx < int(isamCurrentEstimate.size()); node_idx++)
    {
        Pose6D& p =keyframePosesUpdated[node_idx];
        p.x = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().x();
        p.y = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().y();
        p.z = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).translation().z();
        p.roll = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().roll();
        p.pitch = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().pitch();
        p.yaw = isamCurrentEstimate.at<gtsam::Pose3>(node_idx).rotation().yaw();
    }
    mKF.unlock();

    mtxRecentPose.lock();
    const gtsam::Pose3& lastOptimizedPose = isamCurrentEstimate.at<gtsam::Pose3>(int(isamCurrentEstimate.size())-1);
    recentOptimizedX = lastOptimizedPose.translation().x();
    recentOptimizedY = lastOptimizedPose.translation().y();

    recentIdxUpdated = int(keyframePosesUpdated.size()) - 1;

    mtxRecentPose.unlock();
} // updatePoses

void runISAM2opt(void)
{
    // called when a variable added 
    isam->update(gtSAMgraph, initialEstimate);
    isam->update();
    
    gtSAMgraph.resize(0);
    initialEstimate.clear();

    isamCurrentEstimate = isam->calculateEstimate();
    updatePoses();
    // gtSAMgraph.print("GTSAM Graph:\n");
}

//   transform point cloud according to pose
pcl::PointCloud<pcl::PointXYZ>::Ptr transformPointCloud_mp(pcl::PointCloud<pcl::PointXYZ>::Ptr cloudIn, Eigen::Matrix4f& transCur)
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

pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, gtsam::Pose3 transformIn)
{
    pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

    PointType *pointFrom;

    int cloudSize = cloudIn->size();
    cloudOut->resize(cloudSize);

    Eigen::Affine3f transCur = pcl::getTransformation(
                                    transformIn.translation().x(), transformIn.translation().y(), transformIn.translation().z(), 
                                    transformIn.rotation().roll(), transformIn.rotation().pitch(), transformIn.rotation().yaw() );
    
    int numberOfCores = 8; // TODO move to yaml 
    #pragma omp parallel for num_threads(numberOfCores)
    for (int i = 0; i < cloudSize; ++i)
    {
        pointFrom = &cloudIn->points[i];
        cloudOut->points[i].x = transCur(0,0) * pointFrom->x + transCur(0,1) * pointFrom->y + transCur(0,2) * pointFrom->z + transCur(0,3);
        cloudOut->points[i].y = transCur(1,0) * pointFrom->x + transCur(1,1) * pointFrom->y + transCur(1,2) * pointFrom->z + transCur(1,3);
        cloudOut->points[i].z = transCur(2,0) * pointFrom->x + transCur(2,1) * pointFrom->y + transCur(2,2) * pointFrom->z + transCur(2,3);
    }
    return cloudOut;
} // transformPointCloud

void loopFindNearKeyframesCloud( pcl::PointCloud<PointType>::Ptr& nearKeyframes, const int& key, const int& submap_size, const int& root_idx)
{
    // extract and stacking near keyframes (in global coord)
    nearKeyframes->clear();
    // for (int i = -submap_size; i <= submap_size; ++i) {
    //     int keyNear = key + i; // see https://github.com/gisbi-kim/SC-A-LOAM/issues/7 ack. @QiMingZhenFan found the error and modified as below. 
    //     if (keyNear < 0 || keyNear >= int(keyframeLaserClouds.size()) )
    //         continue;

    //     mKF.lock(); 
    //     *nearKeyframes += * local2global(keyframeLaserClouds[keyNear], keyframePosesUpdated[root_idx]);
    //     mKF.unlock(); 
    // }

    *nearKeyframes = * local2global(keyframeLaserClouds[key], keyframePosesUpdated[root_idx]);

    if (nearKeyframes->empty())
        return;

    // downsample near keyframes
    // pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
    // downSizeFilterICP.setInputCloud(nearKeyframes);
    // downSizeFilterICP.filter(*cloud_temp);
    // *nearKeyframes = *cloud_temp;
} // loopFindNearKeyframesCloud


std::optional<gtsam::Pose3> doGICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx )
{
    // parse pointclouds
    int historyKeyframeSearchNum = 10; // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0, _loop_kf_idx); // use same root of loop kf idx 
    loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx); 

    if (cureKeyframeCloud->points.size() && targetKeyframeCloud->points.size()) {
        // loop verification 
        sensor_msgs::PointCloud2 cureKeyframeCloudMsg;
        pcl::toROSMsg(*cureKeyframeCloud, cureKeyframeCloudMsg);
        cureKeyframeCloudMsg.header.frame_id = "map";
        pubLoopScanLocal.publish(cureKeyframeCloudMsg);

        sensor_msgs::PointCloud2 targetKeyframeCloudMsg;
        pcl::toROSMsg(*targetKeyframeCloud, targetKeyframeCloudMsg);
        targetKeyframeCloudMsg.header.frame_id = "map";
        pubLoopSubmapLocal.publish(targetKeyframeCloudMsg);

        // std::cout << "registration: FAST_GICP" << std::endl;
        static fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>::Ptr gicp(new fast_gicp::FastGICP<pcl::PointXYZ, pcl::PointXYZ>());
        gicp->setNumThreads(4);
        gicp->setTransformationEpsilon(1e-6);
        gicp->setMaximumIterations(100);
        // gicp->setMaxCorrespondenceDistance(0.2);
        // gicp->setCorrespondenceRandomness(20);
        gicp->setMaxCorrespondenceDistance(0.3);
        gicp->setCorrespondenceRandomness(30);

        // Align pointclouds
        gicp->setInputSource(cureKeyframeCloud);
        gicp->setInputTarget(targetKeyframeCloud);

        pcl::PointCloud<pcl::PointXYZ>::Ptr transCurrentCloudInWorld(new pcl::PointCloud<pcl::PointXYZ>());
        gicp->align(*transCurrentCloudInWorld);
        gicp->swapSourceAndTarget();

        // sensor_msgs::PointCloud2 cureKeyframe_mv_CloudMsg;
        // // *transformPointCloud(cureKeyframeCloud,poseFrom)
        // // pcl::toROSMsg(*transformPointCloud(cureKeyframeCloud,poseFrom), cureKeyframe_mv_CloudMsg);
        // pcl::toROSMsg(*transformPointCloud_mp(cureKeyframeCloud,aligntest), cureKeyframe_mv_CloudMsg);
        // cureKeyframe_mv_CloudMsg.header.frame_id = "map";
        // pubLoopScan_aft_Local.publish(cureKeyframe_mv_CloudMsg);

        float loopFitnessScoreThreshold = 0.3; // user parameter but fixed low value is safe. 
        if (gicp->hasConverged() == false || gicp->getFitnessScore() > loopFitnessScoreThreshold) {
            std::cout << "[SC loop] GICP fitness test failed (" << gicp->getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
            return std::nullopt;
        } else {
            std::cout << "[SC loop] GICP fitness test passed (" << gicp->getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
            // cnt += 1;
            // std::cout << "but optimization is not occured for testing performance" << std::endl;
            // return std::nullopt;
        }

        // // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        Eigen::Affine3f correctionLidarFrame;
        correctionLidarFrame = gicp->getFinalTransformation();
        
        pcl::getTranslationAndEulerAngles (correctionLidarFrame, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = Pose3(Rot3::RzRyRx(0.0, 0.0, 0.0), Point3(0.0, 0.0, 0.0));

        // loop verification 
        sensor_msgs::PointCloud2 cureKeyframe_mv_CloudMsg;
        // *transformPointCloud(cureKeyframeCloud,poseFrom)
        pcl::toROSMsg(*transformPointCloud(cureKeyframeCloud,poseFrom), cureKeyframe_mv_CloudMsg);
        // pcl::toROSMsg(*transformPointCloud_mp(cureKeyframeCloud,guess), cureKeyframe_mv_CloudMsg);
        cureKeyframe_mv_CloudMsg.header.frame_id = "map";
        pubLoopScan_aft_Local.publish(cureKeyframe_mv_CloudMsg);

        // if (cnt>1) {
        //     // return std::nullopt;
        //     exit(0);
        // }

        cnt += 1;

        return poseFrom.between(poseTo);
    }
} // doGICPVirtualRelative

void localmap_pg()
{   
    while(1)
    {
		while ( !odometryBuf.empty() && !fullResBuf.empty() )
        {
			mBuf.lock();       
            while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < fullResBuf.front()->header.stamp.toSec())
                odometryBuf.pop();
            if (odometryBuf.empty())
            {
                mBuf.unlock();
                break;
            }

            // Time equal check
            // timeLaserOdometry = carposeBuf.front()->header.stamp.toSec();
            timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
            timeLaser = fullResBuf.front()->header.stamp.toSec();
            // TODO

            laserCloudFullRes->clear();
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
            fullResBuf.pop();

            Pose6D pose_curr = getOdom(odometryBuf.front());
            odometryBuf.pop();

            // Pose6D pose_curr = carOdom(carposeBuf.front());
            // carposeBuf.pop();

            mBuf.unlock(); 

            // Early reject by counting local delta movement (for equi-spereated kf drop)
            
            odom_pose_prev = odom_pose_curr;
            odom_pose_curr = pose_curr;
            Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform
            double delta_translation = sqrt(dtf.x*dtf.x + dtf.y*dtf.y + dtf.z*dtf.z); // note: absolute value. 
            keytranslationAccumulated += delta_translation;
            keyrotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.  

            if( keytranslationAccumulated > keyframeMeterGap || keyrotaionAccumulated > keyframeRadGap ) {
                isNowKeyFrame = true;
                keytranslationAccumulated = 0.0; // reset 
                keyrotaionAccumulated = 0.0; // reset 
            } else {
                isNowKeyFrame = false;
            }

            translationAccumulated += delta_translation;
            rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.  

            if( translationAccumulated > frameMeterGap || rotaionAccumulated > frameRadGap ) {
                isNowFrame = true;
                translationAccumulated = 0.0; // reset 
                rotaionAccumulated = 0.0; // reset 
            } else {
                isNowFrame = false;
            }

            // cout << "translationAccumulated" << translationAccumulated << endl;
            // cout << "keytranslationAccumulated" << keytranslationAccumulated << endl;

            if( ! isNowFrame ) 
                continue; 

            // if (prev_keyframe_node_idx > 0){
            //     continue; 
            // }

            //
            // Save data and Add consecutive node 
            //
            // pcl::PointCloud<PointType>::Ptr thisKeyFrameDS(new pcl::PointCloud<PointType>());
            // downSizeFilterScancontext.setInputCloud(thisKeyFrame);
            // downSizeFilterScancontext.filter(*thisKeyFrameDS);
            mKF.lock(); 
            // keyframeLaserClouds.push_back(thisKeyFrameDS);
            keyframeLaserClouds.push_back(thisKeyFrame);
            keyframePoses.push_back(pose_curr);
            keyframePosesUpdated.push_back(pose_curr); // init
            keyframeTimes.push_back(timeLaserOdometry);

            laserCloudMapPGORedraw = true;
            mKF.unlock(); 


            const int prev_node_idx = keyframePoses.size() - 2; 
            const int curr_node_idx = keyframePoses.size() - 1; // becuase cpp starts with 0 (actually this index could be any number, but for simple implementation, we follow sequential indexing)
            if( ! gtSAMgraphMade /* prior node */) {
                const int init_node_idx = 0; 
                gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
                // auto poseOrigin = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

                mtxPosegraph.lock();
                {
                    // prior factor 
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                    initialEstimate.insert(init_node_idx, poseOrigin);
                    // runISAM2opt();          
                }   
                mtxPosegraph.unlock();

                gtSAMgraphMade = true; 

                cout << "posegraph prior node " << init_node_idx << " added" << endl;
            } else /* consecutive node (and odom factor) after the prior added */ { // == keyframePoses.size() > 1 
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

                mtxPosegraph.lock();
                {
                    // odom factor
                    gtsam::Pose3 relPose = poseFrom.between(poseTo);
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relPose, odomNoise));

                    initialEstimate.insert(curr_node_idx, poseTo);                
                    // writeEdge({prev_node_idx, curr_node_idx}, relPose, edges_str); // giseop
                    // runISAM2opt();
                }
                mtxPosegraph.unlock();

                if(curr_node_idx % 10 == 0)
                    cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }

            // cout << "isNowKeyFrame" << isNowKeyFrame << endl;

            if( isNowKeyFrame ) {
                // cout << "1111" << endl;
                cout << prev_keyframe_node_idx << " // " << curr_node_idx << endl;
                auto local_relative_pose_optional = doGICPVirtualRelative(prev_keyframe_node_idx, curr_node_idx);
                if(local_relative_pose_optional) {
                    gtsam::Pose3 local_relative_pose = local_relative_pose_optional.value();
                    mtxPosegraph.lock();
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_keyframe_node_idx, curr_node_idx, local_relative_pose, robustOdomNoise));
                    // writeEdge({prev_node_idx, curr_node_idx}, local_relative_pose, edges_str); // giseop
                    cout << local_relative_pose << endl;
                    // runISAM2opt();
                    mtxPosegraph.unlock();
                } 
                keynodes.push_back(curr_node_idx);
                prev_keyframe_node_idx = curr_node_idx;
            }
        }

        // ps. 
        // pub path and point cloud in another thread

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // localmap_pg

void performSCLoopClosure(void)
{
    if( int(keyframePoses.size()) < 30) // do not try too early 
        return;

    std::vector<int> node_vec;
    for(int node_idx = 0; node_idx <= int(keyframePoses.size())-2; node_idx++) {
        const Pose6D& lc_poseFrom = keyframePoses.at(node_idx);
        const Pose6D& lc_poseTo = keyframePoses.at(int(keyframePoses.size())-1);
        double lc_rel_dist = (lc_poseFrom.x - lc_poseTo.x)*(lc_poseFrom.x - lc_poseTo.x) + (lc_poseFrom.y - lc_poseTo.y)*(lc_poseFrom.y - lc_poseTo.y);

        const int del_idx = (int(keyframePoses.size())-1) - node_idx;
        // cout << "rel_distance is " << lc_rel_dist << " // del_idx is " << del_idx << endl;
        if (lc_rel_dist < 0.5 && del_idx > 30){
            // cout << "rel_distance is " << lc_rel_dist << " // del_idx is " << del_idx << endl;
            node_vec.push_back(node_idx);
        }
    }
    // cout << "node_vec.size(): " << node_vec.size() << endl;
    if (node_vec.size() == 0){
        // cout << "node_vec.size(): " << node_vec.size() << endl;
        return;
    }
    else {
        const int prev_node_idx = node_vec.back();
        const int curr_node_idx = keyframePoses.size() - 1;
        cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        mBuf.lock();
        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // addding actual 6D constraints in the other thread, icp_calculation.
        mBuf.unlock();
    }
} // performSCLoopClosure

void process_lcd(void)
{
    float loopClosureFrequency = 5.0; // can change 
    ros::Rate rate(loopClosureFrequency);
    while (ros::ok())
    {
        rate.sleep();
        performSCLoopClosure();
        // performRSLoopClosure(); // TODO
    }
} // process_lcd

void process_icp(void)
{
    while(1)
    {
		while ( !scLoopICPBuf.empty() )
        {
            if( scLoopICPBuf.size() > 30 ) {
                ROS_WARN("Too many loop clousre candidates to be ICPed is waiting ... Do process_lcd less frequently (adjust loopClosureFrequency)");
            }

            mBuf.lock(); 
            std::pair<int, int> loop_idx_pair = scLoopICPBuf.front();
            scLoopICPBuf.pop();
            mBuf.unlock(); 

            const int prev_node_idx = loop_idx_pair.first;
            const int curr_node_idx = loop_idx_pair.second;
            auto relative_pose_optional = doGICPVirtualRelative(prev_node_idx, curr_node_idx);
            if(relative_pose_optional) {
                gtsam::Pose3 relative_pose = relative_pose_optional.value();
                mtxPosegraph.lock();
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
                // writeEdge({prev_node_idx, curr_node_idx}, relative_pose, edges_str); // giseop
                // runISAM2opt();
                mtxPosegraph.unlock();
            } 
        }

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_icp

void process_viz_path(void)
{
    float hz = 20.0; 
    ros::Rate rate(hz);
    while (ros::ok()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubPath();
        }
    }
}

void process_isam(void)
{
    float hz = 20; 
    ros::Rate rate(hz);
    while (ros::ok()) {
        rate.sleep();
        if( gtSAMgraphMade ) {
            mtxPosegraph.lock();
            runISAM2opt();
            // cout << "running isam2 optimization ..." << endl;
            mtxPosegraph.unlock();

            // saveOptimizedVerticesKITTIformat(isamCurrentEstimate, savepose_name); // pose
            // saveOptimizedPose(isamCurrentEstimate, savepose_name);
            // saveOdometryVerticesKITTIformat(odomKITTIformat); // pose
            // saveGTSAMgraphG2oFormat(isamCurrentEstimate);
        }
    }
}

void pubMap(void)
{
    // int SKIP_FRAMES = 2; // sparse map visulalization to save computations 
    // int counter = 0;

    laserCloudMapPGO->clear();
    recentlaserCloudMapPGO->clear();

    mKF.lock(); 
    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()); node_idx++) {
    // for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) {
    //     *laserCloudMapPGO += *local2global_map(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx], node_idx , recentIdxUpdated);
    //     // if(counter % SKIP_FRAMES == 0) {
    //     //     *laserCloudMapPGO += *local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
    //     // }
    //     // counter++;
    // }
    for_each(keynodes.begin(), keynodes.end(), [&](int& node_idx){
        *laserCloudMapPGO += *local2global_map(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx], node_idx , recentIdxUpdated);        //output : 1 2 3 4
    });

    *recentlaserCloudMapPGO = *local2global_map(keyframeLaserClouds[recentIdxUpdated-1], keyframePosesUpdated[recentIdxUpdated-1], recentIdxUpdated-1 , recentIdxUpdated);

    mKF.unlock(); 

    // downSizeFilterMapPGO.setInputCloud(laserCloudMapPGO);
    // downSizeFilterMapPGO.filter(*laserCloudMapPGO);

    sensor_msgs::PointCloud2 laserCloudMapPGOMsg;
    pcl::toROSMsg(*laserCloudMapPGO, laserCloudMapPGOMsg);
    laserCloudMapPGOMsg.header.frame_id = "map";
    pubMapAftPGO.publish(laserCloudMapPGOMsg);

    sensor_msgs::PointCloud2 recentlaserCloudMapPGOMsg;
    pcl::toROSMsg(*recentlaserCloudMapPGO, recentlaserCloudMapPGOMsg);
    recentlaserCloudMapPGOMsg.header.frame_id = "map";
    recentpubMapAftPGO.publish(recentlaserCloudMapPGOMsg);
}

void process_viz_map(void)
{
    float vizmapFrequency = 20; // 0.1 means run onces every 10s
    ros::Rate rate(vizmapFrequency);
    while (ros::ok()) {
        rate.sleep();
        if(recentIdxUpdated > 1) {
            pubMap();
        }
    }
} // pointcloud_viz


int main(int argc, char **argv)
{
	ros::init(argc, argv, "avm2pointcloud_node");
	ros::NodeHandle nh;

    // system params 
	nh.param<double>("keyframe_meter_gap", keyframeMeterGap, 0.5); // pose assignment every k m move 
	nh.param<double>("keyframe_deg_gap", keyframeDegGap, 10); // pose assignment every k deg rot
	nh.param<double>("keyframe_meter_gap", frameMeterGap, 0.05); // pose assignment every k m move 
	nh.param<double>("keyframe_deg_gap", frameDegGap, 1); // pose assignment every k deg rot 
    keyframeRadGap = deg2rad(keyframeDegGap);
    frameRadGap = deg2rad(frameDegGap);

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    initNoises();

    // float filter_size = 0.1; 
    // downSizeFilterScancontext.setLeafSize(filter_size, filter_size, filter_size);
    // downSizeFilterICP.setLeafSize(filter_size, filter_size, filter_size);

    // double mapVizFilterSize;
	// nh.param<double>("mapviz_filter_size", mapVizFilterSize, 0.1); // pose assignment every k frames 
    // downSizeFilterMapPGO.setLeafSize(mapVizFilterSize, mapVizFilterSize, mapVizFilterSize);

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/AVM_PointCloud", 100, laserCloudFullResHandler);
    // ros::Subscriber subDRData = nh.subscribe("/DRData_aft_pointcloud", 100, CallbackDRData);

	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/currentPose", 100, laserOdometryHandler);
    
	pubOdomAftPGO = nh.advertise<nav_msgs::Odometry>("/aft_pgo_odom", 100);
	pubOdomRepubVerifier = nh.advertise<nav_msgs::Odometry>("/repub_odom", 100);
	pubPathAftPGO = nh.advertise<nav_msgs::Path>("/aft_pgo_path", 100);
    // pubPathGT = nh.advertise<nav_msgs::Path>("/aft_pgo_gt_path", 100);
	pubMapAftPGO = nh.advertise<sensor_msgs::PointCloud2>("/aft_pgo_map", 100);
    recentpubMapAftPGO = nh.advertise<sensor_msgs::PointCloud2>("/recent_aft_pgo_map", 1);

    // pubOdomAftPGO_Array = nh.advertise<std_msgs::Float32MultiArray>("/LocalizationData", 100);

	pubLoopScanLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_scan_local", 100);
    pubLoopScan_aft_Local = nh.advertise<sensor_msgs::PointCloud2>("/aft_loop_scan_local", 100);
	pubLoopSubmapLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_submap_local", 100);

	std::thread posegraph_slam {localmap_pg}; // local map pose graph construction
	// std::thread lc_detection {process_lcd}; // loop closure detection 
	// std::thread icp_calculation {process_icp}; // loop constraint calculation via icp 
	// std::thread isam_update {process_isam}; // if you want to call less isam2 run (for saving redundant computations and no real-time visulization is required), uncommment this and comment all the above runisam2opt when node is added. 

	std::thread viz_map {process_viz_map}; // visualization - map (low frequency because it is heavy)
	std::thread viz_path {process_viz_path}; // visualization - path (high frequency)

 	ros::spin();

	return 0;
}