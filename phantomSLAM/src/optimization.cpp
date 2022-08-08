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
#include <std_msgs/Bool.h>



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

int m_AVM_IMG_WIDTH = 0, m_AVM_IMG_HEIGHT = 0, m_SLAM_IMG_WIDTH = 0;
double m_REAL_OCCUPANCY_SIZE_X = 0.0, m_REAL_OCCUPANCY_SIZE_Y = 0.0;
double m_SLAM_OCCUPANCY_SIZE_X = 0.0, m_SLAM_OCCUPANCY_SIZE_Y = 0.0;
double m_METER_PER_PIXEL = 0.0;

double keyframeMeterGap;
double keyframeDegGap, keyframeRadGap;
double translationAccumulated = 1000000.0; // large value means must add the first given frame.
double rotaionAccumulated = 1000000.0; // large value means must add the first given frame.

bool isNowKeyFrame = false; 

Pose6D odom_pose_prev {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init 
Pose6D odom_pose_curr {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // init pose is zero 

std::queue<nav_msgs::Odometry::ConstPtr> odometryBuf;
std::queue<sensor_msgs::PointCloud2ConstPtr> fullResBuf;
std::queue<std::pair<int, int> > scLoopICPBuf;

std::mutex mBuf;
std::mutex mKF;

double timeLaserOdometry = 0.0;
double timeLaser = 0.0;

bool m_parkingStartFlag = true;

pcl::PointCloud<PointType>::Ptr laserCloudFullRes(new pcl::PointCloud<PointType>());
pcl::PointCloud<PointType>::Ptr laserCloudMapAfterPGO(new pcl::PointCloud<PointType>());

pcl::PointCloud<pcl::PointXYZ>::Ptr LoopCurCloudInWorld(new pcl::PointCloud<pcl::PointXYZ>());
pcl::PointCloud<pcl::PointXYZ>::Ptr LoopTarCloudInWorld(new pcl::PointCloud<pcl::PointXYZ>());

std::vector<pcl::PointCloud<PointType>::Ptr> keyframeLaserClouds; 
std::vector<Pose6D> keyframePoses;
std::vector<Pose6D> keyframePosesUpdated;
std::vector<double> keyframeTimes;
int recentIdxUpdated = 0;

gtsam::NonlinearFactorGraph gtSAMgraph;
bool gtSAMgraphMade = false;
gtsam::Values initialEstimate;
gtsam::ISAM2 *isam;
gtsam::Values isamCurrentEstimate;

noiseModel::Diagonal::shared_ptr priorNoise;
noiseModel::Diagonal::shared_ptr odomNoise;
noiseModel::Base::shared_ptr robustLoopNoise;
noiseModel::Base::shared_ptr robustGPSNoise;

std::mutex mtxICP;
std::mutex mtxPosegraph;
std::mutex mtxRecentPose;

pcl::PointCloud<pcl::PointXYZRGB>::Ptr laserCloudMapPGO(new pcl::PointCloud<pcl::PointXYZRGB>());
pcl::PointCloud<pcl::PointXYZRGB>::Ptr recentlaserCloudMapPGO(new pcl::PointCloud<pcl::PointXYZRGB>());


double recentOptimizedX = 0.0;
double recentOptimizedY = 0.0;

ros::Publisher pubMapAftPGO, pubOdomAftPGO, pubPathAftPGO, pubOdomAftPGO_Array, recentpubMapAftPGO; // , pubPathGT;
ros::Publisher pubLoopScanLocal, pubLoopSubmapLocal, pubLoopScan_aft_Local;

// void CallbackReplanning(const std_msgs::Bool::ConstPtr& msg);

std::string padZeros(int val, int num_digits = 6) 
{
  std::ostringstream out;
  out << std::internal << std::setfill('0') << std::setw(num_digits) << val;
  return out.str();
}


// JW add
void CallbackParkingStart(const nav_msgs::Path::ConstPtr& msg) {
    m_parkingStartFlag = true;
}

void CallbackReplanning(const std_msgs::Bool::ConstPtr& msg) {
    if (msg->data) {
        m_parkingStartFlag = false;
        // cout << "REPLANNING !!! [Dead Reckoning]" << endl;
    }
}

gtsam::Pose3 Pose6DtoGTSAMPose3(const Pose6D& p)
{
    return gtsam::Pose3( gtsam::Rot3::RzRyRx(p.roll, p.pitch, p.yaw), gtsam::Point3(p.x, p.y, p.z) );
} // Pose6DtoGTSAMPose3


void saveOptimizedPose(gtsam::Values _estimates, std::string _filename)
{
    using namespace gtsam;

    // ref from gtsam's original code "dataset.cpp"
    std::fstream stream(_filename.c_str(), std::fstream::out);

    for(const auto& key_value: _estimates) {
        auto p = dynamic_cast<const GenericValue<Pose3>*>(&key_value.value);
        if (!p) continue;

        const Pose3& pose = p->value();

        Point3 t = pose.translation();
        Rot3 R = pose.rotation();
        auto col1 = R.column(1); // Point3
        auto col2 = R.column(2); // Point3
        auto col3 = R.column(3); // Point3

        stream << col1.x() << " " << col2.x() << " " << col3.x() << " " << t.x() << " "
               << col1.y() << " " << col2.y() << " " << col3.y() << " " << t.y() << " "
               << col1.z() << " " << col2.z() << " " << col3.z() << " " << t.z() << std::endl;
    }
}


void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr &_laserOdometry)
{
	mBuf.lock();
	odometryBuf.push(_laserOdometry);
	mBuf.unlock();
} // laserOdometryHandler

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

void pubLoc( void )
{
    std_msgs::Float32MultiArray odomAftPGO_array;
    std::vector<double> vec1 = {0.0, 0.0, 0.0, 0.0, 0};
    if(recentIdxUpdated > 1 && m_parkingStartFlag ) {
        // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()) - 1; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
        const Pose6D& pose_est = keyframePosesUpdated.at(recentIdxUpdated-1); // upodated poses

        vec1 = {pose_est.x, pose_est.y, pose_est.yaw, 0.0, 0};
        }
    mKF.lock(); 

    // set up dimensions
    odomAftPGO_array.layout.dim.push_back(std_msgs::MultiArrayDimension());
    odomAftPGO_array.layout.dim[0].size = vec1.size();
    odomAftPGO_array.layout.dim[0].stride = 1;
    odomAftPGO_array.layout.dim[0].label = "pose"; // or whatever name you typically use to index vec1

    // copy in the data
    odomAftPGO_array.data.clear();
    odomAftPGO_array.data.insert(odomAftPGO_array.data.end(), vec1.begin(), vec1.end());

    mKF.unlock(); 
    pubOdomAftPGO_Array.publish(odomAftPGO_array); // poses array 
} // pubLoc

void pubPath( void )
{
    // pub odom and path 
    nav_msgs::Odometry odomAftPGO;
    nav_msgs::Path pathAftPGO;
    pathAftPGO.header.frame_id = "map";

    mKF.lock(); 

    // for (int node_idx=0; node_idx < int(keyframePosesUpdated.size()) - 1; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) // -1 is just delayed visualization (because sometimes mutexed while adding(push_back) a new one)
    {
        const Pose6D& pose_est = keyframePosesUpdated.at(node_idx); // upodated poses

        nav_msgs::Odometry odomAftPGOthis;
        odomAftPGOthis.header.frame_id = "map";
        odomAftPGOthis.child_frame_id = "aft_pgo";
        odomAftPGOthis.header.stamp = ros::Time().fromSec(keyframeTimes.at(node_idx));

        odomAftPGOthis.pose.pose.position.x = pose_est.x;
        odomAftPGOthis.pose.pose.position.y = pose_est.y;
        odomAftPGOthis.pose.pose.position.z = pose_est.z;

        odomAftPGOthis.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(pose_est.roll, pose_est.pitch, pose_est.yaw);
        odomAftPGO = odomAftPGOthis;

        geometry_msgs::PoseStamped poseStampAftPGO;
        poseStampAftPGO.header = odomAftPGOthis.header;
        poseStampAftPGO.pose = odomAftPGOthis.pose.pose;

        pathAftPGO.header.stamp = odomAftPGOthis.header.stamp;
        pathAftPGO.header.frame_id = "map";
        pathAftPGO.poses.push_back(poseStampAftPGO);

    }

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


std::optional<gtsam::Pose3> doICPVirtualRelative( int _loop_kf_idx, int _curr_kf_idx )
{
    // parse pointclouds
    int historyKeyframeSearchNum = 10; // enough. ex. [-25, 25] covers submap length of 50x1 = 50m if every kf gap is 1m
    pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
    pcl::PointCloud<PointType>::Ptr targetKeyframeCloud(new pcl::PointCloud<PointType>());
    loopFindNearKeyframesCloud(cureKeyframeCloud, _curr_kf_idx, 0, _loop_kf_idx); // use same root of loop kf idx 
    loopFindNearKeyframesCloud(targetKeyframeCloud, _loop_kf_idx, historyKeyframeSearchNum, _loop_kf_idx); 


    const Pose6D& loop_pose = keyframePoses.at(_loop_kf_idx);
    const Pose6D& curr_pose = keyframePoses.at(_curr_kf_idx);

    double rel_dist_x = loop_pose.x - curr_pose.x;
    double rel_dist_y = loop_pose.y - curr_pose.y;
    // double rel_dist_x = curr_pose.x - loop_pose.x;
    // double rel_dist_y = curr_pose.y - loop_pose.y;
    // double rel_dist_yaw = loop_pose.yaw - curr_pose.yaw;
    double rel_dist_yaw = curr_pose.yaw - loop_pose.yaw;

    // Eigen::Matrix4f Tar_mat = Eigen::Matrix4f::Identity(); 
    // Tar_mat(0,0) = cos(loop_pose.yaw);
    // Tar_mat(0,1) = sin(loop_pose.yaw);
    // Tar_mat(1,0) = -sin(loop_pose.yaw);
    // Tar_mat(1,1) = cos(loop_pose.yaw);
    // Tar_mat(0,3) = loop_pose.x;
    // Tar_mat(1,3) = loop_pose.y;
    // *LoopTarCloudInWorld = *transformPointCloud_mp(targetKeyframeCloud,Tar_mat);

    // Eigen::Matrix4f Cur_mat = Eigen::Matrix4f::Identity(); 
    // Cur_mat(0,0) = cos(curr_pose.yaw);
    // Cur_mat(0,1) = sin(curr_pose.yaw);
    // Cur_mat(1,0) = -sin(curr_pose.yaw);
    // Cur_mat(1,1) = cos(curr_pose.yaw);
    // Cur_mat(0,3) = curr_pose.x;
    // Cur_mat(1,3) = curr_pose.y;
    // *LoopCurCloudInWorld = *transformPointCloud_mp(cureKeyframeCloud,Cur_mat);

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
    gicp->setTransformationEpsilon(1e-4);
    gicp->setMaximumIterations(300);
    gicp->setMaxCorrespondenceDistance(0.16* 200 / m_AVM_IMG_WIDTH);

    // gicp->setMaxCorrespondenceDistance(0.008);
    // gicp->setCorrespondenceRandomness(40);

    // Align pointclouds
    gicp->setInputSource(cureKeyframeCloud);
    gicp->setInputTarget(targetKeyframeCloud);

    pcl::PointCloud<pcl::PointXYZ>::Ptr transCurrentCloudInWorld(new pcl::PointCloud<pcl::PointXYZ>());

    gicp->align(*transCurrentCloudInWorld);
    gicp->swapSourceAndTarget();


    float loopFitnessScoreThreshold = 0.3; // user parameter but fixed low value is safe. 
    if (gicp->hasConverged() == false || gicp->getFitnessScore() > loopFitnessScoreThreshold) {
        // std::cout << "[SC loop] GICP fitness test failed (" << gicp->getFitnessScore() << " > " << loopFitnessScoreThreshold << "). Reject this SC loop." << std::endl;
        return std::nullopt;
    } 
    // else {
    //     std::cout << "[SC loop] GICP fitness test passed (" << gicp->getFitnessScore() << " < " << loopFitnessScoreThreshold << "). Add this SC loop." << std::endl;
    //     // std::cout << "but optimization is not occured for testing performance" << std::endl;
    //     // return std::nullopt;
    // }

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

    return poseFrom.between(poseTo);
} // doICPVirtualRelative

void process_pg()
{  
    while(1)
    {
		while (!odometryBuf.empty() && !fullResBuf.empty() )
        {
            //
            // pop and check keyframe is or not  
            // 
			mBuf.lock();       
            while (!odometryBuf.empty() && odometryBuf.front()->header.stamp.toSec() < fullResBuf.front()->header.stamp.toSec())
                odometryBuf.pop();
            if (odometryBuf.empty())
            {
                mBuf.unlock();
                break;
            }


            // Time equal check
            timeLaserOdometry = odometryBuf.front()->header.stamp.toSec();
            timeLaser = fullResBuf.front()->header.stamp.toSec();
            // TODO

            laserCloudFullRes->clear();
            pcl::PointCloud<PointType>::Ptr thisKeyFrame(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(*fullResBuf.front(), *thisKeyFrame);
            fullResBuf.pop();

            Pose6D pose_curr = getOdom(odometryBuf.front());
            odometryBuf.pop();      

            mBuf.unlock(); 

            // Early reject by counting local delta movement (for equi-spereated kf drop)
            
            odom_pose_prev = odom_pose_curr;
            odom_pose_curr = pose_curr;
            Pose6D dtf = diffTransformation(odom_pose_prev, odom_pose_curr); // dtf means delta_transform

            double delta_translation = sqrt(dtf.x*dtf.x + dtf.y*dtf.y + dtf.z*dtf.z); // note: absolute value. 
            translationAccumulated += delta_translation;
            rotaionAccumulated += (dtf.roll + dtf.pitch + dtf.yaw); // sum just naive approach.  

            if( translationAccumulated > keyframeMeterGap || rotaionAccumulated > keyframeRadGap ) {
                isNowKeyFrame = true;
                translationAccumulated = 0.0; // reset 
                rotaionAccumulated = 0.0; // reset 
            } else {
                isNowKeyFrame = false;
            }
            
            if( ! isNowKeyFrame ) 
                continue; 

            mKF.lock(); 

            keyframeLaserClouds.push_back(thisKeyFrame);
            keyframePoses.push_back(pose_curr);
            keyframePosesUpdated.push_back(pose_curr); // init
            keyframeTimes.push_back(timeLaserOdometry);

            mKF.unlock(); 

            const int prev_node_idx = keyframePoses.size() - 2; 
            const int curr_node_idx = keyframePoses.size() - 1; // becuase cpp starts with 0 (actually this index could be any number, but for simple implementation, we follow sequential indexing)
            if( ! gtSAMgraphMade/* prior node */) {
                const int init_node_idx = 0; 
                gtsam::Pose3 poseOrigin = Pose6DtoGTSAMPose3(keyframePoses.at(init_node_idx));
                // auto poseOrigin = gtsam::Pose3(gtsam::Rot3::RzRyRx(0.0, 0.0, 0.0), gtsam::Point3(0.0, 0.0, 0.0));

                mtxPosegraph.lock();
                {
                    // prior factor 
                    gtSAMgraph.add(gtsam::PriorFactor<gtsam::Pose3>(init_node_idx, poseOrigin, priorNoise));
                    initialEstimate.insert(init_node_idx, poseOrigin);
                    runISAM2opt();        
                    // cout << "posegraph prior node " << init_node_idx << " added" << endl;
                }   
                mtxPosegraph.unlock();

                gtSAMgraphMade = true; 

            } else {/* consecutive node (and odom factor) after the prior added */  // == keyframePoses.size() > 1 
                gtsam::Pose3 poseFrom = Pose6DtoGTSAMPose3(keyframePoses.at(prev_node_idx));
                gtsam::Pose3 poseTo = Pose6DtoGTSAMPose3(keyframePoses.at(curr_node_idx));

                mtxPosegraph.lock();
                {
                    // odom factor
                    // cout << "posegraph prior node " << curr_node_idx << " added" << endl;
                    gtsam::Pose3 relPose = poseFrom.between(poseTo);
                    gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relPose, odomNoise));

                    initialEstimate.insert(curr_node_idx, poseTo);                
                    // writeEdge({prev_node_idx, curr_node_idx}, relPose, edges_str); // giseop
                    runISAM2opt();
                }
                mtxPosegraph.unlock();

                // if(curr_node_idx % 10 == 0)
                //     cout << "posegraph odom node " << curr_node_idx << " added." << endl;
            }
            // gtSAMgraph.print("GTSAM Graph:\n");
            // if want to print the current graph, use gtSAMgraph.print("\nFactor Graph:\n");
        }

        // ps. 
        // pub path and point cloud in another thread

        // wait (must required for running the while loop)
        std::chrono::milliseconds dura(2);
        std::this_thread::sleep_for(dura);
    }
} // process_pg

void process_pub_loc(void)
{
    float hz = 20.0; 
    ros::Rate rate(hz);
    while (ros::ok()) {
        rate.sleep();
        pubLoc();
    }
}


void performSCLoopClosure(void)
{
    std::vector<int> node_vec;

    // if ((int(keyframePoses.size())-1) % 40 == 1) {
    //     if ((int(keyframePoses.size())-1)-40 < 0) {
    //         return;
    //     }
    //     const int prev_node_idx = keyframePoses.size() - 41;
    //     const int curr_node_idx = keyframePoses.size() - 1;
    //     // cout << "iter_lc" << prev_node_idx << " and " << curr_node_idx << "" << endl;

    //     mBuf.lock();
    //     scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
    //     // addding actual 6D constraints in the other thread, icp_calculation.
    //     mBuf.unlock();
    // }

    if( int(keyframePoses.size()) < 30) // do not try too early 
        return;
    
    for(int node_idx = 0; node_idx <= int(keyframePoses.size())-2; node_idx++) {
        const Pose6D& lc_poseFrom = keyframePoses.at(node_idx);
        const Pose6D& lc_poseTo = keyframePoses.at(int(keyframePoses.size())-1);
        double lc_rel_dist = (lc_poseFrom.x - lc_poseTo.x)*(lc_poseFrom.x - lc_poseTo.x) + (lc_poseFrom.y - lc_poseTo.y)*(lc_poseFrom.y - lc_poseTo.y);

        const int del_idx = (int(keyframePoses.size())-1) - node_idx;
        // cout << "rel_distance is " << lc_rel_dist << " // del_idx is " << del_idx << endl;
        if (lc_rel_dist < 0.3 && del_idx > 120){
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
        // cout << "Loop detected! - between " << prev_node_idx << " and " << curr_node_idx << "" << endl;

        mBuf.lock();
        scLoopICPBuf.push(std::pair<int, int>(prev_node_idx, curr_node_idx));
        // addding actual 6D constraints in the other thread, icp_calculation.
        mBuf.unlock();
    }
} // performSCLoopClosure

void process_lcd(void)
{
    float loopClosureFrequency = 10.0; // can change 
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
            auto relative_pose_optional = doICPVirtualRelative(prev_node_idx, curr_node_idx);
            if(relative_pose_optional) {
                gtsam::Pose3 relative_pose = relative_pose_optional.value();
                mtxPosegraph.lock();
                gtSAMgraph.add(gtsam::BetweenFactor<gtsam::Pose3>(prev_node_idx, curr_node_idx, relative_pose, robustLoopNoise));
                // writeEdge({prev_node_idx, curr_node_idx}, relative_pose, edges_str); // giseop
                runISAM2opt();
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
    float hz = 10.0; 
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
    float hz = 10; 
    ros::Rate rate(hz);
    while (ros::ok()) {
        rate.sleep();
        if( gtSAMgraphMade ) {
            mtxPosegraph.lock();
            // cout << "running isam2 optimization ..." << endl;
            runISAM2opt();
            mtxPosegraph.unlock();
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
    for (int node_idx=0; node_idx < recentIdxUpdated; node_idx++) {
        *laserCloudMapPGO += *local2global_map(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx], node_idx , recentIdxUpdated);
        // if(counter % SKIP_FRAMES == 0) {
        //     *laserCloudMapPGO += *local2global(keyframeLaserClouds[node_idx], keyframePosesUpdated[node_idx]);
        // }
        // counter++;
    }

    *recentlaserCloudMapPGO = *local2global_map(keyframeLaserClouds[recentIdxUpdated-1], keyframePosesUpdated[recentIdxUpdated-1], recentIdxUpdated-1 , recentIdxUpdated);

    mKF.unlock(); 

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
    float vizmapFrequency = 1; // 0.1 means run onces every 10s
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
	ros::init(argc, argv, "optimization_node");
	ros::NodeHandle nh;

    // system params 
	nh.param<double>("keyframe_meter_gap", keyframeMeterGap, 0.0075); // pose assignment every k m move 
	nh.param<double>("keyframe_deg_gap", keyframeDegGap, 0.1); // pose assignment every k deg rot 
    keyframeRadGap = deg2rad(keyframeDegGap);

    nh.getParam("AVM_IMG_WIDTH",              m_AVM_IMG_WIDTH);
    nh.getParam("AVM_IMG_HEIGHT",             m_AVM_IMG_HEIGHT);
    nh.getParam("REAL_OCCUPANCY_SIZE_X",      m_REAL_OCCUPANCY_SIZE_X);
    nh.getParam("REAL_OCCUPANCY_SIZE_Y",      m_REAL_OCCUPANCY_SIZE_Y);
    nh.getParam("SLAM_OCCUPANCY_SIZE_X",      m_SLAM_OCCUPANCY_SIZE_X);
    nh.getParam("SLAM_OCCUPANCY_SIZE_Y",      m_SLAM_OCCUPANCY_SIZE_Y);

    ISAM2Params parameters;
    parameters.relinearizeThreshold = 0.01;
    parameters.relinearizeSkip = 1;
    isam = new ISAM2(parameters);
    initNoises();

	ros::Subscriber subLaserCloudFullRes = nh.subscribe<sensor_msgs::PointCloud2>("/currentFeature", 100, laserCloudFullResHandler);
	ros::Subscriber subLaserOdometry = nh.subscribe<nav_msgs::Odometry>("/currentPose", 100, laserOdometryHandler);
    ros::Subscriber sub_parkingStart = nh.subscribe("/parkingPath", 10, CallbackParkingStart);
    ros::Subscriber sub_rePlanning = nh.subscribe("replanning", 1, CallbackReplanning);


	pubOdomAftPGO = nh.advertise<nav_msgs::Odometry>("/aft_pgo_odom", 100);
	pubPathAftPGO = nh.advertise<nav_msgs::Path>("/aft_pgo_path", 100);
	pubMapAftPGO = nh.advertise<sensor_msgs::PointCloud2>("/aft_pgo_map", 100);
    recentpubMapAftPGO = nh.advertise<sensor_msgs::PointCloud2>("/recent_aft_pgo_map", 1);

    pubOdomAftPGO_Array = nh.advertise<std_msgs::Float32MultiArray>("/LocalizationData", 1);
    // pubOdomAftPGO_Array = nh.advertise<std_msgs::Float32MultiArray>("/YWLocalizationData", 1);

	pubLoopScanLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_scan_local", 100);
    pubLoopScan_aft_Local = nh.advertise<sensor_msgs::PointCloud2>("/aft_loop_scan_local", 100);
	pubLoopSubmapLocal = nh.advertise<sensor_msgs::PointCloud2>("/loop_submap_local", 100);

    
	std::thread posegraph_slam {process_pg}; // pose graph construction
    std::thread lc_detection {process_lcd}; // loop closure detection 
	std::thread icp_calculation {process_icp}; // loop constraint calculation via icp 
	// std::thread isam_update {process_isam}; // if you want to call less isam2 run (for saving redundant computations and no real-time visulization is required), uncommment this and comment all the above runisam2opt when node is added. 
    std::thread pub_loc {process_pub_loc};
	// std::thread viz_map {process_viz_map}; // visualization - map (low frequency because it is heavy)
	// std::thread viz_path {process_viz_path}; // visualization - path (high frequency)

 	ros::spin();

	return 0;
}
