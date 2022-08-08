#include <iostream>
#include <algorithm>
#include <cmath>
#include <fstream>
#include <dirent.h>
#include <string.h>
#include <ctime>
#include <random>

#include <ros/ros.h>    
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <list>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/program_options.hpp>

#include <ompl/base/StateSpaceTypes.h>
#include <ompl/base/spaces/ReedsSheppStateSpace.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/objectives/PathLengthOptimizationObjective.h>
#include <ompl/base/objectives/StateCostIntegralObjective.h>
#include <ompl/base/objectives/MaximizeMinClearanceObjective.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/MotionValidator.h>
// #include 

#include <ompl/geometric/SimpleSetup.h>
#include <ompl/geometric/planners/rrt/InformedRRTstar.h>
#include <ompl/geometric/planners/rrt/RRTConnect.h>
#include <ompl/geometric/planners/rrt/RRTstar.h>
#include <ompl/geometric/PathSimplifier.h>
#include <ompl/tools/benchmark/Benchmark.h>
#include <ompl/tools/config/MagicConstants.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <boost/timer/timer.hpp>
#include <boost/thread/thread.hpp>

// #include <cv_bridge/cv_bridge.h>

#include "../inc/HybridCurvatureStateSpace.h"
#include "../inc/GeneralizedCurvatureStateSpace.h"
#include "../inc/RRTstar.h" //add, for visualizing RRTstar tree
#include "../inc/biRRTstar.h"
#include "../inc/RRTstar_target.h"
#include "../inc/RRTtarget.h" // add for IV
#include "../inc/target_tree.h"

using namespace std;
using namespace Eigen;
// using namespace cv;

namespace ob = ompl::base;
namespace og = ompl::geometric;

std_msgs::Header m_header;

int m_label_path_size = 0;

#define meter2pixel 1.0
#define KMpH2MpS 0.277777778
#define Mps2KMpH 3.6
#define DISTANCE(x1, y1, x2, y2) sqrt((x1 - x2)*(x1 - x2) +(y1 - y2)*(y1 - y2))
#define USING_REVERSE true // if true: rrt planning generates tree from GOAL.
#define READ_PATH false

// sample collision check parameter
#define SAFEREGION       1.975/2.0*1.19    // pure width of IONIQ / 2 + 2*extra_collision_range
#define COLLISION_RADIUS 1.9    //1.44 Scenario2 --> // Scenario2 --> 1.3 // orig: 1.3345
#define POINTSAFEREGION1 0.255*1.0 //0.255
#define POINTSAFEREGION2 0.260*1.0 //0.26
#define   SIDESAFEREGION 0.200*1.0 //0.20

#define MANUAL_GOAL false
#define PLANNING_CNT 3              // jw add
#define VISUALIZATION       false   // ms add
#define RE_COLLISION_CHK    true    // jw add

#define USING_BIRRTstar     true    // ms add
#define USING_TARGET_TREE   true  // ms add

#define M_SPACE_WIDTH 2.7  // 2.63
#define M_SPACE_LENGTH 7.3  //2.68,  5.2

////////////////////////////////////////// INITIALIZATION //////////////////////////////////////////////
bool m_using_hybrid_curvature;
double m_turning_radius;
double m_max_curvature_derivative;
double m_gamma;
double m_target_bias_ratio;
double m_goal_bias;
double m_planning_time;
double m_center2rear = 0.0;   // jw add
// int m_test_iter;
double m_extend_length = 0.0;
double m_offset = 0.1; 


struct CARPOSE {
    double x,y,th,vel;
};CARPOSE m_car;

og::SimpleSetup* ss;
og::PathSimplifier * path_simplifier;
ob::StateSpacePtr state_space, rs_state_space;
ob::MotionValidatorPtr state_space_mv;

// vector<Vector3d> m_vPath_h;
vector<Vector3d> m_vPath_g[PLANNING_CNT];   // jw add
bool m_vPath_bool[PLANNING_CNT] = {false, }; // jw add

vector<Vector3d> m_refPath;
vector<Vector3d> m_targetPath;

int m_trial_count = 0;
bool m_hc_valid = false;

typedef ob::SE2StateSpace::StateType STATETYPE;
typedef ob::SE2StateSpace STATESPACE;

bool m_validStart = false;
bool m_validGoal = false;
bool m_planning_start = false;
bool m_path_flag = false;
bool m_is_collision = false;
int m_success_count = 0;
double START_G[3] = {0.0, 0.0, (0)};
double GOAL_G[3] = {0.0, 0.0, (0)};
double GOAL_ORI_G[3] = {0.0, 0.0, (0)};
int GOAL_EXTEND_SIGN = -1; // ms add, -1 for reverse-parking and +1 for forward-parking
bool m_path_pub_flag = false;

ob::RealVectorBounds BOUNDS(2);// Contains obstacles' INFO for planning
vector<vector<VectorXd>> g_map;//VecotrXd (in Eigen) which has arbitrary size
ob::PlannerStatus m_g_solved;

pcl::PointCloud<pcl::PointXYZ>::Ptr g_pTree;
pcl::KdTreeFLANN<pcl::PointXYZ> g_kdTree;

ros::Publisher Pose_Goal, Pose_Start;
ros::Publisher Pub_RS_field, Pub_Sample_field;
ros::Publisher Pub_Path, Pub_Path_forward, Pub_Path_backward;
ros::Publisher Pub_obstacleLocal;
ros::Publisher Pub_completed;
ros::Publisher Pub_parkingStart;
ros::Publisher Pub_collision_check, Pub_collision_check_car; // add
ros::Publisher Pub_TargetTree;  // ms add

ros::Subscriber Sub_heightMap;
ros::Subscriber Sub_init;
ros::Subscriber Sub_localization;
ros::Subscriber Sub_parkingGoal;
ros::Subscriber Sub_rePlanning;
ros::Subscriber Sub_manualPlanningFlag; // add
ros::Subscriber Sub_occMap; // add for debugging

typedef pcl::PointXYZI VPoint;
typedef pcl::PointCloud<VPoint> VPointCloud;
VPointCloud m_obstaclePtCloudLocal;
double map_range = 0.0, m_gridResol = 0.0, m_gridDim = 0.0;
int modx1 = 0, mody1 =0, modx2 = 0, mody2 =0, modx3 = 0, mody3 =0, modx4 = 0, mody4 =0; // add: for removing the area of the parking spot

// about Target tree ms add
target_tree_generator *m_target_tree_gen; // it generates a target tree
vector<Vector2d> m_bias_sample_dice;// it contains probabilities of sampling of each node in the target tree.
int m_best_target_node;
double m_min_target_path_cost = 999.0;
double m_proper_len = 0.0;
bool m_score_computed = false;
int m_goal_node = 0;

std::ofstream path_log;
std::ofstream myfile_rs, myfile_hc; 
std::ofstream cuspNlength_log;
std::ifstream read_path, read_path2;
std::ifstream label_path;   // label path를 읽어옴.
int m_total_path_waypt;

geometry_msgs::PoseArray collision_edge;    //add

geometry_msgs::PoseArray g_arraySample_pose, r_arraySample_pose;

// add
void GetRelCoordsFromCar(double ori_x, double ori_y, double ori_th, double &rel_x, double &rel_y, double &rel_th) {
    double Len = DISTANCE(m_car.x, m_car.y, ori_x, ori_y);
    double alpha = atan2(ori_y - m_car.y, ori_x - m_car.x);
    rel_x = Len * cos(alpha - m_car.th);
    rel_y = Len * sin(alpha - m_car.th);
    rel_th = ori_th  - m_car.th;
}

void Local2Global(double Lx, double Ly, double &gX, double &gY) {
    gX = m_car.x + (Lx * cos(m_car.th) - Ly * sin(m_car.th));
    gY = m_car.y + (Lx * sin(m_car.th) + Ly * cos(m_car.th));
}

void arr2real(int recvX, int recvY, double& outX, double& outY) {
    outX = recvX * m_gridResol - (m_gridResol*m_gridDim - m_gridResol) / 2.0;
    outY = recvY * m_gridResol - (m_gridResol*m_gridDim - m_gridResol) / 2.0;
}

void real2arr(double recvX, double recvY, int& outX, int& outY) {
    outX = (m_gridDim / 2.0) + recvX / m_gridResol;
    outY = (m_gridDim / 2.0) + recvY / m_gridResol;
}


// add ------------------------------------------------------------------ for extending the area of the parking spot
void coord(double dx, double dy, double xx, double yy, double thh, int& x_, int& y_) {
    double modx = cos(M_PI/2+thh)*dx - sin(M_PI/2+thh)*dy + xx;
    double mody = sin(M_PI/2+thh)*dx + cos(M_PI/2+thh)*dy + yy;
    real2arr(modx, mody, x_, y_);
}

int withinpoint(int x, int y) {
    typedef boost::geometry::model::d2::point_xy<int> point_type;
    typedef boost::geometry::model::polygon<point_type> polygon_type;

    polygon_type poly;
    poly.outer().assign({
        point_type {modx1, mody1}, point_type {modx2, mody2},
        point_type {modx3, mody3}, point_type {modx4, mody4},
        point_type {modx1, mody1}
    });

    point_type p(x, y);

    return boost::geometry::within(p, poly);
}
// -----------------------------------------------------------------------

void coord(double dx, double dy, double xx, double yy, double thh, double modx, double mody, int& x_, int& y_) {
    modx = cos(M_PI/2+thh)*dx - sin(M_PI/2+thh)*dy + xx;
    mody = sin(M_PI/2+thh)*dx + cos(M_PI/2+thh)*dy + yy;
    real2arr(modx, mody, x_, y_);
}

vector<double> a_linspace(double a, double b, int num)
{
    // create a vector of length num
    vector<double> v(num);
    double tmp = 0.0;
    // now assign the values to the vector
    for (int i = 0; i < num; i++) {
        v[i] = a + i * ( (b - a) / (double)num );
    }
    return v;
}

void collision_range(double x, double y, double theta, int idx) {
    /////////////////////////////////////////////////////////////////////
    collision_edge.header = m_header;
    std::vector<double> angle_set = a_linspace(0.0, 2.0 * M_PI, 27);
    std::vector<double> angle_set_small = a_linspace(0.0, 2.0 * M_PI, 7);   // jw add
    geometry_msgs::PoseStamped _poseStamped;
    // Large Circle
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + SAFEREGION * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + SAFEREGION * sin(angle_set[i]);    //y_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    // Large Circle1
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + COLLISION_RADIUS*cos(theta) + SAFEREGION * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + COLLISION_RADIUS*sin(theta) + SAFEREGION * sin(angle_set[i]);    //y_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    // Large Circle2
    for(int i = 0; i < angle_set.size(); i++) {
        _poseStamped.pose.position.x = x + 2.0 * COLLISION_RADIUS*cos(theta) + SAFEREGION * cos(angle_set[i]);    //x_g;
        _poseStamped.pose.position.y = y + 2.0 * COLLISION_RADIUS*sin(theta) + SAFEREGION * sin(angle_set[i]);    //y_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set_small.size(); i++) {
        _poseStamped.pose.position.x = x - 0.719*cos(theta) - 0.9175*sin(theta) + POINTSAFEREGION1 * cos(angle_set_small[i]);    //x_g;
        _poseStamped.pose.position.y = y - 0.719*sin(theta) + 0.9175*cos(theta) + POINTSAFEREGION1 * sin(angle_set_small[i]);    //y_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set_small.size(); i++) {
        _poseStamped.pose.position.x = x - 0.719*cos(theta) + 0.9175*sin(theta) + POINTSAFEREGION1 * cos(angle_set_small[i]);    //x_g;
        _poseStamped.pose.position.y = y - 0.719*sin(theta) - 0.9175*cos(theta) + POINTSAFEREGION1 * sin(angle_set_small[i]);    //y_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set_small.size(); i++) {
        _poseStamped.pose.position.x = x + 2*COLLISION_RADIUS*cos(theta) + 0.735*cos(theta) - 0.9175*sin(theta) + POINTSAFEREGION2 * cos(angle_set_small[i]);    //x_g;
        _poseStamped.pose.position.y = y + 2*COLLISION_RADIUS*sin(theta) + 0.735*sin(theta) + 0.9175*cos(theta) + POINTSAFEREGION2 * sin(angle_set_small[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set_small.size(); i++) {
        _poseStamped.pose.position.x = x + 2*COLLISION_RADIUS*cos(theta) + 0.735*cos(theta) + 0.9175*sin(theta) + POINTSAFEREGION2 * cos(angle_set_small[i]);    //x_g;
        _poseStamped.pose.position.y = y + 2*COLLISION_RADIUS*sin(theta) + 0.735*sin(theta) - 0.9175*cos(theta) + POINTSAFEREGION2 * sin(angle_set_small[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set_small.size(); i++) {
        _poseStamped.pose.position.x = x + COLLISION_RADIUS*cos(theta) - 0.6593*cos(theta) - 0.8587*sin(theta) + SIDESAFEREGION * cos(angle_set_small[i]);    //x_g;
        _poseStamped.pose.position.y = y + COLLISION_RADIUS*sin(theta) - 0.6593*sin(theta) + 0.8587*cos(theta) + SIDESAFEREGION * sin(angle_set_small[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set_small.size(); i++) {
        _poseStamped.pose.position.x = x + COLLISION_RADIUS*cos(theta) - 0.6593*cos(theta) + 0.8587*sin(theta) + SIDESAFEREGION * cos(angle_set_small[i]);    //x_g;
        _poseStamped.pose.position.y = y + COLLISION_RADIUS*sin(theta) - 0.6593*sin(theta) - 0.8587*cos(theta) + SIDESAFEREGION * sin(angle_set_small[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set_small.size(); i++) {
        _poseStamped.pose.position.x = x + COLLISION_RADIUS*cos(theta) + 0.6593*cos(theta) - 0.8587*sin(theta) + SIDESAFEREGION * cos(angle_set_small[i]);    //x_g;
        _poseStamped.pose.position.y = y + COLLISION_RADIUS*sin(theta) + 0.6593*sin(theta) + 0.8587*cos(theta) + SIDESAFEREGION * sin(angle_set_small[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    for(int i = 0; i < angle_set_small.size(); i++) {
        _poseStamped.pose.position.x = x + COLLISION_RADIUS*cos(theta) + 0.6593*cos(theta) + 0.8587*sin(theta) + SIDESAFEREGION * cos(angle_set_small[i]);    //x_g;
        _poseStamped.pose.position.y = y + COLLISION_RADIUS*sin(theta) + 0.6593*sin(theta) - 0.8587*cos(theta) + SIDESAFEREGION * sin(angle_set_small[i]);    //x_g;
        _poseStamped.pose.position.z = 0.5;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(0.0);
        _poseStamped.header = m_header;
        _poseStamped.pose.orientation = odom_quat;
        collision_edge.poses.push_back(_poseStamped.pose);
    }
    if (idx == 0)
        Pub_collision_check.publish(collision_edge);
    else
        Pub_collision_check_car.publish(collision_edge);
    collision_edge.poses.clear();
    /////////////////////////////////////////////////////////////////////
}

void pushSample(ob::State* pose, bool rs_sample) {
    STATETYPE *pose_ = pose->as<STATETYPE>();

    geometry_msgs::PoseStamped poseStamped, dposeStamped;
    poseStamped.pose.position.x = pose_->getX();    //x_g;
    poseStamped.pose.position.y = pose_->getY();    //y_g;
    poseStamped.pose.position.z = 0.1;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(pose_->getYaw());
    poseStamped.header = m_header;
    poseStamped.pose.orientation = odom_quat;
    g_arraySample_pose.header = r_arraySample_pose.header = m_header;
    
    // g_arraySample_pose.poses.push_back(poseStamped.pose);
    if (!rs_sample)    g_arraySample_pose.poses.push_back(poseStamped.pose);
    else               r_arraySample_pose.poses.push_back(poseStamped.pose);
}

void clearSample() {
    g_arraySample_pose.poses.clear();
    r_arraySample_pose.poses.clear();
}

// --------------- target tree
// ms add
std::vector<double> biasSample_(ob::State* pose, double bias_ratio) {// it only works in Target Tree
    STATETYPE *bias_pose = pose->as<STATETYPE>();
    double dice = (double)rand() / RAND_MAX;
    if (dice < bias_ratio && m_bias_sample_dice.size() != 0) {
        int u;
        u = rand() % (m_bias_sample_dice.size() - 500) + 500;
        int node_ind = m_bias_sample_dice[u][0];
        double dist_to_goal = 0.0;
        bias_pose->setX(m_target_tree_gen->target_tree_[node_ind]->state->x);
        bias_pose->setY(m_target_tree_gen->target_tree_[node_ind]->state->y);
        bias_pose->setYaw(m_target_tree_gen->target_tree_[node_ind]->state->theta);
        dist_to_goal = m_target_tree_gen->target_tree_[node_ind]->distance_to_goal;
        std::vector<double> output{1.0, (double)u, (double)node_ind, dist_to_goal};
        return output;  // it notices the sample is extracted at goal-tree, [true / false, ind, ind_i, ind_j, distance_to_goal]
    }
    std::vector<double> output{-1.0, -1.0, -1.0, -1.0, 999.9};
    return output;
}

// it checks newly added target_path removed. Also, it updates min-target-path-cost
void remove_bias(int ind) {
    m_bias_sample_dice.erase(m_bias_sample_dice.begin() + ind);
}

void bestUpdate(int node_ind) {
    m_best_target_node = node_ind;
}

void get_min_target_path() {// build a target path, which is no collision and has minimum length.
    vector<Vector3d> vPath;
    vPath.clear();   
    int node_ind = m_best_target_node;
    tree::Node *target_node = m_target_tree_gen->target_tree_[node_ind];// it is a current target node
    while (target_node != nullptr) {
        vPath.push_back(Vector3d(target_node->state->x, target_node->state->y, target_node->state->theta));
        target_node = target_node->parent;//  a path to the goal pose (from the target-node)
    }
    m_targetPath = vPath;
}

//--------------------------------------------------------------

class ClearanceObjective : public ob::StateCostIntegralObjective
{
public:
    ClearanceObjective(const ob::SpaceInformationPtr& si) :
        ob::StateCostIntegralObjective(si, true)
    {
    }

    // Our requirement is to maximize path clearance from obstacles,
    // but we want to represent the objective as a path cost
    // minimization. Therefore, we set each state's cost to be the
    // reciprocal of its clearance, so that as state clearance
    // increases, the state cost decreases.
    ob::Cost stateCost(const ob::State* s) const override
    {
        return ob::Cost(1 / (si_->getStateValidityChecker()->clearance(s) +
            std::numeric_limits<double>::min()));
    }
};

ob::OptimizationObjectivePtr getClearanceObjective(const ob::SpaceInformationPtr& si)
{
    return std::make_shared<ClearanceObjective>(si);
}

ob::OptimizationObjectivePtr getBalancedObjective(const ob::SpaceInformationPtr& si)
{
    auto lengthObj(std::make_shared<ob::PathLengthOptimizationObjective>(si));
    auto opt(std::make_shared<ob::MultiOptimizationObjective>(si));
    opt->addObjective(lengthObj, 1.0);
    // auto clearObj(std::make_shared<ClearanceObjective>(si));//FIXED
    // opt->addObjective(clearObj, 2.0);

    return ob::OptimizationObjectivePtr(opt);
}

int SearchNodeByRadius(pcl::PointXYZ searchPoint, float radius)
{
    std::vector<int> pointIdxRadiusSearch;
    std::vector<float> pointRadiusSquaredDistance;
    std::vector<pcl::PointXYZ> pvNode;

    g_kdTree.radiusSearch(searchPoint, radius, pointIdxRadiusSearch, pointRadiusSquaredDistance );
    return pointIdxRadiusSearch.size();
}

bool isFreeSpace(float x, float y, double yaw)
{
    int obsCnt = SearchNodeByRadius(pcl::PointXYZ(x, y, 0),SAFEREGION);//.size();
    //--------------------------------------------------------------------------------------------------------
    //each point 
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x - 0.719*cos(yaw) - 0.9175*sin(yaw), y - 0.719*sin(yaw) + 0.9175*cos(yaw), 0),POINTSAFEREGION1);//.size();
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x - 0.719*cos(yaw) + 0.9175*sin(yaw), y - 0.719*sin(yaw) - 0.9175*cos(yaw), 0),POINTSAFEREGION1);//.size();
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + 2*COLLISION_RADIUS*cos(yaw) + 0.735*cos(yaw) - 0.9175*sin(yaw), y + 2*COLLISION_RADIUS*sin(yaw) + 0.735*sin(yaw) + 0.9175*cos(yaw), 0),POINTSAFEREGION2);//.size();
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + 2*COLLISION_RADIUS*cos(yaw) + 0.735*cos(yaw) + 0.9175*sin(yaw), y + 2*COLLISION_RADIUS*sin(yaw) + 0.735*sin(yaw) - 0.9175*cos(yaw), 0),POINTSAFEREGION2);//.size();
    //empty side
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + COLLISION_RADIUS*cos(yaw) - 0.6593*cos(yaw) - 0.8587*sin(yaw), y + COLLISION_RADIUS*sin(yaw) - 0.6593*sin(yaw) + 0.8587*cos(yaw), 0),SIDESAFEREGION);//.size();
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + COLLISION_RADIUS*cos(yaw) - 0.6593*cos(yaw) + 0.8587*sin(yaw), y + COLLISION_RADIUS*sin(yaw) - 0.6593*sin(yaw) - 0.8587*cos(yaw), 0),SIDESAFEREGION);//.size();
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + COLLISION_RADIUS*cos(yaw) + 0.6593*cos(yaw) - 0.8587*sin(yaw), y + COLLISION_RADIUS*sin(yaw) + 0.6593*sin(yaw) + 0.8587*cos(yaw), 0),SIDESAFEREGION);//.size();
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + COLLISION_RADIUS*cos(yaw) + 0.6593*cos(yaw) + 0.8587*sin(yaw), y + COLLISION_RADIUS*sin(yaw) + 0.6593*sin(yaw) - 0.8587*cos(yaw), 0),SIDESAFEREGION);//.size();
    //--------------------------------------------------------------------------------------------------------

    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + COLLISION_RADIUS*cos(yaw), y + COLLISION_RADIUS*sin(yaw), 0),SAFEREGION);//.size();
    obsCnt += SearchNodeByRadius(pcl::PointXYZ(x + 2*COLLISION_RADIUS*cos(yaw), y + 2*COLLISION_RADIUS*sin(yaw), 0),SAFEREGION);//.size();
    
    if( obsCnt > 0 )
        return false;
    else
        return true;
}

bool isStateValid(const ob::SpaceInformation *si, const vector<vector<VectorXd> >& map, const ob::State *state)
{
    const STATETYPE *s = state->as<STATETYPE>();
    return si->satisfiesBounds(s) && isFreeSpace(s->getX(),s->getY(),s->getYaw());
}

double three_pt_curvature(double x1, double y1, double x2, double y2, double x3, double y3) {
    double fAreaOfTriangle = fabs((x1 * (y2 - y3) +\
                                    x2 * (y3 - y1) +\
                                    x3 * (y1 - y2)) / 2);
    double fDist12 = sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
    double fDist23 = sqrt((x2 - x3) * (x2 - x3) + (y2 - y3) * (y2 - y3));
    double fDist13 = sqrt((x3 - x1) * (x3 - x1) + (y3 - y1) * (y3 - y1));
    double fKappa = 4 * fAreaOfTriangle / (fDist12 * fDist23 * fDist13);

    //cross product
    double first_vec_x = x2 - x1;
    double first_vec_y = y2 - y1;
    double second_vec_x = x3 - x2;
    double second_vec_y = y3 - y2;
    double cross_product = first_vec_x * second_vec_y - second_vec_x * first_vec_y;
    int sign_ = (cross_product>=0) ? 1 : -1;

    if (isnan(fKappa) != 0)
        fKappa = 0.0;

    return sign_ * fKappa;    
}

int m_pathIdx = 99999, m_path_len = 999999, m_minCnt = 99;
void min_switching(int cnt) {
    if (m_vPath_bool[cnt]) {
        int pathIdx = 0;
        double vec_diff = 0.0, vec_1 = 0.0, vec_2 = 0.0;
        for(int i = 1; i < m_vPath_g[cnt].size()-1; i++) {
            vec_1 = atan2(m_vPath_g[cnt][i][1] - m_vPath_g[cnt][i-1][1], m_vPath_g[cnt][i][0] - m_vPath_g[cnt][i-1][0]);
            vec_2 = atan2(m_vPath_g[cnt][i+1][1] - m_vPath_g[cnt][i][1], m_vPath_g[cnt][i+1][0] - m_vPath_g[cnt][i][0]);
            if (cos(vec_1) * cos(vec_2) + sin(vec_1) * sin(vec_2) < 0.0)  // dot product
                pathIdx++;
        }
        cout <<"# "<< cnt << " ----- RRT Switching Cnt: " << pathIdx << ", Len: " << m_vPath_g[cnt].size() << endl;

        if (pathIdx <= m_pathIdx) {
            if (pathIdx == m_pathIdx) 
                if (m_vPath_g[cnt].size() > m_path_len) 
                    return;
                
            m_path_len = m_vPath_g[cnt].size();
            m_pathIdx = pathIdx;
            m_minCnt = cnt;
        }
    }
}

void Publish_topic() {
    double cum_length = 0.0;
    double dt = 100;
    nav_msgs::Path rrt_path_msg;
    rrt_path_msg.header = m_header;
    
    if (!READ_PATH) {
        rrt_path_msg.poses.resize(m_vPath_g[m_minCnt].size() + ceil(m_extend_length*dt));
        int rrt_path_size = m_vPath_g[m_minCnt].size();
        int count_ = 0;        
        if (!USING_TARGET_TREE && !USING_BIRRTstar && USING_REVERSE) {
            for(int i = m_vPath_g[m_minCnt].size()-1; i >= 0; i--) {
                rrt_path_msg.poses[m_vPath_g[m_minCnt].size()-1 - i].pose.position.x = m_vPath_g[m_minCnt][i][0];
                rrt_path_msg.poses[m_vPath_g[m_minCnt].size()-1 - i].pose.position.y = m_vPath_g[m_minCnt][i][1];
                rrt_path_msg.poses[m_vPath_g[m_minCnt].size()-1 - i].pose.position.z = 0.6;
                rrt_path_msg.poses[m_vPath_g[m_minCnt].size()-1 - i].pose.orientation = tf::createQuaternionMsgFromYaw(m_vPath_g[m_minCnt][i][2]);// Get a radian!m_vPath[i][2];//yaw

                // myfile_rs << std::fixed << std::setprecision(11) << m_vPath_h[i][0] << "," << m_vPath_h[i][1] << "," << m_vPath_h[i][2] << endl;
                // path_log << std::fixed << std::setprecision(11) << 
                //             m_vPath_g[m_minCnt][i][0] << "," << m_vPath_g[m_minCnt][i][1] << "," << m_vPath_g[m_minCnt][i][2] << endl;
                count_++;
            }       
        }
        else {// not reverse
            for(int i = 0; i < m_vPath_g[m_minCnt].size(); i++) {
                rrt_path_msg.poses[i].pose.position.x = m_vPath_g[m_minCnt][i][0];
                rrt_path_msg.poses[i].pose.position.y = m_vPath_g[m_minCnt][i][1];
                rrt_path_msg.poses[i].pose.position.z = 0.6;
                rrt_path_msg.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(m_vPath_g[m_minCnt][i][2]);// Get a radian!m_vPath[i][2];//yaw

                // myfile_rs << std::fixed << std::setprecision(11) << m_vPath_g[m_minCnt][i][0] << "," << m_vPath_g[m_minCnt][i][1] << "," << m_vPath_g[m_minCnt][i][2] << endl;
                // path_log << std::fixed << std::setprecision(11) << 
                //             m_vPath_g[m_minCnt][i][0] << "," << m_vPath_g[m_minCnt][i][1] << "," << m_vPath_g[m_minCnt][i][2] << endl;
                count_++;
                if (i != 0)
                    cum_length += DISTANCE(m_vPath_g[m_minCnt][i-1][0], m_vPath_g[m_minCnt][i-1][1], 
                                            m_vPath_g[m_minCnt][i][0], m_vPath_g[m_minCnt][i][1]);
            }
        }
        // double switching_th = atan2((rrt_path_msg.poses[rrt_path_size - 1].pose.position.y - rrt_path_msg.poses[rrt_path_size - 2].pose.position.y),
                                        // (rrt_path_msg.poses[rrt_path_size - 1].pose.position.x - rrt_path_msg.poses[rrt_path_size - 2].pose.position.x));
        double switching_th = GOAL_ORI_G[2] + M_PI;// 혹은 마지막 path의 orientation 사용해도 됨.rrt_path_msg.poses[rrt_path_size - 1].pose.orientation.z                                        
        for(int j = 0; j < m_extend_length*dt; j++) {
            rrt_path_msg.poses[rrt_path_size + j].pose.position.x = rrt_path_msg.poses[rrt_path_size - 1].pose.position.x + (j+1)*cos(switching_th)/dt;
            rrt_path_msg.poses[rrt_path_size + j].pose.position.y = rrt_path_msg.poses[rrt_path_size - 1].pose.position.y + (j+1)*sin(switching_th)/dt;
            rrt_path_msg.poses[rrt_path_size + j].pose.position.z = 0.6;
            // myfile_rs << std::fixed << std::setprecision(11) << rrt_path_msg.poses[rrt_path_size - 1].pose.position.x + (j+1)*cos(switching_th)/dt << "," \
            //                                                 << rrt_path_msg.poses[rrt_path_size - 1].pose.position.y + (j+1)*sin(switching_th)/dt << "," \
            //                                                 << switching_th + M_PI << endl;
            // path_log << std::fixed << std::setprecision(11) << rrt_path_msg.poses[rrt_path_size - 1].pose.position.x + (j+1)*cos(switching_th)/dt << ", " \
            //                                                 << rrt_path_msg.poses[rrt_path_size - 1].pose.position.y + (j+1)*sin(switching_th)/dt << "," \
            //                                                 << switching_th + M_PI << endl;
        }
        // path_log.close();
        // myfile_rs.close();
    } 
    else {
        rrt_path_msg.poses.resize(m_total_path_waypt);
        string line;
        vector<string> vec;
        char c;// eat the comma
        double x, y, yaw;
        double prev_x, prev_y;
        int ii = 0;
        // while(getline(read_path, line)) {
        //     std::istringstream iss(line);
        //     iss >> x >> c >> y >> c >> yaw;
        //     cout << "x: " << x << "y: " << y << "yaw: " << yaw << endl;
        //     rrt_path_msg.poses[ii].pose.position.x = x;
        //     rrt_path_msg.poses[ii].pose.position.y = y;
        //     rrt_path_msg.poses[ii].pose.position.z = 0.0;
        //     ii++;
        //     cum_length += DISTANCE(prev_x, prev_y, x, y);
        //     prev_x = x; prev_y = y;
        // }
        // read_path.close();
    }
    cout << "path length: " << cum_length << endl;
    cout << "PUBLISH!" << endl;
    Pub_Path.publish(rrt_path_msg);
    m_path_pub_flag = true;
}

void UpdateGlobalPathData(int cnt) { 
    if (m_g_solved) {
        // Construct 'Path instance' for a given space information
        og::PathGeometric path = ss->getSolutionPath();// Get optimal path which is solved using the declared PLANNER within that defined time
        
        path.interpolate((int)(path.length()/0.01));// Insert a number of states in a path so that the path is made up of exactly count states
        // path.interpolate((int)(path.length()/0.015));// Insert a number of states in a path so that the path is made up of exactly count states
        vector<Vector3d> vPath;
        vPath.clear();
        int collision_count = 0;
        for(int i = 0;i < path.getStateCount(); i++) {// Get the number of states (way-points) that make up this path.
            const STATETYPE *s = path.getState(i)->as<STATETYPE>();
            double x = s->getX(), y = s->getY(), yaw = s->getYaw();
            if (RE_COLLISION_CHK && !isFreeSpace(x, y, yaw)) {
                m_is_collision = true;
                // std::cout << "COLLISION!" << std::endl;
                break;
            }
            vPath.push_back(Vector3d(x,y,yaw));
        }
        // m_vPath_h = vPath;
        m_vPath_g[cnt] = vPath;
    }
    else
        OMPL_ERROR("---------- Fail the path planning ----------");
}


void compute_score(double root_x, double root_y, double root_th) {
    if (m_score_computed)
        return;
    
    m_target_tree_gen->set_interval(0.01);
    int u = 0;
    int max_u = 0;
    double max_score;
    vector<double> straight_range = linspace(0.1, 3.0, 25, true);// if parallel 1.0
    clock_t start_time = clock();

    m_target_tree_gen->build_target_tree(root_x, root_y, root_th, 0.0, 10.0, 25);// scenario s1c45 [O]
    max_score = m_target_tree_gen->score_;
    for(const auto &line_length: straight_range) {
        m_target_tree_gen->build_target_tree(root_x, root_y, root_th, line_length, 10.0, 25);// scenario s1c45 [O]
        if (max_score < m_target_tree_gen->score_) {
            max_score = m_target_tree_gen->score_;
            max_u  = u;
        }
        u += 1;
    }
    m_proper_len = straight_range[max_u];
    m_target_tree_gen->clear_tree();
    m_score_computed = true;
}

void init_cctarget_tree(double root_x, double root_y, double root_th) {
    ///////////////////////////// Target Tree ////////////////////////////////////
    // init
    // builds target tree
    m_bias_sample_dice.clear();
    m_target_tree_gen->clear_tree();
    m_best_target_node = 0;
    m_targetPath.clear();

    m_target_tree_gen->set_interval(0.01);
    m_target_tree_gen->build_target_tree(root_x, root_y, root_th, m_proper_len, 10.0, 25);
    // m_target_tree_gen->build_target_tree_parallel(root_x, root_y, root_th, m_proper_len, 10.0, 25);// [O]
    geometry_msgs::PoseArray goal_tree;
    goal_tree.header = m_header;

    double sum_of_chance = 0.0;
    for(const auto &tree_node : m_target_tree_gen->target_tree_)
        sum_of_chance += 1.0;

    // provide probs. of each target-tree node.
    int kk = 0;
    double cum_chance = 0.0;
    
    vector<tree::Node *> hctree_ = m_target_tree_gen->target_tree_;
    for(int jj = 0; jj < hctree_.size(); jj++) { 
        m_bias_sample_dice.push_back(Vector2d(jj, (double)kk / sum_of_chance));
        kk++;
        if (VISUALIZATION) {
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.pose.position.x = hctree_[jj]->state->x;
            poseStamped.pose.position.y = hctree_[jj]->state->y;
            poseStamped.pose.position.z = 0.0;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(hctree_[jj]->state->theta);
            poseStamped.header = m_header;
            poseStamped.pose.orientation = odom_quat;
            goal_tree.poses.push_back(poseStamped.pose);
        }
    }
    
    if (VISUALIZATION) {
        Pub_TargetTree.publish(goal_tree);
    }
}

void plan_init(double* start, double* goal)
{    
    ob::ScopedState<> ss_start(state_space), ss_goal(state_space);// Declare ss_start and ss_goal into 'state space 'pointer' g_space'
    m_is_collision = false;  // ms add
    ////////////////////////////////
    // set the start and goal states
    for (int i = 0 ; i < 3 ; i++) {
        ss_start[i] = start[i];
        ss_goal[i] = goal[i];
    }

    // OMPL planner setting 
    ss->clear();
    ss->setStartAndGoalStates(ss_start, ss_goal);
    ob::SpaceInformationPtr si(ss->getSpaceInformation());
    
    if (m_using_hybrid_curvature)
        si->setMotionValidator(std::make_shared<HybridCurvatureMotionValidator>(si));
    
    ss->setStateValidityChecker(std::bind(&isStateValid, si.get(), g_map, std::placeholders::_1));
    // ss->setOptimizationObjective(getBalancedObjective(si));

    if (USING_TARGET_TREE && m_using_hybrid_curvature) {  // ms add
        cout << "**PLANNER: CCTARGET_TREE with RRTstar**" << endl;
        compute_score(goal[0], goal[1], goal[2]);
        init_cctarget_tree(goal[0], goal[1], goal[2]);

        og::RRTstar_target *RRTstar_target;
        RRTstar_target = new og::RRTstar_target(ss->getSpaceInformation(), biasSample_, remove_bias, bestUpdate, m_target_bias_ratio);
        ss->setPlanner(ob::PlannerPtr(RRTstar_target));        
    }
    else if (USING_BIRRTstar) {  // ms add
        cout << "**PLANNER: Bidirectional-RRTstar**" << endl;
        og::biRRTstar *biRRTstar;
        biRRTstar = new og::biRRTstar(ss->getSpaceInformation());
        ss->setPlanner(ob::PlannerPtr(biRRTstar));
    }
    else {
        cout << "**PLANNER: INFORMED-RRTstar***" << endl;
        ss->setPlanner(std::make_shared<og::InformedRRTstar>(si));
    }
    ss->setup();
    path_simplifier = new og::PathSimplifier(si);
}

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
bool plan_()
{
    // ss->planner_->setRange(5.0);
    m_g_solved = ss->solve(m_planning_time);
    // ss->simplifySolution();
    return m_g_solved == ob::PlannerStatus::EXACT_SOLUTION && ss->haveExactSolutionPath();
}


void plan() { // #1
    if (m_validStart && m_validGoal) {
        state_space->as<STATESPACE>()->setBounds(BOUNDS);// Get obstacles' INFO HC
        // ////////// setup orig
        for (int i = 0 ; i < PLANNING_CNT ; i++) {  // jw add
            if (!USING_TARGET_TREE && !USING_BIRRTstar && USING_REVERSE)
                plan_init(GOAL_G, START_G); // ms add
            else               
                plan_init(START_G, GOAL_G);

        // ////////// setup
        // for (int i = 0 ; i < PLANNING_CNT ; i++) {  // jw add
            cout << "--- plan_init OK" << endl;
            if (plan_()) {
                UpdateGlobalPathData(i);
                m_path_flag = m_vPath_bool[i] = m_is_collision ? false : true; // add
                // m_path_flag = m_vPath_bool[i] = true;
                if (USING_TARGET_TREE) {  // ms add
                    get_min_target_path();// build target path
                    m_vPath_g[i].pop_back();
                    m_vPath_g[i].insert(m_vPath_g[i].end(), m_targetPath.begin(), m_targetPath.end());
                }
            }
            else// A path is not found within planning time.
                m_vPath_bool[i] = false;
                // m_path_flag = false;
        }
        cout << "--- FINISH plan" << endl;
    }
}

void real_plan_loop() {
    for (int i = 0 ; i < PLANNING_CNT ; i++) {  // jw add
        m_vPath_g[i].clear();   // jw add
        m_vPath_bool[i] = false; // jw add
    }
    
    m_path_flag = false;
    cout << "Sub START " << START_G[0] << " " << START_G[1] << " " << START_G[2] << endl;
    cout << "Sub GOAL " << GOAL_G[0] << " " << GOAL_G[1] << " " << GOAL_G[2] << endl;
    
    GOAL_ORI_G[0] = GOAL_G[0];
    GOAL_ORI_G[1] = GOAL_G[1];
    GOAL_ORI_G[2] = GOAL_G[2];

    m_validStart = true;// which means the initial plan is excuted.

    // std::getchar();
    if (READ_PATH)
        Publish_topic();
    else {  // jw add
        while (!m_path_flag) {
            plan();
            for (int i = 0 ; i < PLANNING_CNT ; i++)
                min_switching(i);
            
            if (m_path_flag) {
                cout << "path is found!" << endl;
                system("killall -9 python3 &");                   // jw add
                
                system("rosrun parking_spot_detector_corner_point main_parking_space.py &");

                // system("roslaunch lego_loam run.launch &");                     //new LEGO LOAM
                
                // system("roslaunch phantomSLAM slam.launch &");                     //YW SLAM
                
                break;
            }
            m_path_len = m_pathIdx = 99999; m_minCnt = 99;
        }
    }
    m_is_collision = m_validGoal = false;
    cout << "Optimal path idx: "<< m_minCnt <<", cnt: "<< m_pathIdx << " Len: " << m_path_len<< endl;
    Publish_topic();

    // if (g_arraySample_pose.poses.size() > 0 || r_arraySample_pose.poses.size()) {
    //     cout << "relevant poses: " << g_arraySample_pose.poses.size() << endl;
    //     Pub_Sample_field.publish(g_arraySample_pose);
    //     cout << "rs state poses: " << r_arraySample_pose.poses.size() << endl;
    //     Pub_RS_field.publish(r_arraySample_pose);
    // }
    // g_arraySample_pose.poses.clear();
    // r_arraySample_pose.poses.clear();
}


void CanTopicProcess(const std_msgs::Float32MultiArray::ConstPtr& msg){
    // cout << "fuck" << endl;
    if (!m_validStart && m_validGoal && msg->data.at(0) < 0.3*KMpH2MpS) {// GOAL cand and velocity are subscribed! and velocity ~= 0 // MSK: add
        cout << "LET'S PLAN!" << endl;
        // m_planning_start = true;
        // for DR start at this step
        std_msgs::Float32MultiArray msg__;
        msg__.data.push_back(1);
        Pub_parkingStart.publish(msg__);
        //
        
        START_G[0] = 0.0; START_G[1] = 0.0; START_G[2] = 0.0;// Start point is the center of the rear axle
        geometry_msgs::PoseStamped poseStamped;
        poseStamped.pose.position.x = START_G[0];
        poseStamped.pose.position.y = START_G[1];
        poseStamped.pose.position.z = 0.0;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(START_G[2]);// Get a radian!
        poseStamped.header = m_header;
        poseStamped.pose.orientation = odom_quat;
        Pose_Start.publish(poseStamped); // add it visualizes at YOLO
        real_plan_loop();
    }
    m_validGoal = false;
}

void CallbackReplanning(const std_msgs::Bool::ConstPtr& msg) {
    // if (msg->data.at(0) == true) {
        cout << "REPLANNING! [Informed RRT*]" << endl;
        // Local2Global(START_G[0], START_G[1], START_G[0], START_G[1]);
        // START_G[0] = m_car.x;   START_G[1] = m_car.y;   START_G[2] = m_car.th;

        m_validGoal = true;
        m_validStart = false;
        m_path_flag = false;
        // real_plan_loop();
    // }
    // m_validGoal = false;
}

void CallbackParkingGoalManual(const geometry_msgs::PoseStamped::ConstPtr& end) {
//     m_validGoal = true;// If true, the goal cands is subscribed!
//     // if (!m_planning_start) {
//     GOAL_ORI_G[0] = GOAL_G[0] = end->pose.position.x;
//     GOAL_ORI_G[1] = GOAL_G[1] = end->pose.position.y;
//     GOAL_G[2] = GOAL_ORI_G[2] = tf::getYaw(end->pose.orientation);
//     // GOAL_EXTEND_SIGN = end->pose.position.z;
//     // GOAL_G[0] -= GOAL_EXTEND_SIGN*m_goal_bias*cos(GOAL_G[2]);
//     // GOAL_G[1] -= GOAL_EXTEND_SIGN*m_goal_bias*sin(GOAL_G[2]);
//     GOAL_G[0] -= m_center2rear*cos(GOAL_G[2]); // center of the rear axle
//     GOAL_G[1] -= m_center2rear*sin(GOAL_G[2]);
//     // }

//     // collision_range(GOAL_G[0], GOAL_G[1], GOAL_G[2]);    // with respect to the rear axle

//     geometry_msgs::PoseStamped poseStamped;
//     poseStamped.pose.position.x = GOAL_ORI_G[0];
//     poseStamped.pose.position.y = GOAL_ORI_G[1];
//     poseStamped.pose.position.z = 0.0;
//     geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(GOAL_ORI_G[2]);// Get a radian!
//     poseStamped.header = m_header;
//     poseStamped.pose.orientation = odom_quat;
//     Pose_Goal.publish(poseStamped); // add it visualizes at YOLO
}

void CallbackParkingGoal(const geometry_msgs::PoseArray::ConstPtr& end) {    //[end] which is the coordinates of the goal
    // cout << "callback!1111" << endl;
    m_validGoal = true;// If true, the goal cands is subscribed!
    // if (!m_path_pub_flag && end->poses.size() > 0) {
    
    if (end->poses.size() > 0) {
        double dist_orig2now = DISTANCE(end->poses[0].position.x, end->poses[0].position.y, GOAL_ORI_G[0], GOAL_ORI_G[1]);
        // cout << dist_orig2now << "   " << m_path_flag << endl;
        if (m_path_flag == false || (m_path_flag == true && dist_orig2now < 1.0)) {
            // GOAL_ORI_G[0] = GOAL_G[0] = end->poses[0].position.x;
            // GOAL_ORI_G[1] = GOAL_G[1] = end->poses[0].position.y;
            // GOAL_G[2] = GOAL_ORI_G[2] = tf::getYaw(end->poses[0].orientation);
            GOAL_G[0] = end->poses[0].position.x;
            GOAL_G[1] = end->poses[0].position.y;
            GOAL_G[2] = tf::getYaw(end->poses[0].orientation);
            // cout << "callback!2222" << endl;
            collision_range(GOAL_G[0] - m_center2rear * cos(GOAL_G[2]), GOAL_G[1] - m_center2rear * sin(GOAL_G[2]), GOAL_G[2], 0); // with respect to the rear axle
            // collision_range(GOAL_G[0] , GOAL_G[1], GOAL_G[2], 0); // with respect to the rear axle
            
            geometry_msgs::PoseStamped poseStamped;
            poseStamped.pose.position.x = GOAL_G[0];
            poseStamped.pose.position.y = GOAL_G[1];
            poseStamped.pose.position.z = 0.1;
            geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(GOAL_G[2]);// Get a radian!
            poseStamped.header = m_header;
            poseStamped.pose.orientation = odom_quat;
            Pose_Goal.publish(poseStamped); // add it visualizes at YOLO
        }
    }
}

void CallbackLocalizationData(const std_msgs::Float32MultiArray::ConstPtr& msg) { // the center of the vehicle
    m_car.x = msg->data.at(0);      // x
    m_car.y = msg->data.at(1);      // y
    m_car.th = msg->data.at(2);     // theta
    m_car.vel = msg->data.at(3);    // [m/s]    
    collision_range(m_car.x , m_car.y, m_car.th, 1);    // ms add
}

int withinpoint(int modx1, int mody1, int modx2, int mody2, int modx3, int mody3, int modx4, int mody4, int x, int y) {
    typedef boost::geometry::model::d2::point_xy<int> point_type;
    typedef boost::geometry::model::polygon<point_type> polygon_type;

    polygon_type poly;
    poly.outer().assign({
        point_type {modx1, mody1},
        point_type {modx2, mody2},
        point_type {modx3, mody3},
        point_type {modx4, mody4},
        point_type {modx1, mody1}
    });

    point_type p(x, y);

    return boost::geometry::within(p, poly);
}

void CallbackVeloObs(const VPointCloud velo_obs) {
    // cout << "call back" << endl;
    BOUNDS.low[0] = -12.6369;
    BOUNDS.low[1] = -12.6369;
    BOUNDS.high[0] = 12.6369;
    BOUNDS.high[1] = 12.6369;
    
    vector<Vector2d> vObstacle;    
    int obs_count = 0;
    // cout << "size: " << velo_obs.points.size() << endl;
    for (int i = 0; i < velo_obs.points.size(); i++) {
        vObstacle.push_back(Vector2d(velo_obs.points[obs_count].x, 
                                     velo_obs.points[obs_count].y));
        obs_count++;                                
    }
    if( g_pTree != NULL )
        g_pTree->clear();

    g_pTree = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for(int k=0; k<vObstacle.size(); k++)
        g_pTree->push_back(pcl::PointXYZ(vObstacle[k](0), vObstacle[k](1), 0.0));

    g_kdTree.setInputCloud(g_pTree);
}

void CallbackHeightMap(const nav_msgs::OccupancyGrid::Ptr map) {// for occupancy grid map construction
    map_range = map->info.resolution * map->info.width;
    m_gridDim = map->info.width;
    m_gridResol = map->info.resolution;
    BOUNDS.low[0] = -map_range/2*1.0;
    BOUNDS.low[1] = -map_range/2*1.0;
    BOUNDS.high[0] = map_range/2*1.0;
    BOUNDS.high[1] = map_range/2*1.0;

    // cout << "MAP BOUND: " << map_range/2*1.0 << endl;
    bool** HeightMap;
    bool** HeightMapMargin;
    HeightMap = new bool*[map->info.width];
    HeightMapMargin = new bool*[map->info.width];

    for (int x = 0; x < map->info.width; x++)
        HeightMap[x] = new bool[map->info.height]; 
        
    // Global Coordinate Obstacle Data
    m_obstaclePtCloudLocal.clear();
    m_obstaclePtCloudLocal.header.frame_id = "map";
    m_obstaclePtCloudLocal.points.resize(map->info.width*map->info.height);    //
        
    int obs_count = 0;
    vector<Vector2d> vObstacle;
    for (int x = m_gridDim-1 ; x >= 0 ; x--) {
        for (int y = m_gridDim-1 ; y >=0 ; y--) {
            HeightMap[x][y] = map->data[y * (map->info.width) + x] ? true : false;///All observalbe
            if (withinpoint(x, y) == 1) {// add 
                HeightMap[x][y] = false;
            }
            double doX, doY, doX_g, doY_g;// Global obstacles' INFO 
            arr2real(x * HeightMap[x][y], y * HeightMap[x][y], doX, doY);
            Local2Global(doX, doY, doX_g, doY_g);
            // isFreeSpace(GOAL_G[0], GOAL_G[1], GOAL_G[2]);

            // if (sqrt(doX*doX + doY*doY) > 1.0) { // 1 [m]
                vObstacle.push_back(Vector2d(doX, doY));// Current doX and doY are located at LOCAL

                m_obstaclePtCloudLocal.points[obs_count].z = 1.0;
                m_obstaclePtCloudLocal.points[obs_count].x = doX_g;
                m_obstaclePtCloudLocal.points[obs_count++].y = doY_g;
            // }
        }
    }
    double Rx, Ry, Rth;
    GetRelCoordsFromCar(GOAL_G[0], GOAL_G[1], GOAL_G[2], Rx, Ry, Rth);
    coord(M_SPACE_WIDTH/2, - M_SPACE_LENGTH/2, Rx, Ry, Rth, modx1, mody1);
    coord(M_SPACE_WIDTH/2,   M_SPACE_LENGTH/2, Rx, Ry, Rth, modx2, mody2);
    coord(-M_SPACE_WIDTH/2,  M_SPACE_LENGTH/2, Rx, Ry, Rth, modx3, mody3);
    coord(-M_SPACE_WIDTH/2, -M_SPACE_LENGTH/2, Rx, Ry, Rth, modx4, mody4);

    m_obstaclePtCloudLocal.points.resize(obs_count);
    Pub_obstacleLocal.publish(m_obstaclePtCloudLocal);

    if( g_pTree != NULL )
        g_pTree->clear();

    g_pTree = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);

    for(int k=0; k<vObstacle.size(); k++)
        g_pTree->push_back(pcl::PointXYZ(vObstacle[k](0), vObstacle[k](1), 0.0));

    g_kdTree.setInputCloud(g_pTree);
}

void PLANNER_FRAMEWORK()
{
    int argc = 0;
    char** argv;
    ros::init(argc, argv, "Informed_RRT_star");
    ros::NodeHandle node;
    ros::NodeHandle priv_nh("~");

    m_header.stamp = ros::Time::now();
    m_header.frame_id = "map";

    Pose_Goal = node.advertise<geometry_msgs::PoseStamped>("PoseGoal", 1); // MSK: TODO
    Pose_Start = node.advertise<geometry_msgs::PoseStamped>("PoseStart", 1); // MSK: TODO
    Pub_obstacleLocal = node.advertise<VPointCloud>("velodyne_obs", 1);// MSK: Obstacle Information
    Pub_RS_field = node.advertise<geometry_msgs::PoseArray>("PoseArray_RS_field", 1);// MSK: RRT field vector// Those two are not used.
    Pub_Sample_field = node.advertise<geometry_msgs::PoseArray>("PoseArray_Sample_field", 1);
    Pub_Path = node.advertise<nav_msgs::Path>("parkingPath", 1);// Publish RRT Path
    Pub_parkingStart = node.advertise<std_msgs::Float32MultiArray>("/parkingStart", 1);
    Pub_collision_check = node.advertise<geometry_msgs::PoseArray>("collision_edge", 1);    //add
    Pub_collision_check_car = node.advertise<geometry_msgs::PoseArray>("collision_edge_car", 1);    //add
    Pub_TargetTree = node.advertise<geometry_msgs::PoseArray>("target_tree", 1);  // ms add: visualize target tree

    Sub_heightMap = node.subscribe("occ_map", 1, CallbackHeightMap);// MSK: occupancy map subscribe from CARLA or REAL_ENV
    Sub_localization = node.subscribe("LocalizationData", 10, CallbackLocalizationData);// MSK: Get current Info. <x, y, th, v>

    if (MANUAL_GOAL)
        Sub_parkingGoal = node.subscribe("goal", 1, CallbackParkingGoalManual);
    else
        Sub_parkingGoal = node.subscribe("parking_cands", 1, CallbackParkingGoal); // get from avm
        // Sub_parkingGoal = node.subscribe("PoseGoal", 1, CallbackParkingGoal_); // MS add

    ros::Subscriber exsub = node.subscribe("CanVelData2", 1, CanTopicProcess);// MSK: subscribing the VELOCITY
    Sub_rePlanning = node.subscribe("replanning", 1, CallbackReplanning);
    cout << "START of DYROS planner*: ROS Version" << endl;
    // Sub_manualPlanningFlag = node.subscribe("manual_plan", 1, CallbackManualPlan);  // add

    // Initialize member values for planner
    node.getParam("turning_radius", m_turning_radius);
    node.getParam("max_curvature_derivative", m_max_curvature_derivative);
    node.getParam("target_bias_ratio", m_target_bias_ratio);
    node.getParam("goal_bias", m_goal_bias);
    node.getParam("planning_time", m_planning_time);
    node.getParam("center2rear", m_center2rear);    // jw add
    // node.getParam("test_iteration", m_test_iter);
    node.getParam("using_hybrid_curvature", m_using_hybrid_curvature);
    node.getParam("extend_length", m_extend_length);
    m_target_tree_gen = new target_tree_generator(1.0 / m_turning_radius, m_max_curvature_derivative, isFreeSpace);
    if (m_using_hybrid_curvature) {
        state_space = ob::StateSpacePtr(new HybridCurvatureStateSpace(m_turning_radius, m_max_curvature_derivative));
        rs_state_space = ob::StateSpacePtr(new ReedsSheppStateSpace(m_turning_radius));
    }
    else
        // state_space = ob::StateSpacePtr(new GeneralizedCurvatureStateSpace(m_turning_radius, m_max_curvature_derivative, m_gamma));
        state_space = ob::StateSpacePtr(new ReedsSheppStateSpace(m_turning_radius));
    ss = new og::SimpleSetup(state_space);

    // m_extend_length = m_center2rear - GOAL_EXTEND_SIGN * m_goal_bias + EXTEND_LEN; //- 0.1;         //extend the rrt path
    ros::spin();
}


int main(int argc, char* argv[])
{
    try {
        boost::thread t = boost::thread(boost::bind(&PLANNER_FRAMEWORK));
        // cuspNlength_log.open("/home/dyros-vehicle/catkin_ws/cusp_length.csv", std::ios_base::app); // append instead of overwrite
        // if (!READ_PATH)
        //     myfile_rs.open("/home/dyros-vehicle/catkin_ws/hc_path_test_0.07.csv");
        // path_log.open("/home/dyros-vehicle/ms_ws/20210902/path_data.csv"); // add
        // string reading_path = "/home/dyros-vehicle/catkin_ws/hc_path_s2c12U5_0.07.csv";
        // read_path2.open(reading_path);

        int ii = 0;
        // string line;
        // while(getline(read_path2, line))
        //     ii++;
        m_total_path_waypt = ii;
        // read_path2.close();

        // read_path.open(reading_path);
        // label_path.open(reading_path);
        srand((unsigned int)time(0));

        t.join();
    }

    catch(std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    catch(...) {
        std::cerr << "Exception of unknown type!\n";
    }
    return 0;
}