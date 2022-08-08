#include <vector>
#include <Eigen/Core>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <iostream>
#include <string>
#include <fstream>
#include <string>
#include <stdlib.h>
#include <time.h>

using namespace std;
using namespace Eigen;
using std::ifstream;

class LocalPlannerThread {

private :
    struct CARPOSE
    {
        double x;
        double y;
        double th;
        double vel;
    };
    CARPOSE m_car;

    int init_argc;
    char** init_argv;

    ros::Subscriber Sub_lookAheadPt;
    ros::Subscriber Sub_LearningJW;
    ros::Subscriber Sub_localization;
    ros::Subscriber Sub_parkingPath;
    ros::Subscriber Sub_occMap;
    ros::Subscriber Sub_driving_function;

    ros::Publisher Pub_MarkerLookaheadPtJW;
    ros::Publisher Pub_MarkerCar, Pub_MarkerCar_center, Pub_poseVehicle;
    ros::Publisher Pub_CarPw;
    ros::Publisher Pub_DataLogging;
    ros::Publisher Pub_ControlCmd;
    ros::Publisher Pose_Start;
    ros::Publisher Pub_Replanning;
	ros::Publisher Pub_MarkerParkingPath;
    ros::Publisher Pub_vehicleTraj;
    ros::Publisher Pub_BreakCommand;
    ros::Publisher Pub_temp_path;
    ros::Publisher Pub_tentacle, Pub_chosenPath, Pub_chosenPathbyValue;


    visualization_msgs::Marker m_CarPos;
    visualization_msgs::Marker m_CarPosLine;
    visualization_msgs::Marker m_CarPosLine_center;
    visualization_msgs::Marker m_lookaheadPtJW;
    visualization_msgs::Marker m_lookaheadPtLineJW;
    visualization_msgs::Marker m_CarPw;
    visualization_msgs::Marker m_parkingPath;
    visualization_msgs::MarkerArray m_parkingPathArr;

    string m_load_path;

    bool m_finishFlag;
    vector<Vector2d> *m_LocalSplinePath = new vector<Vector2d>[99];

    int m_carIdx;
    int m_pLIdx;
    double m_CrossTrkErr;

    double m_cross_track_error;
    double m_cross_track_error_mean;
    double count_cte;

    double m_targetVelocity;
    double m_controlVelocity;
    double m_orthogonal_val;
    double m_velocityGain;
    int m_maxSearchIdx;

    double m_I_brake;
    double m_I_accel;

    double m_car_x, m_car_y;
    double m_car_center_x, m_car_center_y;
    double m_car2switchPt;

    int m_navi_info;
    int m_intersection_flag;

    vector<Vector2d> m_switching_point_vector;
    vector<int> m_switching_point_idx;
    
    ros::Time m_ros_time;
    ros::Time m_ros_time2;
    double m_cum_distance;
    
    void MemberValInit();
    void MarkerInit();
    void Compute();
    void vehicleTrajectory(int time);
    
    void CallbackLearningData(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void CallbackModelBasedDrivingData(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void CallbackLocalizationData(const std_msgs::Float32MultiArray::ConstPtr& msg);
    void CallbackParkingPath(const nav_msgs::Path::ConstPtr& msg);
    void CallbackOccMapForDriving(const nav_msgs::OccupancyGrid::Ptr& map);

 
    void Local2Global(double Lx, double Ly, double &gX, double &gY);
    double PurePursuit(double lookX, double lookY);
    double SteeringAng_Radius(int carIdx);
    double CalculateClosestPt(int& pWIdx);
    int CalculateLookAheadPt();
    
    pair<double, double> Kanayama(double velo_input);//@MSK
    double SlowAtSwitchingPoint(double velo_input);
    double CurvatureVelocityControl(double target_velocity, double curvature);
    double CalculateCurvature(int idx);
    double CalculateCurvature(int idx, double prev_curv);

    double m_prev_steering;
    double m_prev_velocity;
    double m_diff_threshold; //[deg]
    double m_cumulative_error;
    double m_curvature;
    double m_counter;
    bool m_using_tracking_strategy;
    // add
    std::ofstream m_error_log, m_error_log2;
    int m_carIdx_prev;
    vector<int> m_waypt_seg;
    double m_prev_curv;

    void SwitchOrFinishChk();
    void Publish_topic(double LookAheadPtX, double LookAheadPtY);
    void LookAheadOrthogonal(double& LookAheadPtX, double& LookAheadPtY);
    
    void vehicleControl(double steer, double velocity);

    double cur_rad(double x1, double y1,double x2, double y2,double x3, double y3);

public :
    LocalPlannerThread(int argc, char** argv);
    ~LocalPlannerThread();

    void Orthogonal(int value);
    
    double m_Steer_cmd;
    double m_acc;
    double m_brake;

    int m_pathIdx;
    int m_switchLen;
    float m_dir_mode;

    bool m_pathFlag;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

};

