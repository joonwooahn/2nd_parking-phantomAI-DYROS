#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <string.h>
#include "opencv2/opencv.hpp"
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <dead_reckoning/EspMeasurements.h>
#include <dead_reckoning/WheelSpeedMeasurements.h>
#include <dead_reckoning/GearState.h>
 
using namespace std;

class DeadReckoning {
    public:
        DeadReckoning(ros::NodeHandle node);
        ~DeadReckoning();
            
    private:
        double wheel_base = 3.010;
        double rear_wheel_distance = 1.652; // yw: 1.633

        struct CARPOSE
        {
            double x;
            double y;
            double yaw;
            double vel;
        };
        CARPOSE m_car;
        
        bool forward{true};
        int sign = 1;
        double m_dt = 0.05;
        float c_velo{0.0};
        float c_yaw_rate{0.0};
        double c_steer = 0.0;
        double c_yaw = 0.0;
        double slip_beta = 0.0;

        ros::Time ros_time;
        ros::Time prev_ros_time;
        double m_hw_time;
        nav_msgs::Path dr_path_msg;
        nav_msgs::Path loam_path_msg;
        geometry_msgs::PoseStamped pose;
        geometry_msgs::PoseStamped loam_pose;
        geometry_msgs::PoseStamped m_poseStamped;

        bool start_trigger = true;

        void CallbackParkingStart(const nav_msgs::Path::ConstPtr& msg);    // JW add
        // void CallbackVelocityData(const std_msgs::Float32MultiArray::ConstPtr& msg);
        // void CallbackSteerData(const std_msgs::Float32MultiArray::ConstPtr& msg);
        void CallbackReplanning(const std_msgs::Bool::ConstPtr& msg);

        void CallbackGearSwitching(const std_msgs::Bool::ConstPtr& msg);
        void CallbackLoam(const std_msgs::Float32MultiArray::ConstPtr& msg);

        void CallbackWheelSpeedData(const dead_reckoning::WheelSpeedMeasurements::ConstPtr& msg);
        void CallbackESP_Data(const dead_reckoning::EspMeasurements::ConstPtr& msg);
        void CallbackGearState(const dead_reckoning::GearState::ConstPtr& msg);
        
        void PubDRpath();
        void PubDR_localization();
        void UpdatePosition();
        ros::Time start_ros_time;

        ros::Subscriber sub_velo, sub_steer, sub_gear, sub_canVelData;
        ros::Subscriber sub_wheel_speed, sub_front_image;//phantom msg subscriber
        ros::Subscriber sub_esp;
        ros::Subscriber sub_parkingStart, sub_tmp;

        ros::Publisher Pub_poseVehicle;
        ros::Publisher pub_dr_path;
        ros::Publisher pub_loam_path;
        ros::Publisher pub_localization_data;
        ros::Publisher pub_canVelData;
};
