#include <string>
#include <algorithm>
#include "../include/dead_reckoning.h"
 
#define DEG2RAD M_PI/180
#define KpH2MpS 0.277778
#define Mps2KpH 3.6

bool m_parkingStartFlag = false;
ros::Subscriber Sub_replanning;
 
DeadReckoning::DeadReckoning(ros::NodeHandle node) {
    cout << "Initialize Vehicle Position" << endl;
    m_car.x = 0.0;
    m_car.y = 0.0;
    m_car.yaw = 0.0;
    m_car.vel = 0.0;
 
    // sub_loam = node.subscribe("/LocalizationData", 10, &DeadReckoning::CallbackLoam, this);
    sub_wheel_speed = node.subscribe("/vehicle_state/wheel_speed_measurements", 10, &DeadReckoning::CallbackWheelSpeedData, this);
    sub_esp = node.subscribe("/vehicle_state/esp_measurements", 1, &DeadReckoning::CallbackESP_Data, this);
    sub_gear = node.subscribe("/vehicle_state/gear_state", 1, &DeadReckoning::CallbackGearState, this);
    sub_parkingStart = node.subscribe("/parkingPath", 10, &DeadReckoning::CallbackParkingStart, this);
    Sub_replanning = node.subscribe("/replanning", 1, &DeadReckoning::CallbackReplanning, this);  // From doRRT node


    pub_localization_data = node.advertise<std_msgs::Float32MultiArray>("/LocalizationData", 1);
    // pub_dr_path = node.advertise<nav_msgs::Path>("DR_path", 1);
    // Pub_poseVehicle = node.advertise<geometry_msgs::PoseStamped>("poseVehicle", 1);
    // pub_canVelData = node.advertise<std_msgs::Float32MultiArray>("/CanVelData2", 1);    // JW add

    ros::Rate r(100);
    // Create the vehicle's AXIS
    // std_msgs::Header header;
    // header.stamp = ros::Time::now();
    // header.frame_id = "map";
    ros::spin();
}


void DeadReckoning::CallbackReplanning(const std_msgs::Bool::ConstPtr& msg) {
    m_car.x = 0.0;
    m_car.y = 0.0;
    m_car.yaw = 0.0;
    m_car.vel = 0.0;
    m_parkingStartFlag = true;
}

// JW add
void DeadReckoning::CallbackParkingStart(const nav_msgs::Path::ConstPtr& msg) {
    m_car.x = 0.0;
    m_car.y = 0.0;
    m_car.yaw = 0.0;
    m_car.vel = 0.0;
    m_parkingStartFlag = true;
}

void DeadReckoning::CallbackGearState(const dead_reckoning::GearState::ConstPtr& msg) {
    start_ros_time = ros::Time::now();
    forward = (msg->gear_state == 5) ? true : false;
}

void DeadReckoning::CallbackESP_Data(const dead_reckoning::EspMeasurements::ConstPtr& msg) {
    float lat_accel = msg->lat_accel;
    float long_accel = msg->long_accel;
    float yaw_rate = msg->yaw_rate;
    
    bool yaw_rate_stat = msg->yaw_rate_stat;
    bool yaw_rate_diag = msg->yaw_rate_diag;
}

void DeadReckoning::CallbackWheelSpeedData(const dead_reckoning::WheelSpeedMeasurements::ConstPtr& msg) {
    float rear_left_wheel = msg->rear_left;
    float rear_right_wheel = msg->rear_right;
    float front_left_wheel = msg->front_left;
    float front_right_wheel = msg->front_right;
    double can_hw_timestamp = msg->hw_timestamp;

////////////////
    if (start_trigger) {
        m_hw_time = can_hw_timestamp;
        start_trigger = false;
    }
    // ros_time = ros::Time::now();
    m_dt = (can_hw_timestamp - m_hw_time);
    m_hw_time = can_hw_timestamp;
////////////////
       
    double yaw_gain = 1.0; 

    if (forward)    sign = 1;
    else            sign = -1;

    if (sign < 0) 
        yaw_gain = 1.124;  // backward 
        // yaw_gain = 1.05;  // backward 
        

    // c_yaw_rate = yaw_rate; // [rad/s]
    c_yaw_rate = yaw_gain*(rear_left_wheel - rear_right_wheel) * KpH2MpS / rear_wheel_distance; // [rad/s]
    
    double c_veloFront = 0.5 * (front_left_wheel + front_right_wheel) * KpH2MpS;
    double c_veloRear =  0.5 * (rear_left_wheel + rear_right_wheel) * KpH2MpS;
    c_velo = 1.03*0.5*(c_veloFront + c_veloRear);
    // c_velo = 1.03*0.5*(c_veloFront + c_veloRear);
    
    // std_msgs::Float32MultiArray msgVel;
    // cout << "vel: " << c_velo << endl;
    // pub_canVelData.publish(msgVel);

    if (m_parkingStartFlag) {
        UpdatePosition();
        PubDR_localization();// \Localization
    }
}


//Based on kinematic bicycle model.
void DeadReckoning::UpdatePosition() {
    //Calculate dt
    // if (start_trigger) {
    //     prev_ros_time = ros::Time::now();
    //     start_trigger = false;
    // }
    // ros_time = ros::Time::now();
    // dt = (ros_time - prev_ros_time).toSec();
    // prev_ros_time = ros_time;
    

    if (forward)    sign = 1;
    else            sign = -1;
    
    // Kinematic Bicycle model
    m_car.x += (sign)*c_velo*cos(m_car.yaw)*m_dt;
    m_car.y -= (sign)*c_velo*sin(m_car.yaw)*m_dt;
    m_car.yaw += (sign)*c_yaw_rate*m_dt;
    m_car.vel = (sign)*c_velo;
    
    // PubDRpath();// for visualizing a cumulative path
}

void DeadReckoning::PubDR_localization() {          //deadreckoning data  
    // cout << "pub!" <<endl;
    std_msgs::Float32MultiArray msg;
    // msg.data.push_back(m_car.x + 1.501 * cos(-m_car.yaw));
    // msg.data.push_back(m_car.y + 1.501 * sin(-m_car.yaw));
    msg.data.push_back(m_car.x);
    msg.data.push_back(m_car.y);
    msg.data.push_back(-m_car.yaw);
    msg.data.push_back(m_car.vel/KpH2MpS);

    pub_localization_data.publish(msg);
}

// void DeadReckoning::PubDRpath() {
//     dr_path_msg.header.frame_id = "map";
//     dr_path_msg.header.stamp = ros::Time::now();
//     pose.header = dr_path_msg.header;

//     pose.pose.position.x = m_car.x;
//     pose.pose.position.y = m_car.y;
//     pose.pose.position.z = 1.0;
//     dr_path_msg.poses.push_back(pose);

//     pub_dr_path.publish(dr_path_msg);
// }

DeadReckoning::~DeadReckoning() {}