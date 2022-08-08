#include <ros/ros.h>
#include <ros/package.h>
#include <math.h>
#include <algorithm>
// #include <string.h>
#include <string>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

nav_msgs::Path loam_msg;
nav_msgs::Path yw_msg;
geometry_msgs::PoseStamped pose_loam;
geometry_msgs::PoseStamped pose_yw;

struct LOAMPOSE {
    double x, y, th;
};
LOAMPOSE m_loam;

struct YWPOSE {
    double x, y, th;
};
YWPOSE m_yw;

void Initialize();
void PubLOAMpath();
void PubYWpath();

ros::Subscriber sub_localization_data, sub_localization_data_yw;
ros::Publisher pub_yw_path, pub_loam_path;

void Initialize() {
    m_loam.x = 0.0;
    m_loam.y = 0.0;
    m_loam.th = 0.0;
    m_yw.x = 0.0;
    m_yw.y = 0.0;
    m_yw.th = 0.0;
}

void CallbackLOAMData(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    m_loam.x = msg->data.at(0);      // x
    m_loam.y = msg->data.at(1);      // y
    m_loam.th = msg->data.at(2);     // theta

    PubLOAMpath();
}

void CallbackYWData(const std_msgs::Float32MultiArray::ConstPtr& msg) {
    m_yw.x = msg->data.at(0);      // x
    m_yw.y = msg->data.at(1);      // y
    m_yw.th = msg->data.at(2);     // theta
    
    PubYWpath();
}

void PubLOAMpath() {
    loam_msg.header.frame_id = "map";
    pose_loam.header = loam_msg.header;
    pose_loam.pose.position.x = m_loam.x ;
    pose_loam.pose.position.y = m_loam.y ;
    pose_loam.pose.position.z = 0;
    loam_msg.poses.push_back(pose_loam);

    pub_loam_path.publish(loam_msg);
}

void PubYWpath() {
    yw_msg.header.frame_id = "map";
    pose_yw.header = yw_msg.header;
    pose_yw.pose.position.x = m_yw.x ;
    pose_yw.pose.position.y = m_yw.y ;
    pose_yw.pose.position.z = 0;
    yw_msg.poses.push_back(pose_yw);

    pub_yw_path.publish(yw_msg);
}

int main(int argc, char *argv[]){
    ros::init(argc, argv, "comparing_node");
    ros::NodeHandle node;
    ros::NodeHandle priv_nh("~");

    sub_localization_data = node.subscribe("/LocalizationData", 1, &CallbackLOAMData); // lego_loam
    sub_localization_data_yw = node.subscribe("/YWLocalizationData", 1, &CallbackYWData); // YW SLAM

    pub_loam_path = node.advertise<nav_msgs::Path>("/LOAM_path", 1);
    pub_yw_path = node.advertise<nav_msgs::Path>("/YW_path", 1);

    Initialize();
 
    ros::spin();
}