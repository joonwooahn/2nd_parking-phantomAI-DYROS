#include "undistorted_top_view/ocam_functions.h"
#include "undistorted_top_view/Parameters.h"

#include <iostream>
#include <string>
#include <exception>
 
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Path.h>
 
#include <cv_bridge/cv_bridge.h>
// #include <cv.h> ///
// #include <opencv2/opencv.hpp> /// jw add
#include <opencv2/core/core.hpp>
// #include <opencv2/highgui/highgui.hpp>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/point_cloud.h>
#include <algorithm>

// #include <undistorted_top_view/PhantomVisionNetMsg.h> 
// #include <undistorted_top_view/VisionPhantomnetData.h>
// #include <undistorted_top_view/VisionPhantomnetDataList.h>
#include <undistorted_top_view/ParkingPhantomnetData.h>
#include <undistorted_top_view/ParkingPhantomnetDetection.h>
#include <undistorted_top_view/Bounding_Box.h> 
 
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/io/io.hpp>
#include <boost/program_options.hpp>

#define DISTANCE(x1, y1, x2, y2) sqrt((x1 - x2)*(x1 - x2) +(y1 - y2)*(y1 - y2))

using namespace cv;
using namespace std;

// #define m_AVM_IMG_WIDTH  200  //400
// #define m_AVM_IMG_HEIGHT 200  //400
// #define m_REAL_OCCUPANCY_SIZE_X 16.5    // m_AVM_IMG_WIDTH 400PIX == 25Meter
// #define m_REAL_OCCUPANCY_SIZE_Y 16.5    // m_AVM_IMG_HEIGHT 400PIX == 25Meter
int m_AVM_IMG_WIDTH = 0, m_AVM_IMG_HEIGHT = 0;
double m_REAL_OCCUPANCY_SIZE_X = 0.0, m_REAL_OCCUPANCY_SIZE_Y = 0.0;
double m_METER_PER_PIXEL = 0.0;
int m_PADDING_VALUE = 0; 
bool m_PADDING_UP = true;
// bool m_PADDING_UP = false;
double m_goal_bias = 0.0;
int m_FULL_IMG_RESOL_WIDTH = 0, m_FULL_IMG_RESOL_HEIGHT = 0, m_CROP_ROI_WIDTH = 0, m_CROP_ROI_HEIGHT = 0;
double m_PARKING_SPOT_WIDTH = 0.0, m_PARKING_SPOT_LENGTH = 0.0, m_FREE_ERASE_GAIN = 0.0;
double m_OCCUPANCY_RATIO = 0.0;
int m_front_rear_dist = 21; /// for 4 images
int m_parking_index = 0;
// bool m_ALL_IMAGE_FLAG = false;
bool m_ALL_IMAGE_FLAG = true;
string m_PARKING_POSE_TOPIC_NAME = "";

int flag = 0;
int imgcnt = 0;

#define M_DEG2RAD  3.1415926/180.0
#define FRONT 0
#define REAR 1
#define LEFT 2
#define RIGHT 3
//////////////////////////////////////////////////////

// #define RESIZE_BAG_TO_ORG 3.75 
// #define PARKING_SPACE 21
// #define OBJ_DET_CONFIDENCE 0.1
// #define ENLARGED_BOX 15
    
ros::Subscriber Sub_phantom_front, Sub_phantom_rear, Sub_phantom_left, Sub_phantom_right;
ros::Subscriber Sub_phantom_front_seg, Sub_phantom_rear_seg, Sub_phantom_left_seg, Sub_phantom_right_seg;
ros::Subscriber Sub_parkingPath, Sub_replanning;
ros::Subscriber Sub_localizationData, Sub_parkingGoal;

ros::Publisher Pub_AVM_img, Pub_AVM_seg_img, Pub_AVM_seg_img_gray, Pub_AVM_DR, Pub_ICP_img;
ros::Publisher Pub_occupancyGridMap;
// ros::Publisher Pub_Boundingbox;

cv::Mat AVM_front, AVM_rear, AVM_left, AVM_right;
cv::Mat AVM_seg_front, AVM_seg_rear, AVM_seg_left, AVM_seg_right;
cv::Mat AVM_seg_front_gray, AVM_seg_rear_gray, AVM_seg_left_gray, AVM_seg_right_gray;

// Calibration Results
ocam_model m_front_model, m_left_model, m_right_model, m_rear_model;

// occupancy grid map for path planning
nav_msgs::OccupancyGrid occupancyGridMap;
int m_dimension = 0, m_gridDim = 0;
double m_gridResol = 0;
 
struct CARPOSE {
    double x, y, th, vel, dir;
};  CARPOSE m_car;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr m_avm_data(new pcl::PointCloud<pcl::PointXYZRGB>);

bool m_flagParking = false;
// bool m_flagParking = true;  
bool m_flag_goal_empty = false;     // real
unsigned int m_avmDRsize = 0, m_DRcnt = 0;
bool m_rgb_color = true;
// bool m_rgb_color = false;



// Inverse Matrix in States
double Rear_Inv_Rot_Matrix[3][3] =
{ { -0.840691520969634,   -0.00532631038221457,   -0.541488131899006},
  { -0.00182074169124791,  -0.999918170510072,    0.0126624319735592},
  { -0.541511266244256,	0.0116311092120905,	0.840613029775913} };

double Front_Inv_Rot_Matrix[3][3] =
{ {0.942629404158933,	-0.000904859548010679,	-0.333839763425768},
  {-0.000316098443064055,	0.999993459294333,	-0.00360297798048014},
  {0.333840840067163,	0.00350179921638951,	0.942622952672753} };

double Left_Inv_Rot_Matrix[3][3] =
{ { 0.0394432989745752,	0.630444914256981,	-0.775231214705335},
  { -0.999165508851345,	0.0166489676524436,	-0.0372974234758920},
  { -0.0106071715284714,	0.776055424543644,	0.630575503765053} };

double Right_Inv_Rot_Matrix[3][3] =
{ { 0.0389773666687844,	-0.655098065566229,	-0.754537798509095},
  { 0.999240047084178,	0.0257841039800383,	0.0292319736787658},
  { 0.000305271643985279,	-0.755103770665550,	0.655605218356209} };

// ParkingPhantomnetDetection Left_detection;
// typedef struct DETECTIONS{
//     char classification;
//     float probability;
//     float x;
//     float y;
//     float width;
//     float height;
// };
// std::vector<DETECTIONS> Left_detection;

// // Extrinsic Parameters
// double M_front_param[6] = {0.688 * M_DEG2RAD,  21.631 * M_DEG2RAD,   3.103* M_DEG2RAD   ,1.905,   0.033, 0.707 };
// double M_left_param[6] =  {1.133 * M_DEG2RAD,  19.535 * M_DEG2RAD,   92.160* M_DEG2RAD  ,0.0,     1.034, 0.974 };
// double M_right_param[6] = {3.440 * M_DEG2RAD,  18.273 * M_DEG2RAD,  -86.127* M_DEG2RAD  ,0.0,    -1.034, 0.988 };
// double M_back_param[6] =  {0.752 * M_DEG2RAD,  31.238 * M_DEG2RAD,  -178.189* M_DEG2RAD ,-2.973, -0.065, 0.883 };

// // // New Extrinsic Parameters 15 mm --> more corect than 9 mm
// double M_front_param[6] = {0.672 * M_DEG2RAD,  21.378 * M_DEG2RAD,   1.462* M_DEG2RAD   ,   1.885,   0.038, 0.686 };
// double M_left_param[6]  = {0.963 * M_DEG2RAD,  19.283 * M_DEG2RAD,   91.702* M_DEG2RAD  ,   0.0,    1.059, 0.978 };
// double M_right_param[6] = {1.714 * M_DEG2RAD,  19.713 * M_DEG2RAD,  -87.631* M_DEG2RAD  ,   0.0,    -1.059, 0.972 };
// double M_back_param[6]  = {-0.257 * M_DEG2RAD, 32.645 * M_DEG2RAD,  179.773* M_DEG2RAD ,   -3.002, -0.033, 0.922 };

// // New Extrinsic Parameters 9 mm
// double M_front_param[6] = {0.617 * M_DEG2RAD,  21.397 * M_DEG2RAD,   1.381* M_DEG2RAD   ,   1.880,   0.038, 0.689 };
// double M_left_param[6] =  {0.970 * M_DEG2RAD,  19.231 * M_DEG2RAD,   91.699* M_DEG2RAD  ,   0.0,    1.053, 0.979 };
// double M_right_param[6] = {1.659 * M_DEG2RAD,  19.690 * M_DEG2RAD,  -87.631* M_DEG2RAD  ,   0.0,    -1.053, 0.979 };
// double M_back_param[6] =  {-0.150 * M_DEG2RAD, 32.634 * M_DEG2RAD,  179.708* M_DEG2RAD ,   -2.997, -0.033, 0.924 };


// new phantomAI
// double M_front_param[6] = {-0.219 * M_DEG2RAD,  19.502 * M_DEG2RAD,  -0.055 * M_DEG2RAD   ,   3.815, -0.036, 0.698 };
// double M_left_param[6] =  {-3.385 * M_DEG2RAD,  50.826 * M_DEG2RAD,  86.420 * M_DEG2RAD  ,   1.934, 1.022, 0.970 };
// double M_right_param[6] = {2.553 * M_DEG2RAD,  48.985 * M_DEG2RAD,  -86.595 * M_DEG2RAD  ,   1.877, -1.062, 0.983 };
// double M_back_param[6] =  {0.863 * M_DEG2RAD, 32.785 * M_DEG2RAD,  -179.637 * M_DEG2RAD ,   -1.081, 0.021, 0.884 };

// new phantomAI
// double M_front_param[6] = {-0.238 * M_DEG2RAD,  20.154 * M_DEG2RAD,  -0.715 * M_DEG2RAD   ,  1.886, 0.037, 0.742 };
// double M_left_param[6] =  {-0.982 * M_DEG2RAD,  51.351 * M_DEG2RAD,  89.074 * M_DEG2RAD  ,   0.000, 1.041, 1.001 };
// double M_right_param[6] = {3.305 * M_DEG2RAD,  51.714 * M_DEG2RAD,  -87.275 * M_DEG2RAD  ,   0.000, -1.041, 0.985 };
// double M_back_param[6] =  {-0.577 * M_DEG2RAD, 32.868 * M_DEG2RAD,  179.834 * M_DEG2RAD ,   -2.992, 0.005, 0.893 };

// double M_front_param[6] = {-0.238 * M_DEG2RAD,  20.154 * M_DEG2RAD,  -0.715 * M_DEG2RAD   ,  1.886, 0.037, 0.742 };
// double M_left_param[6] =  {-0.982 * M_DEG2RAD,  51.351 * M_DEG2RAD,  89.074 * M_DEG2RAD  ,   0.000, 1.041, 0.951 };
// double M_right_param[6] = {0.305 * M_DEG2RAD,  51.714 * M_DEG2RAD,  -87.275 * M_DEG2RAD  ,   0.000, -1.041, 0.935 };
// double M_back_param[6] =  {-0.577 * M_DEG2RAD, 31.868 * M_DEG2RAD,  179.834 * M_DEG2RAD ,   -2.992, 0.005, 0.893 };
double M_front_param[6] = {-0.238 * M_DEG2RAD,  20.500 * M_DEG2RAD,  -0.715 * M_DEG2RAD   ,  3.886, 0.037, 0.720 };
double M_left_param[6] =  {0.180 * M_DEG2RAD,  52.200 * M_DEG2RAD,  88.200 * M_DEG2RAD  ,   2.000, 1.041, 1.001 };
double M_right_param[6] = {0.00 * M_DEG2RAD,  51.300 * M_DEG2RAD,  -89.400 * M_DEG2RAD  ,   2.000, -1.041, 0.985 };
double M_back_param[6] =  {0.800* M_DEG2RAD, 31.400  * M_DEG2RAD,  180.100 * M_DEG2RAD ,   -1.200, 0.095, 0.920 };

void onMouse(int event, int x, int y, int flags, void* param); 

void CallbackParkingPath(const nav_msgs::Path::ConstPtr& msg) {
    m_flagParking = true;
    // m_flagParking = false;
}

void CallbackReplanning(const std_msgs::Bool::ConstPtr& msg) {
    cout << "replanning !!!" << endl;
    m_flagParking = false;    
}

void arr2real(int recvX, int recvY, double& outX, double& outY) {
    outX = recvX * m_gridResol - (m_gridResol*m_gridDim - m_gridResol) / 2.0;
    outY = recvY * m_gridResol - (m_gridResol*m_gridDim - m_gridResol) / 2.0;
}

void real2arr(double recvX, double recvY, int& outX, int& outY) {
    outX = (m_gridDim/2.0) + recvX / m_gridResol;
    outY = (m_gridDim/2.0) + recvY / m_gridResol;
}

//Yangwoo===============================================================
double GOAL_G[3] = {0.0, 0.0, (0)};
int modx1 = 0, mody1 = 0, modx2 = 0, mody2 = 0, modx3 = 0, mody3 = 0, modx4 = 0, mody4 = 0;
int modx1free = 0, mody1free = 0, modx2free = 0, mody2free = 0, modx3free = 0, mody3free = 0, modx4free = 0, mody4free = 0;
int modx1car = 0, mody1car = 0, modx2car = 0, mody2car = 0, modx3car = 0, mody3car = 0, modx4car = 0, mody4car = 0;

void coord(double dx, double dy, double xx, double yy, double thh, int& x_, int& y_) {
    double modx = cos(M_PI/2.0 + thh)*dx - sin(M_PI/2.0 + thh)*dy + xx;
    double mody = sin(M_PI/2.0 + thh)*dx + cos(M_PI/2.0 + thh)*dy + yy;
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

int withinpoint_free(int x, int y) {
    typedef boost::geometry::model::d2::point_xy<int> point_type;
    typedef boost::geometry::model::polygon<point_type> polygon_type;

    polygon_type poly;
    poly.outer().assign({
        point_type {modx1free, mody1free}, point_type {modx2free, mody2free},
        point_type {modx3free, mody3free}, point_type {modx4free, mody4free},
        point_type {modx1free, mody1free}
    });

    point_type p(x, y);

    return boost::geometry::within(p, poly);
}

int withinpoint_car(int x, int y) {
    typedef boost::geometry::model::d2::point_xy<int> point_type;
    typedef boost::geometry::model::polygon<point_type> polygon_type;

    polygon_type poly;
    poly.outer().assign({
        point_type {modx1car, mody1car}, point_type {modx2car, mody2car},
        point_type {modx3car, mody3car}, point_type {modx4car, mody4car},
        point_type {modx1car, mody1car}
    });

    point_type p(x, y);
// 
    // cout << x << " " << y << " " << boost::geometry::within(p, poly) << endl;
    return boost::geometry::within(p, poly);
}
/////////////////////////////////////////////////////////////////////////////////////////////////////

void Local2Global(double Lx, double Ly, double &gX, double &gY) {
    gX = m_car.x + (Lx * cos(m_car.th) - Ly * sin(m_car.th));
    gY = m_car.y + (Lx * sin(m_car.th) + Ly * cos(m_car.th));
}

void onMouse(int event, int x, int y, int flags, void* param) {
	Mat *im = reinterpret_cast<Mat*>(param);

	switch (event) {
	case EVENT_LBUTTONDOWN:
		std::cout << "(" << x << ", " << y << "): " << static_cast<int>(im->at<uchar>(Point(x, y))) << std::endl;
		break;
	}
}

void printMat(double (&arr)[3][3])
{
  for (int i=0; i < 3 ; i++) {
    for (int j=0; j < 3 ; j++)    {
      std::cout << arr[i][j] << " ";
    }
    std::cout << std::endl;
  }
}

void getInvRotMat(double ROLL, double PITCH, double YAW, double (&Inv_Rot_matrix)[3][3])
{
  //Rot_matrix = {{cos(YAW)*cos(PITCH), cos(YAW)*sin(PITCH)*sin(ROLL)-sin(YAW)*cos(ROLL), cos(YAW)*sin(PITCH)*cos(ROLL)+sin(YAW)*sin(ROLL)},
  //              {sin(YAW)*cos(PITCH), sin(YAW)*sin(PITCH)*sin(ROLL)+cos(YAW)*cos(ROLL), sin(YAW)*sin(PITCH)*cos(ROLL)-cos(YAW)*sin(ROLL)},
  //              {-sin(PITCH), cos(PITCH)*sin(ROLL), cos(PITCH)*cos(ROLL)}};
  //double Inv_Rot_matrix[3][3] = {{cos(YAW)*cos(PITCH),                              sin(YAW)*cos(PITCH),                             -sin(PITCH) },
  //                               {cos(YAW)*sin(PITCH)*sin(ROLL)-sin(YAW)*cos(ROLL), sin(YAW)*sin(PITCH)*sin(ROLL)+cos(YAW)*cos(ROLL), cos(PITCH)*sin(ROLL) },
  //                               {cos(YAW)*sin(PITCH)*cos(ROLL)+sin(YAW)*sin(ROLL), sin(YAW)*sin(PITCH)*cos(ROLL)-cos(YAW)*sin(ROLL), cos(PITCH)*cos(ROLL) }};
  //Inv_Rot_matrix[3][3] = {{cos(YAW)*cos(PITCH),                              sin(YAW)*cos(PITCH),                             -sin(PITCH) },
  //                        {cos(YAW)*sin(PITCH)*sin(ROLL)-sin(YAW)*cos(ROLL), sin(YAW)*sin(PITCH)*sin(ROLL)+cos(YAW)*cos(ROLL), cos(PITCH)*sin(ROLL) },
  //                        {cos(YAW)*sin(PITCH)*cos(ROLL)+sin(YAW)*sin(ROLL), sin(YAW)*sin(PITCH)*cos(ROLL)-cos(YAW)*sin(ROLL), cos(PITCH)*cos(ROLL) }};
  Inv_Rot_matrix[0][0] = cos(YAW)*cos(PITCH);
  Inv_Rot_matrix[0][1] = sin(YAW)*cos(PITCH);
  Inv_Rot_matrix[0][2] = -sin(PITCH);
  Inv_Rot_matrix[1][0] = cos(YAW)*sin(PITCH)*sin(ROLL)-sin(YAW)*cos(ROLL);
  Inv_Rot_matrix[1][1] = sin(YAW)*sin(PITCH)*sin(ROLL)+cos(YAW)*cos(ROLL);
  Inv_Rot_matrix[1][2] = cos(PITCH)*sin(ROLL);
  Inv_Rot_matrix[2][0] = cos(YAW)*sin(PITCH)*cos(ROLL)+sin(YAW)*sin(ROLL);
  Inv_Rot_matrix[2][1] = sin(YAW)*sin(PITCH)*cos(ROLL)-cos(YAW)*sin(ROLL);
  Inv_Rot_matrix[2][2] = cos(PITCH)*cos(ROLL);
}

void PrepareExtrinsicMat(double* front_param,double* rear_param,double* left_param,double* right_param, 
                         double (&front_Mat)[3][3],double (&rear_Mat)[3][3],double (&left_Mat)[3][3],double (&right_Mat)[3][3])
{
  getInvRotMat(front_param[0],front_param[1],front_param[2],front_Mat);
  getInvRotMat(rear_param[0],rear_param[1],rear_param[2],rear_Mat);
  getInvRotMat(left_param[0],left_param[1],left_param[2],left_Mat);
  getInvRotMat(right_param[0],right_param[1],right_param[2],right_Mat);
  
  std::cout << "Front Mat" << std::endl;
  printMat(front_Mat);
  std::cout << "Rear Mat" << std::endl;
  printMat(rear_Mat);
  std::cout << "Left Mat" << std::endl;
  printMat(left_Mat);
  std::cout << "Right Mat" << std::endl;
  printMat(right_Mat);
}

void seg2rgb(cv::Mat input_img, cv::Mat& output_img, cv::Mat& output_img_gray) {
    for (int i = 0 ; i < input_img.rows ; i++) {
        for (int j = 0 ; j < input_img.cols ; j++) {
            // if ((int)input_img.at<uchar>(i, j) != 0 && 
            //     (int)input_img.at<uchar>(i, j) != 1 && 
            //     (int)input_img.at<uchar>(i, j) != 2 && 
            //     (int)input_img.at<uchar>(i, j) != 3 &&
            //     (int)input_img.at<uchar>(i, j) != 4 && 
            //     (int)input_img.at<uchar>(i, j) != 5 &&
            //     (int)input_img.at<uchar>(i, j) != 6 &&
            //     (int)input_img.at<uchar>(i, j) != 9 &&
            //     (int)input_img.at<uchar>(i, j) != 10 &&
            //     (int)input_img.at<uchar>(i, j) != 13 &&
            //     (int)input_img.at<uchar>(i, j) != 14 &&
            //     (int)input_img.at<uchar>(i, j) != 15)
            //     cout << (int)input_img.at<uchar>(i, j) << " " ;
// Segmentation: 14 classes
// (ID and description)
// 0 : background
// 1 : lane - solid, dashed, dotted
// 2 : lane - parking, stop, arrow, etc
// 3 / 연석? 도로 경계?
// 4 : vehicle - all types
// 6 : wheel
// 9 : general - cone, curbstone, parking block, etc
// 10: cycle, bicyclist, motorcyclist
// 14: pedestrian
// 15: freespace
// 17: parking space
// 18: crosswalk
// 19: speed bump
// 20: foot
// 21: head

/* Updated ID 
#define SEG_BACKGROUND            0
#define SEG_ROAD_EDGE             1 
#define SEG_LANE_SOLID            2 
#define SEG_LANE_DOTTED           3 
#define SEG_LANE_DASHED           4 
#define SEG_LANE_MARKER           5 
#define SEG_VEHICLE               6     [occ]
#define SEG_CART                  7     [occ]
#define SEG_WHEEL                 8     [occ]
#define SEG_WHEEL_FRONT           9     [occ]
#define SEG_WHEEL_REAR            10    [occ]
#define SEG_GENERAL               11
#define SEG_BICYCLE               12    [occ]
#define SEG_MOTORCYCLE            13    [occ]
#define SEG_BICYCLIST             14    [occ]
#define SEG_MOTORCYCLIST          15    [occ]
#define SEG_PEDESTRIAN            16    [occ]
#define SEG_FREESPACE             17
#define SEG_FREESPACE_HOLE        18    
#define SEG_BOUNDARY              19    [occ]
#define SEG_STROLLER              20    [occ]
#define SEG_STOPPER               21    [occ]
#define SEG_ANIMAL                22    [occ]
#define SEG_CONSTRUCTION          23    [occ]
#define SEG_INVALID               24
#define SEG_CLASSES_NUM           25
*/

            if ((int)input_img.at<uchar>(i, j) == 6 || (int)input_img.at<uchar>(i, j) == 7 ||
            (int)input_img.at<uchar>(i, j) == 8 || (int)input_img.at<uchar>(i, j) == 9 ||
            (int)input_img.at<uchar>(i, j) == 10 || (int)input_img.at<uchar>(i, j) == 12 ||
            (int)input_img.at<uchar>(i, j) == 13 || (int)input_img.at<uchar>(i, j) == 14 ||
            (int)input_img.at<uchar>(i, j) == 15 || (int)input_img.at<uchar>(i, j) == 16 ||
            (int)input_img.at<uchar>(i, j) == 21 || (int)input_img.at<uchar>(i, j) == 22 ||
            (int)input_img.at<uchar>(i, j) == 23 || (int)input_img.at<uchar>(i, j) == 20 ||
            (int)input_img.at<uchar>(i, j) == 19
            ) {
                output_img_gray.at<uchar>(i, j) = 255;

                if ((int)input_img.at<uchar>(i, j) == 17) {
                    output_img_gray.at<uchar>(i, j) = 17;
                }
            }
            else
                output_img_gray.at<uchar>(i, j) = 0;

            switch((int)input_img.at<uchar>(i, j)){
                case 0 : // 
                    output_img.at<Vec3b>(i, j)[0] = 78;
                    output_img.at<Vec3b>(i, j)[1] = 56;
                    output_img.at<Vec3b>(i, j)[2] = 24;
                break; 
                case 1: //SEG_ROAD_EDGE 
                case 2: //SEG_LANE_SOLID
                case 3: //SEG_LANE_DOTTED
                case 4: //SEG_LANE_DASHED
                case 5: //SEG_LANE_MARKER
                    output_img.at<Vec3b>(i, j)[0] = 255;
                    output_img.at<Vec3b>(i, j)[1] = 255;
                    output_img.at<Vec3b>(i, j)[2] = 255;
                break;   
                case 6: //SEG_VEHICLE
                case 7: //SEG_CART
                case 20: //SEG_STROLLER
                    output_img.at<Vec3b>(i, j)[0] = 128;
                    output_img.at<Vec3b>(i, j)[1] = 128;
                    output_img.at<Vec3b>(i, j)[2] = 128;
                break;  
                case 8: //SEG_WHEEL
                case 9: //SEG_WHEEL_FRONT
                case 10: //SEG_WHEEL_REAR
                    output_img.at<Vec3b>(i, j)[0] = 255;
                    output_img.at<Vec3b>(i, j)[1] = 255;
                    output_img.at<Vec3b>(i, j)[2] = 0;
                break;  
                case 11: //SEG_GENERAL
                case 12: //SEG_BICYCLE
                case 13: //SEG_MOTORCYCLE
                    output_img.at<Vec3b>(i, j)[0] = 255;
                    output_img.at<Vec3b>(i, j)[1] = 0;
                    output_img.at<Vec3b>(i, j)[2] = 255;
                break;  
                case 14: //SEG_BICYCLIST
                case 15: //SEG_MOTORCYCLIST
                case 16: //SEG_PEDESTRIAN
                    output_img.at<Vec3b>(i, j)[0] = 0;
                    output_img.at<Vec3b>(i, j)[1] = 0;
                    output_img.at<Vec3b>(i, j)[2] = 255;
                break;  
                case 17: //SEG_FREESPACE - Parking Spot
                    output_img.at<Vec3b>(i, j)[0] = 0;
                    output_img.at<Vec3b>(i, j)[1] = 255;
                    output_img.at<Vec3b>(i, j)[2] = 0;
                case 18: //SEG_FREESPACE_HOLE - Free Space
                    output_img.at<Vec3b>(i, j)[0] = 128;
                    output_img.at<Vec3b>(i, j)[1] = 255;
                    output_img.at<Vec3b>(i, j)[2] = 0;
                break;  
                case 19 : //SEG_BOUNDARY
                    output_img.at<Vec3b>(i, j)[0] = 0;
                    output_img.at<Vec3b>(i, j)[1] = 255;
                    output_img.at<Vec3b>(i, j)[2] = 128;
                break;  
                case 21:  //SEG_STOPPER
                    output_img.at<Vec3b>(i, j)[0] = 255;
                    output_img.at<Vec3b>(i, j)[1] = 0;
                    output_img.at<Vec3b>(i, j)[2] = 0;
                break;  
                default :
                    output_img.at<Vec3b>(i, j)[0] = 0;
                    output_img.at<Vec3b>(i, j)[1] = 0;
                    output_img.at<Vec3b>(i, j)[2] = 0;
            }
        }
    }
    // cout << endl;
}

void preProcessing(cv::Mat input_img, cv::Mat& output_img) {
    // //resize, bagfile_size: (512, 288) * 3.75 => (1920, 1080)
    // cv::resize( input_img, input_img, cv::Size(m_CROP_ROI_WIDTH, m_CROP_ROI_HEIGHT), 0, 0, cv::INTER_LINEAR );
    // // padding
    // cv::Mat temp;

    // //padding 1920 1208
    // if (m_PADDING_UP)
    //     cv::copyMakeBorder(input_img, temp, m_PADDING_VALUE, 0, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
    // else
    //     cv::copyMakeBorder(input_img, temp, 0, m_PADDING_VALUE, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
    // cv::resize( temp, output_img, cv::Size(m_FULL_IMG_RESOL_WIDTH, m_FULL_IMG_RESOL_HEIGHT), 0, 0, cv::INTER_LINEAR );

    // //resize, bagfile_size: (512, 288) * 3.75 => (1920, 1080)
    // cv::resize( input_img, input_img, cv::Size(m_CROP_ROI_WIDTH, m_CROP_ROI_HEIGHT), 0, 0, cv::INTER_LINEAR );
    // // padding
    // cv::Mat temp;
    // cv::copyMakeBorder(input_img, temp, m_PADDING_VALUE - 4, 0, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
    // cv::copyMakeBorder(temp, temp, 0, 4, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
    // cv::resize( temp, output_img, cv::Size(m_FULL_IMG_RESOL_WIDTH, m_FULL_IMG_RESOL_HEIGHT), 0, 0, cv::INTER_LINEAR );

    cv::Size input_img_size = input_img.size();
    //resize, bagfile_size => (1920, 1080)
    cv::resize( input_img, input_img, cv::Size(m_CROP_ROI_WIDTH, m_CROP_ROI_HEIGHT), 0, 0, cv::INTER_LINEAR );
    // padding
    cv::Mat temp;
    // cv::copyMakeBorder(input_img, temp, 0, 128, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
    cv::copyMakeBorder(input_img, temp, 0, 128, 4, 4, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
    
    cv::resize( temp, output_img, cv::Size(m_FULL_IMG_RESOL_WIDTH, m_FULL_IMG_RESOL_HEIGHT), 0, 0, cv::INTER_LINEAR );

}

XY_coord InvProjGND(double X_W,double Y_W,double ROLL,double PITCH, double YAW, double World_CamX,double World_CamY,double World_CamZ, struct ocam_model *myocam_model,
  const int& direction_mode)
{
  double a,b,c;
  double World_vec[3];
  double Cam_vec[3];
  double Unit_vector_Cam[3];

  World_vec[0] = (X_W - World_CamX) / ( -World_CamZ );  //  => a / c
  World_vec[1] = (Y_W - World_CamY) / ( -World_CamZ );  //  => b / c

  double temp = World_vec[0] * World_vec[0] + World_vec[1] * World_vec[1];   //  ( a^2 + b^2 / c^2 )  = (1 - c^2 / c^2 )
  World_vec[2] = -1.0 / sqrt (temp + 1.0);              // c < 0 ( ground)
  World_vec[0] = World_vec[0] * World_vec[2];
  World_vec[1] = World_vec[1] * World_vec[2];

//  if(YAW == -87.631* M_DEG2RAD)
  if (direction_mode == RIGHT)
  {
    Unit_vector_Cam[0] = Right_Inv_Rot_Matrix[0][0] * World_vec[0] + Right_Inv_Rot_Matrix[0][1] * World_vec[1] + Right_Inv_Rot_Matrix[0][2] * World_vec[2];
    Unit_vector_Cam[1] = Right_Inv_Rot_Matrix[1][0] * World_vec[0] + Right_Inv_Rot_Matrix[1][1] * World_vec[1] + Right_Inv_Rot_Matrix[1][2] * World_vec[2];
    Unit_vector_Cam[2] = Right_Inv_Rot_Matrix[2][0] * World_vec[0] + Right_Inv_Rot_Matrix[2][1] * World_vec[1] + Right_Inv_Rot_Matrix[2][2] * World_vec[2];
  }
//  else if(YAW == 91.702* M_DEG2RAD){
  else if (direction_mode == LEFT){

    Unit_vector_Cam[0] = Left_Inv_Rot_Matrix[0][0] * World_vec[0] + Left_Inv_Rot_Matrix[0][1] * World_vec[1] + Left_Inv_Rot_Matrix[0][2] * World_vec[2];
    Unit_vector_Cam[1] = Left_Inv_Rot_Matrix[1][0] * World_vec[0] + Left_Inv_Rot_Matrix[1][1] * World_vec[1] + Left_Inv_Rot_Matrix[1][2] * World_vec[2];
    Unit_vector_Cam[2] = Left_Inv_Rot_Matrix[2][0] * World_vec[0] + Left_Inv_Rot_Matrix[2][1] * World_vec[1] + Left_Inv_Rot_Matrix[2][2] * World_vec[2];
  }
//  else if(YAW == 1.462* M_DEG2RAD){
  else if (direction_mode == FRONT){

    Unit_vector_Cam[0] = Front_Inv_Rot_Matrix[0][0] * World_vec[0] + Front_Inv_Rot_Matrix[0][1] * World_vec[1] + Front_Inv_Rot_Matrix[0][2] * World_vec[2];
    Unit_vector_Cam[1] = Front_Inv_Rot_Matrix[1][0] * World_vec[0] + Front_Inv_Rot_Matrix[1][1] * World_vec[1] + Front_Inv_Rot_Matrix[1][2] * World_vec[2];
    Unit_vector_Cam[2] = Front_Inv_Rot_Matrix[2][0] * World_vec[0] + Front_Inv_Rot_Matrix[2][1] * World_vec[1] + Front_Inv_Rot_Matrix[2][2] * World_vec[2];
  }
//  else if(YAW == 179.773* M_DEG2RAD){
  else{

    Unit_vector_Cam[0] = Rear_Inv_Rot_Matrix[0][0] * World_vec[0] + Rear_Inv_Rot_Matrix[0][1] * World_vec[1] + Rear_Inv_Rot_Matrix[0][2] * World_vec[2];
    Unit_vector_Cam[1] = Rear_Inv_Rot_Matrix[1][0] * World_vec[0] + Rear_Inv_Rot_Matrix[1][1] * World_vec[1] + Rear_Inv_Rot_Matrix[1][2] * World_vec[2];
    Unit_vector_Cam[2] = Rear_Inv_Rot_Matrix[2][0] * World_vec[0] + Rear_Inv_Rot_Matrix[2][1] * World_vec[1] + Rear_Inv_Rot_Matrix[2][2] * World_vec[2];
  }

  double m[2];
  double Cam_vec_new[3] = {-Unit_vector_Cam[2], -Unit_vector_Cam[1], -Unit_vector_Cam[0]};          // first flip robot coord 2 cam coord

  world2cam(m, Cam_vec_new, myocam_model);

  XY_coord xy;
  xy.y= int(m[0]);    //rows
  xy.x= int(m[1]);    //cols

  return xy;    // ( cols, rows)
}

void fill_avm_img(cv::Mat input_img, cv::Mat& output_img, int col, int row, 
                    double x_,double y_, double *Dir_param, ocam_model *model, 
                    bool gray, int Direction_Mode){
    XY_coord xy;
    xy = InvProjGND(x_, y_, Dir_param[0], Dir_param[1], Dir_param[2], Dir_param[3], Dir_param[4] ,Dir_param[5], model, Direction_Mode);
    
    if(((xy.x < m_FULL_IMG_RESOL_WIDTH) && (xy.x >= 0)) && ((xy.y < m_FULL_IMG_RESOL_HEIGHT) && (xy.y >= 0))) {
        if(gray)
            output_img.at<uint8_t>(int(row), int(col)) = static_cast<uint8_t>(input_img.at<uchar>(xy.y ,xy.x));
        else{
            output_img.at<cv::Vec3b>(int(row), int(col))[0] = static_cast<uint8_t>(input_img.at<cv::Vec3b>(xy.y, xy.x)[0]);
            output_img.at<cv::Vec3b>(int(row), int(col))[1] = static_cast<uint8_t>(input_img.at<cv::Vec3b>(xy.y, xy.x)[1]);
            output_img.at<cv::Vec3b>(int(row), int(col))[2] = static_cast<uint8_t>(input_img.at<cv::Vec3b>(xy.y, xy.x)[2]);
            // cout << (int)(input_img.at<cv::Vec3b>(xy.y, xy.x)[0]) << " " << (int)(input_img.at<cv::Vec3b>(xy.y, xy.x)[1]) << " " << (int)(input_img.at<cv::Vec3b>(xy.y, xy.x)[2]) << endl;
            // double y = static_cast<uint8_t>(input_img.at<cv::Vec3b>(xy.y, xy.x)[0]);
            // double cb = static_cast<uint8_t>(input_img.at<cv::Vec3b>(xy.y, xy.x)[1]);
            // double cr = static_cast<uint8_t>(input_img.at<cv::Vec3b>(xy.y, xy.x)[2]);
            // output_img.at<cv::Vec3b>(int(row), int(col))[0] = y + 1.403*(cr-128);
            // output_img.at<cv::Vec3b>(int(row), int(col))[1] = y + -0.714*(cb-128)-0.344*(cr-128);
            // output_img.at<cv::Vec3b>(int(row), int(col))[2] = y + 1.773*(cb-128);
        }
    }
}

void Inverse_Warping(cv::Mat input_img, cv::Mat& output_img, int Direction_Mode, bool gray) {
    cv::Mat Processed_img, roi_polygon_img;  /// for 4 images
    preProcessing(input_img, Processed_img);
    if (gray) {
        output_img = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC1);
        roi_polygon_img = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC1);
    }
    else {
        output_img = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3);
        roi_polygon_img = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3);
    }
     /// for 4 images
    cv::Point pt_set[6];
    cv::Point pt_side_set[6];

    // for pt_set_side
    //ptA------ptB---ptC------ptD
    //---------------------------
    //---------------------------
    //---------------------------
    //---------------------------
    //---------------------------
    //---------------------------
    //---------------------------
    //ptE------ptF----ptG-----ptH
    //cv::Point ptA = cv::Point(0,                    m_front_rear_dist);
    //cv::Point ptB = cv::Point(AVM_IMG_WIDTH-1,    m_front_rear_dist);
    //cv::Point ptC = cv::Point((int)(AVM_IMG_WIDTH*0.5), (int)(AVM_IMG_HEIGHT*0.5)-10);
    //cv::Point ptD = cv::Point((int)(AVM_IMG_WIDTH*0.5), (int)(AVM_IMG_HEIGHT*0.5)+10);
    //cv::Point ptE = cv::Point(0,                AVM_IMG_HEIGHT-m_front_rear_dist-1);
    //cv::Point ptF = cv::Point(AVM_IMG_WIDTH-1,    AVM_IMG_HEIGHT-m_front_rear_dist-1);
    int side_gain = (int)(m_AVM_IMG_WIDTH*0.07);
    cv::Point pt_side_A = cv::Point(0, 0);
    cv::Point pt_side_B = cv::Point((int)(m_AVM_IMG_WIDTH/2)-side_gain, 0);
    cv::Point pt_side_C = cv::Point((int)(m_AVM_IMG_WIDTH/2)+side_gain, 0);
    cv::Point pt_side_D = cv::Point((int)(m_AVM_IMG_WIDTH), 0);
    cv::Point pt_side_E = cv::Point(0                                   , m_AVM_IMG_HEIGHT-1);
    cv::Point pt_side_F = cv::Point((int)(m_AVM_IMG_WIDTH/2)-side_gain, m_AVM_IMG_HEIGHT-1);
    cv::Point pt_side_G = cv::Point((int)(m_AVM_IMG_WIDTH/2)+side_gain, m_AVM_IMG_HEIGHT-1);
    cv::Point pt_side_H = cv::Point((int)(m_AVM_IMG_WIDTH)            , m_AVM_IMG_HEIGHT-1) ;
    
    

    
    //----ptA---------ptB------
    //--------\------/---------
    //---------\----/----------
    //-----------ptC-----------
    //-----------ptD-----------
    //----------/---\----------
    //---------/-----\---------
    //--------/-------\--------
    //----ptE---------ptF------
    // cv::Point ptA = cv::Point((int)(m_AVM_IMG_WIDTH/m_front_rear_dist), 0);
    // cv::Point ptB = cv::Point((int)((m_front_rear_dist-1)*m_AVM_IMG_WIDTH/m_front_rear_dist), 0);
    // cv::Point ptC = cv::Point((int)(m_AVM_IMG_WIDTH*0.5), (int)(m_AVM_IMG_HEIGHT*0.5)-10);
    // cv::Point ptD = cv::Point((int)(m_AVM_IMG_WIDTH*0.5), (int)(m_AVM_IMG_HEIGHT*0.5)+10);
    // cv::Point ptE = cv::Point((int)(m_AVM_IMG_WIDTH/m_front_rear_dist), m_AVM_IMG_HEIGHT);
    // cv::Point ptF = cv::Point((int)((m_front_rear_dist-1)*m_AVM_IMG_WIDTH/m_front_rear_dist), m_AVM_IMG_HEIGHT);
    //---------------------
    //ptA-------------/-ptB
    //----\---------/------
    //------\--ptC---------
    //---------ptD---------
    //------/------\-------
    //----/-----------\----
    //ptE---------------ptF
    //---------------------
    // cv::Point ptA = cv::Point(0,                    m_front_rear_dist);
    // cv::Point ptB = cv::Point(m_AVM_IMG_WIDTH-1,    m_front_rear_dist);
    // cv::Point ptC = cv::Point((int)(m_AVM_IMG_WIDTH*0.5), (int)(m_AVM_IMG_HEIGHT*0.5)-10);
    // cv::Point ptD = cv::Point((int)(m_AVM_IMG_WIDTH*0.5), (int)(m_AVM_IMG_HEIGHT*0.5)+10);
    // cv::Point ptE = cv::Point(0,                m_AVM_IMG_HEIGHT-m_front_rear_dist-1);
    // cv::Point ptF = cv::Point(m_AVM_IMG_WIDTH-1,    m_AVM_IMG_HEIGHT-m_front_rear_dist-1);
    // /// for 4 images
    
    //---------------------
    //ptA-------------/-ptB
    //----\---------/------
    //------\ptD--ptC---------
    //-------ptH--ptG-----
    //------/------\-------
    //----/-----------\----
    //ptE---------------ptF
    //---------------------
    //cv::Point ptA = cv::Point(0,                    m_front_rear_dist);
    //cv::Point ptB = cv::Point(AVM_IMG_WIDTH-1,    m_front_rear_dist);
    //cv::Point ptC = cv::Point((int)(AVM_IMG_WIDTH*0.5), (int)(AVM_IMG_HEIGHT*0.5)-10);
    //cv::Point ptD = cv::Point((int)(AVM_IMG_WIDTH*0.5), (int)(AVM_IMG_HEIGHT*0.5)+10);
    //cv::Point ptE = cv::Point(0,                AVM_IMG_HEIGHT-m_front_rear_dist-1);
    //cv::Point ptF = cv::Point(AVM_IMG_WIDTH-1,    AVM_IMG_HEIGHT-m_front_rear_dist-1);
    cv::Point ptA = cv::Point(10,                    0);
    cv::Point ptA_ = cv::Point(0,                   10);
    cv::Point ptB = cv::Point(m_AVM_IMG_WIDTH-11,    0);
    cv::Point ptB_ = cv::Point(m_AVM_IMG_WIDTH-1,    10);
    cv::Point ptC = cv::Point((int)(m_AVM_IMG_WIDTH*0.5)+(int)(1.0f/m_METER_PER_PIXEL)*1.0, (int)(m_AVM_IMG_HEIGHT*0.5)-(int)(M_front_param[3]/m_METER_PER_PIXEL));
    cv::Point ptD = cv::Point((int)(m_AVM_IMG_WIDTH*0.5)-(int)(1.0f/m_METER_PER_PIXEL)*1.0, (int)(m_AVM_IMG_HEIGHT*0.5)-(int)(M_front_param[3]/m_METER_PER_PIXEL));
    cv::Point ptE = cv::Point(10,                m_AVM_IMG_HEIGHT-1);
    cv::Point ptF = cv::Point(m_AVM_IMG_WIDTH-11,    m_AVM_IMG_HEIGHT-1);
    cv::Point ptE_ = cv::Point(0,                m_AVM_IMG_HEIGHT-11);
    cv::Point ptF_ = cv::Point(m_AVM_IMG_WIDTH-1,    m_AVM_IMG_HEIGHT-11);
    cv::Point ptG = cv::Point((int)(m_AVM_IMG_WIDTH*0.5)+(int)(1.0f/m_METER_PER_PIXEL)*1.0, (int)(m_AVM_IMG_HEIGHT*0.5)-(int)(M_back_param[3]/m_METER_PER_PIXEL));
    cv::Point ptH = cv::Point((int)(m_AVM_IMG_WIDTH*0.5)-(int)(1.0f/m_METER_PER_PIXEL)*1.0, (int)(m_AVM_IMG_HEIGHT*0.5)-(int)(M_back_param[3]/m_METER_PER_PIXEL));


    // /// for 4 images
    // // //OD-----------AA-------OC
    // // //-----------|----------
    // // //-----------|----------
    // // //-------ptD-CD-ptC------
    // // //-------ptH----ptG------
    // // //------/--------\-------
    // // //----/-------------\----
    // // //ptE-----------------ptF
    // // //-----------------------

    // cv::Point ptAA = cv::Point((int)(m_AVM_IMG_WIDTH*0.5),                    0);
    // cv::Point ptCD = cv::Point((int)(m_AVM_IMG_WIDTH*0.5),                    (int)(m_AVM_IMG_HEIGHT*0.5)-(int)(M_front_param[3]/m_METER_PER_PIXEL));
    // cv::Point ptOD = cv::Point(0,                    0);
    // cv::Point ptOC = cv::Point(m_AVM_IMG_WIDTH-1,                    0);

    // cv::Point ptA = cv::Point(10,                    0);
    // cv::Point ptA_ = cv::Point(0,                   10);
    // cv::Point ptB = cv::Point(m_AVM_IMG_WIDTH-11,    0);
    // cv::Point ptB_ = cv::Point(m_AVM_IMG_WIDTH-1,    10);
    // cv::Point ptC = cv::Point((int)(m_AVM_IMG_WIDTH*0.5)+(int)(1.0f/m_METER_PER_PIXEL), (int)(m_AVM_IMG_HEIGHT*0.5)-(int)(M_front_param[3]/m_METER_PER_PIXEL));
    // cv::Point ptD = cv::Point((int)(m_AVM_IMG_WIDTH*0.5)-(int)(1.0f/m_METER_PER_PIXEL), (int)(m_AVM_IMG_HEIGHT*0.5)-(int)(M_front_param[3]/m_METER_PER_PIXEL));
    // cv::Point ptE = cv::Point(10,                m_AVM_IMG_HEIGHT-1);
    // cv::Point ptF = cv::Point(m_AVM_IMG_WIDTH-11,    m_AVM_IMG_HEIGHT-1);
    // cv::Point ptE_ = cv::Point(0,                m_AVM_IMG_HEIGHT-11);
    // cv::Point ptF_ = cv::Point(m_AVM_IMG_WIDTH-1,    m_AVM_IMG_HEIGHT-11);
    // cv::Point ptG = cv::Point((int)(m_AVM_IMG_WIDTH*0.5)+(int)(1.0f/m_METER_PER_PIXEL), (int)(m_AVM_IMG_HEIGHT*0.5)-(int)(M_back_param[3]/m_METER_PER_PIXEL));
    // cv::Point ptH = cv::Point((int)(m_AVM_IMG_WIDTH*0.5)-(int)(1.0f/m_METER_PER_PIXEL), (int)(m_AVM_IMG_HEIGHT*0.5)-(int)(M_back_param[3]/m_METER_PER_PIXEL));





    // cv::Point ptA = cv::Point(0,                    m_front_rear_dist);
    // cv::Point ptA_ = cv::Point(0,                    m_front_rear_dist+3);
    // cv::Point ptB = cv::Point(m_AVM_IMG_WIDTH-1,    m_front_rear_dist);
    // cv::Point ptB_ = cv::Point(m_AVM_IMG_WIDTH-1,    m_front_rear_dist+3);
    // cv::Point ptC = cv::Point((int)(m_AVM_IMG_WIDTH*0.5), (int)(m_AVM_IMG_HEIGHT*0.5)-10);
    // cv::Point ptD = cv::Point((int)(m_AVM_IMG_WIDTH*0.5), (int)(m_AVM_IMG_HEIGHT*0.5)+10);
    // cv::Point ptE = cv::Point(0,                m_AVM_IMG_HEIGHT-m_front_rear_dist-1);
    // cv::Point ptE_ = cv::Point(0,                m_AVM_IMG_HEIGHT-m_front_rear_dist-5);
    // cv::Point ptF = cv::Point(m_AVM_IMG_WIDTH-1,    m_AVM_IMG_HEIGHT-m_front_rear_dist-1);
    // cv::Point ptF_ = cv::Point(m_AVM_IMG_WIDTH-1,    m_AVM_IMG_HEIGHT-m_front_rear_dist-5);

    double x_, y_;
    // Camera Intrinsic & Extrinsic Parameter
    double *Dir_param;
    ocam_model *model;
    if(Direction_Mode == FRONT){
        Dir_param = M_front_param;  model = &m_front_model;
        for(int row = 0; row < (m_AVM_IMG_HEIGHT/2)-1 ; row++){
            for(int col = 0; col < m_AVM_IMG_WIDTH ; col++){
                // IMG coord TO WORLD_coord
                y_ = (m_REAL_OCCUPANCY_SIZE_X/2.0) - double(col) * m_METER_PER_PIXEL;
                x_ = (m_REAL_OCCUPANCY_SIZE_Y/2.0) - double(row) * m_METER_PER_PIXEL;
                fill_avm_img(Processed_img, output_img, col, row, x_, y_, Dir_param, model, gray, Direction_Mode);
            }
        }
         /// for 4 images
        pt_set[0] = ptA; pt_set[1] = ptB; pt_set[2] = ptC;
        pt_set[3] = ptD; pt_set[4] = ptD; pt_set[5] = ptD;
    }
    else if(Direction_Mode == REAR){
    // if(Direction_Mode == REAR){
        Dir_param = M_back_param;   model = &m_rear_model;
        for(int row = m_AVM_IMG_HEIGHT-1; row > (m_AVM_IMG_HEIGHT/2) ; row--){
            for(int col = m_AVM_IMG_WIDTH-1; col > 0 ; col--){
                y_ = -(m_REAL_OCCUPANCY_SIZE_X/2.0) + double(m_AVM_IMG_WIDTH - col) * m_METER_PER_PIXEL;
                x_ = -(m_REAL_OCCUPANCY_SIZE_Y/2.0) + double(m_AVM_IMG_WIDTH - row) * m_METER_PER_PIXEL;
                fill_avm_img(Processed_img, output_img, col, row, x_, y_, Dir_param, model, gray, Direction_Mode);
            }
        }
        /// for 4 images
        pt_set[0] = ptE; pt_set[1] = ptH; pt_set[2] = ptG;
        pt_set[3] = ptF; pt_set[4] = ptF; pt_set[5] = ptF;
    }
    else if(Direction_Mode == LEFT){
        Dir_param = M_left_param;   model = &m_left_model;
        for(int row = m_AVM_IMG_HEIGHT-1; row> 0 ; row--){
            for(int col = 0; col < (m_AVM_IMG_WIDTH/2)-1 ; col++){
                y_ = (m_REAL_OCCUPANCY_SIZE_X/2.0) -  double(col) * m_METER_PER_PIXEL;
                x_ = -(m_REAL_OCCUPANCY_SIZE_Y/2.0) + double(m_AVM_IMG_WIDTH - row) * m_METER_PER_PIXEL;
                fill_avm_img(Processed_img, output_img, col, row, x_, y_, Dir_param, model, gray, Direction_Mode);
            }
        }
        /// for 4 images
        pt_set[0] = ptA_; pt_set[1] = ptD; pt_set[2] = ptH;
        pt_set[3] = ptE_; pt_set[4] = ptE_; pt_set[5] = ptE_;
        // pt_set[0] = ptAA; pt_set[1] = ptCD; pt_set[2] = ptD;
        // pt_set[3] = ptH; pt_set[4] = ptE_; pt_set[5] = ptOD;

        pt_set[0] = ptA_; pt_set[1] = ptD; pt_set[2] = ptH;
        pt_set[3] = ptE_; pt_set[4] = ptE_; pt_set[5] = ptE_;

        pt_side_set[0] = pt_side_A; pt_side_set[1] = pt_side_B;
        pt_side_set[2] = pt_side_F; pt_side_set[3] = pt_side_E;
        pt_side_set[4] = pt_side_E; pt_side_set[5] = pt_side_E;
    }
    else if(Direction_Mode == RIGHT){
        Dir_param = M_right_param;  model = &m_right_model;
        for(int row = 0; row< m_AVM_IMG_HEIGHT ; row++){
            for(int col = m_AVM_IMG_WIDTH-1; col > (m_AVM_IMG_WIDTH/2)-1 ; col--){
                y_ = -(m_REAL_OCCUPANCY_SIZE_X/2.0) + double(m_AVM_IMG_WIDTH - col) * m_METER_PER_PIXEL;
                x_ = (m_REAL_OCCUPANCY_SIZE_Y/2.0)  - double(row) * m_METER_PER_PIXEL;
                fill_avm_img(Processed_img, output_img, col, row, x_, y_, Dir_param, model, gray, Direction_Mode);
            }
        }
        /// for 4 images
        pt_set[0] = ptB_; pt_set[1] = ptC; pt_set[2] = ptG;
        pt_set[3] = ptF_; pt_set[4] = ptF_; pt_set[5] = ptF_;
        // pt_set[0] = ptAA; pt_set[1] = ptCD; pt_set[2] = ptC;
        // pt_set[3] = ptG; pt_set[4] = ptF_; pt_set[5] = ptOC;


        pt_side_set[0] = pt_side_C; pt_side_set[1] = pt_side_D;
        pt_side_set[2] = pt_side_H; pt_side_set[3] = pt_side_G;
        pt_side_set[4] = pt_side_G; pt_side_set[5] = pt_side_G;

    }
     /// for 4 images
    if (m_ALL_IMAGE_FLAG) {
        const cv::Point* ppt[1] = { pt_set };
        int npt[] = { 6 };
        cv::fillPoly(roi_polygon_img, ppt, npt, 1, Scalar(255, 255, 255), 8);
        cv::bitwise_and(output_img, roi_polygon_img, output_img);
    }
    else if (!m_ALL_IMAGE_FLAG && !m_flagParking) {
        const cv::Point* ppt[1] = { pt_side_set };
        int npt[] = { 6 };
        cv::fillPoly(roi_polygon_img, ppt, npt, 1, Scalar(255, 255, 255), 8);
        cv::bitwise_and(output_img, roi_polygon_img, output_img);
    }
}

// void Inverse_Warping(cv::Mat input_img, cv::Mat& output_img, int Direction_Mode, bool gray) {
//     cv::Mat Processed_img, roi_polygon_img;  /// for 4 images
//     preProcessing(input_img, Processed_img);
//     if (gray) {
//         output_img = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC1);
//         roi_polygon_img = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC1);
//     }
//     else {
//         output_img = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3);
//         roi_polygon_img = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3);
//     }

//      /// for 4 images
//     cv::Point pt_set[6];
//     //----ptA---------ptB------
//     //--------\------/---------
//     //---------\----/----------
//     //-----------ptC-----------
//     //-----------ptD-----------
//     //----------/---\----------
//     //---------/-----\---------
//     //--------/-------\--------
//     //----ptE---------ptF------
//     // cv::Point ptA = cv::Point((int)(m_AVM_IMG_WIDTH/m_front_rear_dist), 0);
//     // cv::Point ptB = cv::Point((int)((m_front_rear_dist-1)*m_AVM_IMG_WIDTH/m_front_rear_dist), 0);
//     // cv::Point ptC = cv::Point((int)(m_AVM_IMG_WIDTH*0.5), (int)(m_AVM_IMG_HEIGHT*0.5)-10);
//     // cv::Point ptD = cv::Point((int)(m_AVM_IMG_WIDTH*0.5), (int)(m_AVM_IMG_HEIGHT*0.5)+10);
//     // cv::Point ptE = cv::Point((int)(m_AVM_IMG_WIDTH/m_front_rear_dist), m_AVM_IMG_HEIGHT);
//     // cv::Point ptF = cv::Point((int)((m_front_rear_dist-1)*m_AVM_IMG_WIDTH/m_front_rear_dist), m_AVM_IMG_HEIGHT);

//     //---------------------
//     //ptA-------------/-ptB
//     //----\---------/------
//     //------\--ptC---------
//     //---------ptD---------
//     //------/------\-------
//     //----/-----------\----
//     //ptE---------------ptF
//     //---------------------
//     cv::Point ptA = cv::Point(0,                    m_front_rear_dist);
//     cv::Point ptA_ = cv::Point(0,                    m_front_rear_dist+3);
//     cv::Point ptB = cv::Point(m_AVM_IMG_WIDTH-1,    m_front_rear_dist);
//     cv::Point ptB_ = cv::Point(m_AVM_IMG_WIDTH-1,    m_front_rear_dist+3);
//     cv::Point ptC = cv::Point((int)(m_AVM_IMG_WIDTH*0.5), (int)(m_AVM_IMG_HEIGHT*0.5)-10);
//     cv::Point ptD = cv::Point((int)(m_AVM_IMG_WIDTH*0.5), (int)(m_AVM_IMG_HEIGHT*0.5)+10);
//     cv::Point ptE = cv::Point(0,                m_AVM_IMG_HEIGHT-m_front_rear_dist-1);
//     cv::Point ptE_ = cv::Point(0,                m_AVM_IMG_HEIGHT-m_front_rear_dist-5);
//     cv::Point ptF = cv::Point(m_AVM_IMG_WIDTH-1,    m_AVM_IMG_HEIGHT-m_front_rear_dist-1);
//     cv::Point ptF_ = cv::Point(m_AVM_IMG_WIDTH-1,    m_AVM_IMG_HEIGHT-m_front_rear_dist-5);
//     /// for 4 images

//     double x_, y_;
//     // Camera Intrinsic & Extrinsic Parameter
//     double *Dir_param;
//     ocam_model *model;

//     if(Direction_Mode == FRONT){
//         Dir_param = M_front_param;  model = &m_front_model;
//         for(int row = 0; row < (m_AVM_IMG_HEIGHT/2)-1 ; row++){
//             for(int col = 0; col < m_AVM_IMG_WIDTH ; col++){
//                 // IMG coord TO WORLD_coord 
//                 y_ = (m_REAL_OCCUPANCY_SIZE_X/2.0) - double(col) * m_METER_PER_PIXEL;
//                 x_ = (m_REAL_OCCUPANCY_SIZE_Y/2.0) - double(row) * m_METER_PER_PIXEL;

//                 fill_avm_img(Processed_img, output_img, col, row, x_, y_, Dir_param, model, gray, Direction_Mode);
//             }
//         }
//          /// for 4 images
//         pt_set[0] = cv::Point(0,0); pt_set[1] = cv::Point(m_AVM_IMG_WIDTH-1,0); pt_set[2] = ptB;
//         pt_set[3] = ptC; pt_set[4] = ptA; pt_set[5] = ptA;
//     }
//     else if(Direction_Mode == REAR){
//         Dir_param = M_back_param;   model = &m_rear_model;
//         for(int row = m_AVM_IMG_HEIGHT-1; row > (m_AVM_IMG_HEIGHT/2) ; row--){
//             for(int col = m_AVM_IMG_WIDTH-1; col > 0 ; col--){
//                 y_ = -(m_REAL_OCCUPANCY_SIZE_X/2.0) + double(m_AVM_IMG_WIDTH - col) * m_METER_PER_PIXEL;
//                 x_ = -(m_REAL_OCCUPANCY_SIZE_Y/2.0) + double(m_AVM_IMG_WIDTH - row) * m_METER_PER_PIXEL;

//                 fill_avm_img(Processed_img, output_img, col, row, x_, y_, Dir_param, model, gray, Direction_Mode);
//             }
//         }
//         /// for 4 images
//         pt_set[0] = cv::Point(0,m_AVM_IMG_HEIGHT-1); pt_set[1] = ptE; pt_set[2] = ptD;
//         pt_set[3] = ptF; pt_set[4] = cv::Point(m_AVM_IMG_WIDTH,m_AVM_IMG_HEIGHT-1); pt_set[5] = cv::Point(m_AVM_IMG_WIDTH,m_AVM_IMG_HEIGHT-1);
//     }
//     else if(Direction_Mode == LEFT){
//         Dir_param = M_left_param;   model = &m_left_model;
//         for(int row = m_AVM_IMG_HEIGHT-1; row> 0 ; row--){
//             for(int col = 0; col < (m_AVM_IMG_WIDTH/2)-1 ; col++){
//                 y_ = (m_REAL_OCCUPANCY_SIZE_X/2.0) -  double(col) * m_METER_PER_PIXEL;
//                 x_ = -(m_REAL_OCCUPANCY_SIZE_Y/2.0) + double(m_AVM_IMG_WIDTH - row) * m_METER_PER_PIXEL;

//                 fill_avm_img(Processed_img, output_img, col, row, x_, y_, Dir_param, model, gray, Direction_Mode);
//             }
//         }
//         /// for 4 images
//         pt_set[0] = ptA_; pt_set[1] = ptC; pt_set[2] = ptD;
//         pt_set[3] = ptE_; pt_set[4] = ptE_; pt_set[5] = ptE_;
//     }
//     else if(Direction_Mode == RIGHT){
//         Dir_param = M_right_param;  model = &m_right_model;
//         for(int row = 0; row< m_AVM_IMG_HEIGHT ; row++){
//             for(int col = m_AVM_IMG_WIDTH-1; col > (m_AVM_IMG_WIDTH/2)-1 ; col--){
//                 y_ = -(m_REAL_OCCUPANCY_SIZE_X/2.0) + double(m_AVM_IMG_WIDTH - col) * m_METER_PER_PIXEL;
//                 x_ = (m_REAL_OCCUPANCY_SIZE_Y/2.0)  - double(row) * m_METER_PER_PIXEL;

//                 fill_avm_img(Processed_img, output_img, col, row, x_, y_, Dir_param, model, gray, Direction_Mode);
//             }
//         }
//         /// for 4 images
//         pt_set[0] = ptB_; pt_set[1] = ptC; pt_set[2] = ptD;
//         pt_set[3] = ptF_; pt_set[4] = ptF_; pt_set[5] = ptF_;
//     }

//      /// for 4 images
//     if (m_flagParking && m_ALL_IMAGE_FLAG) {
//         const cv::Point* ppt[1] = { pt_set };
//         int npt[] = { 6 };
//         cv::fillPoly(roi_polygon_img, ppt, npt, 1, Scalar(255, 255, 255), 8);
//         cv::bitwise_and(output_img, roi_polygon_img, output_img);
//     }
// }

// void push_detection_result(const undistorted_top_view::ParkingPhantomnetData::ConstPtr& msg, std::vector<DETECTIONS> *det){
//     DETECTIONS tmp;
//     det->clear();
//     // std::cout<< "size() : " << msg->detections.size() << std::endl;

//     for (int i = 0 ; i < msg->detections.size() ; i++){
//         tmp.classification = msg->detections[i].classification;

//         if((tmp.classification == PARKING_SPACE) && (msg->detections[i].probability > OBJ_DET_CONFIDENCE)) {
//             // std::cout<< "tmp.x" << tmp.x<<std::endl;
//             tmp.probability = msg->detections[i].probability;
//             tmp.x = msg->detections[i].x - ENLARGED_BOX;
//             tmp.y = msg->detections[i].y - ENLARGED_BOX;
//             tmp.width = msg->detections[i].width + 2*ENLARGED_BOX;
//             tmp.height = msg->detections[i].height + 2*ENLARGED_BOX;
//             det->push_back(tmp);
//         }
//     }
//     // std::cout<< "size() : " << det->size() << std::endl;
// }

// Front
void CallbackPhantom_front(const sensor_msgs::ImageConstPtr& msg) 
{
    if (m_ALL_IMAGE_FLAG) {
        if (m_rgb_color){
            cv::Size size(msg->width, msg->height);
            cv::Mat cv_frame_resize_pad = cv::Mat(size, CV_8UC2, (void *)&msg->data[0]);
            cv::Mat cv_rgb;
            cv::cvtColor(cv_frame_resize_pad, cv_rgb, COLOR_YUV2BGR_YUYV);
            Inverse_Warping(cv_rgb, AVM_front, FRONT, false);
        }
        else {
            cv::Mat cv_frame_resize_pad = cv_bridge::toCvCopy(msg, "bgr8" )->image;
            Inverse_Warping(cv_frame_resize_pad, AVM_front, FRONT, false);
        }
    }
}
// Rear
void CallbackPhantom_rear(const sensor_msgs::ImageConstPtr& msg) 
{
    if (m_ALL_IMAGE_FLAG) {
        if (m_rgb_color){
            cv::Size size(msg->width, msg->height);
            cv::Mat cv_frame_resize_pad = cv::Mat(size, CV_8UC2, (void *)&msg->data[0]);
            cv::Mat cv_rgb;
            cv::cvtColor(cv_frame_resize_pad, cv_rgb, COLOR_YUV2BGR_YUYV);
            Inverse_Warping(cv_rgb, AVM_rear, REAR, false);
        }
        else {
            cv::Mat cv_frame_resize_pad = cv_bridge::toCvCopy(msg, "bgr8" )->image;
            Inverse_Warping(cv_frame_resize_pad, AVM_rear, REAR, false);
        }
    }
}
// Left
void CallbackPhantom_left(const sensor_msgs::ImageConstPtr& msg) 
{
    // if (!m_flagParking) {
    if (m_rgb_color){
        cv::Size size(msg->width, msg->height);
        cv::Mat cv_frame_resize_pad = cv::Mat(size, CV_8UC2, (void *)&msg->data[0]);
        cv::Mat cv_rgb;
        cv::cvtColor(cv_frame_resize_pad, cv_rgb, COLOR_YUV2BGR_YUYV);
        Inverse_Warping(cv_rgb, AVM_left, LEFT, false);
    }
    else {
        cv::Mat cv_frame_resize_pad = cv_bridge::toCvCopy(msg, "bgr8" )->image;
        // cvtColor(cv_frame_resize_pad, cv_frame_resize_pad, CV_YCrCb2RGB);
        Inverse_Warping(cv_frame_resize_pad, AVM_left, LEFT, false);
    }
    // }
}
// Right
void CallbackPhantom_right(const sensor_msgs::ImageConstPtr& msg) 
{
    // if (!m_flagParking) {
        if (m_rgb_color){
            cv::Size size(msg->width, msg->height);
            cv::Mat cv_frame_resize_pad = cv::Mat(size, CV_8UC2, (void *)&msg->data[0]);
            cv::Mat cv_rgb;
            cv::cvtColor(cv_frame_resize_pad, cv_rgb, COLOR_YUV2BGR_YUYV);
            Inverse_Warping(cv_rgb, AVM_right, RIGHT, false);
        }
        else {
            cv::Mat cv_frame_resize_pad = cv_bridge::toCvCopy( msg, "bgr8" )->image;
            Inverse_Warping(cv_frame_resize_pad, AVM_right, RIGHT, false);
        }
    // }
}

void CallbackPhantom_seg_front(const undistorted_top_view::ParkingPhantomnetData::ConstPtr& msg) {
    // if (!m_flagParking) {
        // push_detection_result(msg, &Front_detection);
        
        cv::Mat cv_frame_seg = cv_bridge::toCvCopy(msg->segmentation, msg->segmentation.encoding )->image;
        cv::Mat cv_frame_raw_new = cv_bridge::toCvCopy(msg->viz, msg->viz.encoding )->image;
        cv::Mat cv_frame_raw_new_gray = cv_bridge::toCvCopy(msg->viz, msg->segmentation.encoding )->image;
        cv::Mat cv_rgb_pad, cv_gray_pad;
    
        seg2rgb(cv_frame_seg, cv_frame_raw_new, cv_frame_raw_new_gray);
        
        cv::copyMakeBorder(cv_frame_raw_new, cv_rgb_pad, 16, 16, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
        cv::copyMakeBorder(cv_frame_raw_new_gray, cv_gray_pad, 16, 16, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
        
        Inverse_Warping(cv_rgb_pad, AVM_seg_front, FRONT, false);   // seg to classes (color) 
        Inverse_Warping(cv_gray_pad, AVM_seg_front_gray, FRONT, true);  // seg to drivable or non-drivable area (gray)
    // }
}

void CallbackPhantom_seg_rear(const undistorted_top_view::ParkingPhantomnetData::ConstPtr& msg) {
    // if (!m_flagParking) {
        // push_detection_result(msg, &Rear_detection);

        cv::Mat cv_frame_seg = cv_bridge::toCvCopy( msg->segmentation, msg->segmentation.encoding )->image;
        cv::Mat cv_frame_raw_new = cv_bridge::toCvCopy( msg->viz, msg->viz.encoding )->image;
        cv::Mat cv_frame_raw_new_gray = cv_bridge::toCvCopy( msg->viz, msg->segmentation.encoding )->image;
        cv::Mat cv_rgb_pad, cv_gray_pad;
    
        seg2rgb(cv_frame_seg, cv_frame_raw_new, cv_frame_raw_new_gray);
        
        cv::copyMakeBorder(cv_frame_raw_new, cv_rgb_pad, 16, 16, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
        cv::copyMakeBorder(cv_frame_raw_new_gray, cv_gray_pad, 16, 16, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
        
        Inverse_Warping(cv_rgb_pad, AVM_seg_rear, REAR, false);   // seg to classes (color) 
        Inverse_Warping(cv_gray_pad, AVM_seg_rear_gray, REAR, true);  // seg to drivable or non-drivable area (gray)
    // }
}

//Left Segmentation
void CallbackPhantom_seg_left(const undistorted_top_view::ParkingPhantomnetData::ConstPtr& msg) { 
    // if (!m_flagParking) {
        cv::Mat cv_frame_seg = cv_bridge::toCvCopy( msg->segmentation, msg->segmentation.encoding )->image;
        cv::Mat cv_frame_raw_new = cv_bridge::toCvCopy( msg->viz, msg->viz.encoding )->image;
        cv::Mat cv_frame_raw_new_gray = cv_bridge::toCvCopy( msg->viz, msg->segmentation.encoding )->image;
        cv::Mat cv_rgb_pad, cv_gray_pad;
    
        seg2rgb(cv_frame_seg, cv_frame_raw_new, cv_frame_raw_new_gray);
        
        cv::copyMakeBorder(cv_frame_raw_new, cv_rgb_pad, 16, 16, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
        cv::copyMakeBorder(cv_frame_raw_new_gray, cv_gray_pad, 16, 16, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
        
        Inverse_Warping(cv_rgb_pad, AVM_seg_left, LEFT, false);   // seg to classes (color) 
        Inverse_Warping(cv_gray_pad, AVM_seg_left_gray, LEFT, true);  // seg to drivable or non-drivable area (gray)
    // }
}

//Right Segmentation
void CallbackPhantom_seg_right(const undistorted_top_view::ParkingPhantomnetData::ConstPtr& msg) {
    // if (!m_flagParking) {
        cv::Mat cv_frame_seg = cv_bridge::toCvCopy( msg->segmentation, msg->segmentation.encoding )->image;
        cv::Mat cv_frame_raw_new = cv_bridge::toCvCopy( msg->viz, msg->viz.encoding )->image;
        cv::Mat cv_frame_raw_new_gray = cv_bridge::toCvCopy( msg->viz, msg->segmentation.encoding )->image;
        cv::Mat cv_rgb_pad, cv_gray_pad;
    
        seg2rgb(cv_frame_seg, cv_frame_raw_new, cv_frame_raw_new_gray);
        
        cv::copyMakeBorder(cv_frame_raw_new, cv_rgb_pad, 16, 16, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
        cv::copyMakeBorder(cv_frame_raw_new_gray, cv_gray_pad, 16, 16, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
        
        Inverse_Warping(cv_rgb_pad, AVM_seg_right, RIGHT, false);   // seg to classes (color) 
        Inverse_Warping(cv_gray_pad, AVM_seg_right_gray, RIGHT, true);  // seg to drivable or non-drivable area (gray)
    // }
}

void AVMpointCloud(cv::Mat img) {
    int avmCutRange = 0, idxSparse = 1;

    for(int i = avmCutRange ; i < img.size().height - avmCutRange ; i = i+idxSparse){
        for(int j = avmCutRange ; j < img.size().width - avmCutRange ; j = j+idxSparse){
            double x = (m_REAL_OCCUPANCY_SIZE_X / 2.0) - m_METER_PER_PIXEL * i, y = (m_REAL_OCCUPANCY_SIZE_Y/2.0) - m_METER_PER_PIXEL * j, gX, gY;
            Local2Global(x, y, gX, gY);

            pcl::PointXYZRGB pt;
            pt.x = gX;  pt.y = gY;  pt.z = 0.0;

            uint8_t r = static_cast<uint8_t>(img.at<cv::Vec3b>(i,j)[2]), g = static_cast<uint8_t>(img.at<cv::Vec3b>(i,j)[1]), b = static_cast<uint8_t>(img.at<cv::Vec3b>(i,j)[0]);
            uint32_t rgb = ((uint32_t)r << 16 | (uint32_t)g << 8 | (uint32_t)b);
            pt.rgb = *reinterpret_cast<float*>(&rgb);

            m_avm_data->push_back(pt);
        }
    }

    // if (m_flagParking) {
    //     if (m_avm_data->size() > 100000)
    //         m_avm_data->erase(m_avm_data->begin(), m_avm_data->begin() + 10000);
    // } else {
        m_avm_data->erase(m_avm_data->begin(), m_avm_data->begin() + m_avmDRsize);
        m_avmDRsize = m_avm_data->size();
    // }
    
    // cout << m_avm_data->size() << endl;
    Pub_AVM_DR.publish(m_avm_data);
}

void CallbackLocalizationData(const std_msgs::Float32MultiArray::ConstPtr& msg){
    m_car.x = msg->data.at(0);      // x
    m_car.y = msg->data.at(1);      // y
    m_car.th = msg->data.at(2);     // theta
    m_car.vel = msg->data.at(3);    // [m/s]
}

void GetRelCoordsFromCar(double ori_x, double ori_y, double ori_th, double &rel_x, double &rel_y, double &rel_th) {
    double Len = DISTANCE(m_car.x, m_car.y, ori_x, ori_y);
    double alpha = atan2(ori_y - m_car.y, ori_x - m_car.x);
    rel_x = Len * cos(alpha - m_car.th);
    rel_y = Len * sin(alpha - m_car.th);
    rel_th = ori_th - m_car.th;
}

void CallbackParkingGoal(const geometry_msgs::PoseArray::ConstPtr& end) {    //[end] which is the coordinates of the goal
    const int index = m_parking_index;    //0 is cloest
    if (end->poses.size() > 0) {
        m_flag_goal_empty = false;
        GOAL_G[0] = end->poses[index].position.x;
        GOAL_G[1] = end->poses[index].position.y;
        GOAL_G[2] = tf::getYaw(end->poses[index].orientation);

        GOAL_G[0] -= m_goal_bias*cos(GOAL_G[2]);
        GOAL_G[1] -= m_goal_bias*sin(GOAL_G[2]);

        double goalRx = 0.0, goalRy = 0.0, goalRth = 0.0;
        GetRelCoordsFromCar(GOAL_G[0], GOAL_G[1], GOAL_G[2], goalRx, goalRy, goalRth);

        coord(m_PARKING_SPOT_WIDTH/2.0,  -m_PARKING_SPOT_LENGTH/2.0, goalRx, goalRy, goalRth, modx1, mody1);
        coord(m_PARKING_SPOT_WIDTH/2.0,   m_PARKING_SPOT_LENGTH/2.0, goalRx, goalRy, goalRth, modx2, mody2);
        coord(-m_PARKING_SPOT_WIDTH/2.0,  m_PARKING_SPOT_LENGTH/2.0, goalRx, goalRy, goalRth, modx3, mody3);
        coord(-m_PARKING_SPOT_WIDTH/2.0, -m_PARKING_SPOT_LENGTH/2.0, goalRx, goalRy, goalRth, modx4, mody4);

        coord( m_PARKING_SPOT_WIDTH/2.0*m_FREE_ERASE_GAIN, -m_PARKING_SPOT_LENGTH/2.0*m_FREE_ERASE_GAIN, goalRx, goalRy, goalRth, modx1free, mody1free);
        coord( m_PARKING_SPOT_WIDTH/2.0*m_FREE_ERASE_GAIN,  m_PARKING_SPOT_LENGTH/2.0*m_FREE_ERASE_GAIN, goalRx, goalRy, goalRth, modx2free, mody2free);
        coord(-m_PARKING_SPOT_WIDTH/2.0*m_FREE_ERASE_GAIN,  m_PARKING_SPOT_LENGTH/2.0*m_FREE_ERASE_GAIN, goalRx, goalRy, goalRth, modx3free, mody3free);
        coord(-m_PARKING_SPOT_WIDTH/2.0*m_FREE_ERASE_GAIN, -m_PARKING_SPOT_LENGTH/2.0*m_FREE_ERASE_GAIN, goalRx, goalRy, goalRth, modx4free, mody4free);

        coord( m_PARKING_SPOT_WIDTH/2.0*m_FREE_ERASE_GAIN*1.0, -m_PARKING_SPOT_LENGTH/2.0*m_FREE_ERASE_GAIN*1.0, 1.5, 0.0, 0.0, modx1car, mody1car);
        coord( m_PARKING_SPOT_WIDTH/2.0*m_FREE_ERASE_GAIN*1.0,  m_PARKING_SPOT_LENGTH/2.0*m_FREE_ERASE_GAIN*1.0, 1.5, 0.0, 0.0, modx2car, mody2car);
        coord(-m_PARKING_SPOT_WIDTH/2.0*m_FREE_ERASE_GAIN*1.0,  m_PARKING_SPOT_LENGTH/2.0*m_FREE_ERASE_GAIN*1.0, 1.5, 0.0, 0.0, modx3car, mody3car);
        coord(-m_PARKING_SPOT_WIDTH/2.0*m_FREE_ERASE_GAIN*1.0, -m_PARKING_SPOT_LENGTH/2.0*m_FREE_ERASE_GAIN*1.0, 1.5, 0.0, 0.0, modx4car, mody4car);

        // coord( 100, -100, 0.0, 0.0, 0.0, modx1car, mody1car);
        // coord( 100,  100, 0.0, 0.0, 0.0, modx2car, mody2car);
        // coord(-100,  100, 0.0, 0.0, 0.0, modx3car, mody3car);
        // coord(-100, -100, 0.0, 0.0, 0.0, modx4car, mody4car);
    }
    else        m_flag_goal_empty = true;
 
}

void occupancyGridmapPub(cv::Mat img) {
    int** arr_obs;
    bool** arr_free;
    arr_obs  = new int*[occupancyGridMap.info.width];
    arr_free = new bool*[occupancyGridMap.info.width];

    for (int x = 0; x < occupancyGridMap.info.width; x++) {
        arr_obs[x]  = new int[occupancyGridMap.info.height];
        arr_free[x] = new bool[occupancyGridMap.info.height];
    } 

    for(int i = 0; i < occupancyGridMap.info.width ; i++) 
        for(int j = 0; j < occupancyGridMap.info.height; j++) {
        arr_obs[i][j]  = 0;
        arr_free[i][j] = false;
    }

    for(int row = 0 ; row < m_AVM_IMG_WIDTH ; row++) 
        for(int col = 0 ; col < m_AVM_IMG_HEIGHT ; col++) {
            // WORLD_coord TO IMG coord
            double y_ = (m_REAL_OCCUPANCY_SIZE_X/2.0) - double(col)*m_METER_PER_PIXEL;
            double x_ = (m_REAL_OCCUPANCY_SIZE_Y/2.0) - double(row)*m_METER_PER_PIXEL;

            int arrX, arrY;
            real2arr(y_, x_, arrY, arrX);
            if ((int)img.at<uint8_t>(int(row), int(col)) == 255 || (int)img.at<uint8_t>(int(row), int(col)) == 17) {
                arr_obs[arrX][arrY]++;
                if ((int)img.at<uint8_t>(int(row), int(col)) == 17)
                    arr_free[arrX][arrY] = true;
            }
        }

    int cnt = 0;
    occupancyGridMap.data.clear();
    occupancyGridMap.data.resize(occupancyGridMap.info.width*occupancyGridMap.info.height);
    for(int i = 0 ; i < occupancyGridMap.info.width ; i++)
        for(int j = 0 ; j < occupancyGridMap.info.height ; j++) {
            if (arr_obs[j][i] > (int)(m_OCCUPANCY_RATIO*(m_AVM_IMG_WIDTH/m_REAL_OCCUPANCY_SIZE_X)))   {
                occupancyGridMap.data[cnt] = 100;
                
                if (((!m_flag_goal_empty && withinpoint(j, i) == 1)) || withinpoint_car(j, i) == 1)
                // if (((!m_flag_goal_empty && withinpoint(j, i) == 1)) )
                    occupancyGridMap.data[cnt] = 0;
            }
            else                                            
                occupancyGridMap.data[cnt] = 0;

            if (arr_free[j][i]) 
                if (!m_flag_goal_empty && withinpoint_free(j, i) == 1) 
                    occupancyGridMap.data[cnt] = 0;
            cnt++;
        }

    if (Pub_occupancyGridMap.getNumSubscribers() > 0)
        Pub_occupancyGridMap.publish(occupancyGridMap);
}

int main(int argc, char **argv) {   
    ros::init(argc, argv, "undistorted_top_view_node");
    ros::NodeHandle node;
    ros::NodeHandle priv_nh("~");

    std::string OCAM_CALIB_FILE_PATH;
    node.getParam("ALL_IMAGE_FLAG",       m_ALL_IMAGE_FLAG);
    OCAM_CALIB_FILE_PATH = "/home/joonwooahn/catkin_ws/src/undistorted_top_view/include";
    node.getParam("OCAM_CALIB_FILE_PATH",       OCAM_CALIB_FILE_PATH);
    node.getParam("AVM_IMG_WIDTH",              m_AVM_IMG_WIDTH);
    node.getParam("AVM_IMG_HEIGHT",             m_AVM_IMG_HEIGHT);
    node.getParam("REAL_OCCUPANCY_SIZE_X",      m_REAL_OCCUPANCY_SIZE_X);
    node.getParam("REAL_OCCUPANCY_SIZE_Y",      m_REAL_OCCUPANCY_SIZE_Y);
    node.getParam("OCCUPANCY_RATIO",            m_OCCUPANCY_RATIO);
 
    node.getParam("PADDING_VALUE",              m_PADDING_VALUE);
    node.getParam("PADDING_UP",                 m_PADDING_UP);
    m_PADDING_UP = false;
    node.getParam("goal_bias",                  m_goal_bias);
    node.getParam("FULL_IMG_RESOL_WIDTH",       m_FULL_IMG_RESOL_WIDTH);
    node.getParam("FULL_IMG_RESOL_HEIGHT",      m_FULL_IMG_RESOL_HEIGHT);
    node.getParam("CROP_ROI_WIDTH",             m_CROP_ROI_WIDTH);
    node.getParam("CROP_ROI_HEIGHT",            m_CROP_ROI_HEIGHT);
    node.getParam("PARKING_SPOT_WIDTH",         m_PARKING_SPOT_WIDTH);
    m_PARKING_SPOT_WIDTH *= 1.1;
    node.getParam("PARKING_SPOT_LENGTH",        m_PARKING_SPOT_LENGTH);
    node.getParam("FREE_ERASE_GAIN",            m_FREE_ERASE_GAIN);
    node.getParam("GRID_RESOLUTION",            m_gridResol);
    node.getParam("front_rear_dist",            m_front_rear_dist);
    node.getParam("parking_index",              m_parking_index);
     
    node.getParam("PARKING_POSE_TOPIC_NAME", m_PARKING_POSE_TOPIC_NAME);

    Sub_parkingPath = node.subscribe("/parkingPath", 1, CallbackParkingPath);  // From doRRT node
    Sub_replanning = node.subscribe("/replanning", 1, CallbackReplanning);  // From doRRT node

    Sub_phantom_left        = node.subscribe("/csi_cam/side_left/image_raw", 1, CallbackPhantom_left);
    Sub_phantom_right       = node.subscribe("/csi_cam/side_right/image_raw", 1, CallbackPhantom_right);
    Sub_phantom_left_seg    = node.subscribe("/parking/phantomnet/side_left", 1 , CallbackPhantom_seg_left);
    Sub_phantom_right_seg   = node.subscribe("/parking/phantomnet/side_right", 1 , CallbackPhantom_seg_right);

    Sub_phantom_front       = node.subscribe("/csi_cam/front_center_svm/image_raw", 1 , CallbackPhantom_front);
    Sub_phantom_rear        = node.subscribe("/csi_cam/rear_center_svm/image_raw", 1, CallbackPhantom_rear);
    Sub_phantom_front_seg   = node.subscribe("/parking/phantomnet/front_center_svm", 1 , CallbackPhantom_seg_front);
    Sub_phantom_rear_seg    = node.subscribe("/parking/phantomnet/rear_center_svm", 1 , CallbackPhantom_seg_rear);

    Sub_localizationData = node.subscribe("/LocalizationData", 1, CallbackLocalizationData);
    // Sub_parkingGoal = node.subscribe("/parking_cands", 1, CallbackParkingGoal);
    Sub_parkingGoal = node.subscribe(m_PARKING_POSE_TOPIC_NAME, 1, CallbackParkingGoal); // get from avm

    Pub_AVM_img         = node.advertise<sensor_msgs::Image>("/AVM_image", 1);
    Pub_AVM_seg_img     = node.advertise<sensor_msgs::Image>("/AVM_seg_image", 1);
    Pub_ICP_img         = node.advertise<sensor_msgs::Image>("/AVM_ICP_image", 1);

    Pub_AVM_seg_img_gray = node.advertise<sensor_msgs::Image>("/AVM_seg_image_gray", 1);
    Pub_AVM_DR           = node.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/AVM_image_DR", 1);
       
    // Load Intrinsic Parameter from Directory**
    // get_ocam_model(&m_front_model, (OCAM_CALIB_FILE_PATH + "/calib_results_phantom_190_028_front.txt").c_str());
    // get_ocam_model(&m_left_model,  (OCAM_CALIB_FILE_PATH + "/calib_results_phantom_190_022_left.txt").c_str());
    // get_ocam_model(&m_right_model, (OCAM_CALIB_FILE_PATH + "/calib_results_phantom_190_023_right.txt").c_str());
    // get_ocam_model(&m_rear_model,  (OCAM_CALIB_FILE_PATH + "/calib_results_phantom_190_029_rear.txt").c_str());


  // Load Intrinsic & Extrinsic Parameter from Directory**
  if(flag == 0) 
  {
    PrepareExtrinsicMat(M_front_param,M_back_param,M_left_param,M_right_param,Front_Inv_Rot_Matrix,Rear_Inv_Rot_Matrix,Left_Inv_Rot_Matrix,Right_Inv_Rot_Matrix);
    //std::cout << "Front Mat" << std::endl;
    //printMat(Front_Inv_Rot_Matrix);
    //std::cout << "Rear Mat" << std::endl;
    //printMat(Rear_Inv_Rot_Matrix);
    //std::cout << "Left Mat" << std::endl;
    //printMat(Left_Inv_Rot_Matrix);
    //std::cout << "Right Mat" << std::endl;
    //printMat(Right_Inv_Rot_Matrix);
    if(
    !get_ocam_model(&m_front_model, (OCAM_CALIB_FILE_PATH + "/calib_results_phantom_190_073_front.txt").c_str()) ||
    !get_ocam_model(&m_left_model, (OCAM_CALIB_FILE_PATH + "/calib_results_phantom_190_071_left.txt").c_str()) ||
    !get_ocam_model(&m_right_model, (OCAM_CALIB_FILE_PATH + "/calib_results_phantom_190_072_right.txt").c_str()) ||
    !get_ocam_model(&m_rear_model, (OCAM_CALIB_FILE_PATH + "/calib_results_phantom_190_074_rear.txt").c_str()))
      return 2;
    flag =1;
  }
    // get_ocam_model(&m_front_model, (OCAM_CALIB_FILE_PATH + "/calib_results_phantom_190_073_front.txt").c_str());
    // get_ocam_model(&m_left_model,  (OCAM_CALIB_FILE_PATH + "/calib_results_phantom_190_071_left.txt").c_str());
    // get_ocam_model(&m_right_model, (OCAM_CALIB_FILE_PATH + "/calib_results_phantom_190_072_right.txt").c_str());
    // get_ocam_model(&m_rear_model,  (OCAM_CALIB_FILE_PATH + "/calib_results_phantom_190_074_rear.txt").c_str());

    AVM_front = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3);
    AVM_rear  = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3); 
    AVM_left  = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3); 
    AVM_right = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3);

    AVM_seg_front = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3);
    AVM_seg_rear  = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3);
    AVM_seg_left  = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3);
    AVM_seg_right = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3);

    AVM_seg_front_gray = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC1);
    AVM_seg_rear_gray  = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC1);
    AVM_seg_left_gray  = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC1);
    AVM_seg_right_gray = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC1);

    cv::Mat aggregated_img, aggregated_seg_img, aggregated_seg_gray, seged_img, cropped_seged_img, img_gray, seged_free_img, seged_rgb_img;

    double PIXEL_PER_METER = (double)(m_AVM_IMG_WIDTH)/m_REAL_OCCUPANCY_SIZE_X;
    m_METER_PER_PIXEL = (double)(1.0/(double)(PIXEL_PER_METER));
    m_dimension = (int)(m_REAL_OCCUPANCY_SIZE_X*2.0);
    m_gridDim = (int)(m_dimension*(int)(1/m_gridResol));

    occupancyGridMap.header.frame_id = "map";
    occupancyGridMap.info.resolution = m_gridResol;
    occupancyGridMap.info.width = occupancyGridMap.info.height = m_gridDim;
    occupancyGridMap.info.origin.position.x = occupancyGridMap.info.origin.position.y = -m_dimension/2 - m_gridResol*2;
    occupancyGridMap.info.origin.position.z = 0.0;
    occupancyGridMap.data.resize(occupancyGridMap.info.width*occupancyGridMap.info.width);
    Pub_occupancyGridMap = node.advertise<nav_msgs::OccupancyGrid>("/occ_map", 1);

    m_car.x = m_car.y = m_car.th = 0.0;
    m_avm_data->clear();
    m_avm_data->header.frame_id = "map";
    
    cout << "OCam Undistort Node Start !!!" << endl;

    // // ros::spin();
    ros::Rate loop_rate(20);

    while(ros::ok()) { 
        // if (!m_flagParking) {
        //     ros::AsyncSpinner spinner(4);
        //     spinner.start();

        // if (m_flagParking) {    //before parking
        //     aggregated_img      = AVM_left + AVM_right;
        //     aggregated_seg_img  = AVM_seg_left + AVM_seg_right;
        //     aggregated_seg_gray = AVM_seg_left_gray + AVM_seg_right_gray;
        // } 
        // else {
            if (m_ALL_IMAGE_FLAG) {
                aggregated_img      = AVM_front + AVM_rear + AVM_left + AVM_right;
                aggregated_seg_img  = AVM_seg_front + AVM_seg_rear + AVM_seg_left + AVM_seg_right;
                aggregated_seg_gray = AVM_seg_front_gray + AVM_seg_rear_gray + AVM_seg_left_gray + AVM_seg_right_gray;
            }
            else {
                aggregated_img      = AVM_left + AVM_right;
                aggregated_seg_img  = AVM_seg_left + AVM_seg_right;
                aggregated_seg_gray = AVM_seg_left_gray + AVM_seg_right_gray;
                // aggregated_img = AVM_front + AVM_rear;
                // aggregated_seg_img = AVM_seg_front + AVM_seg_rear;
                // aggregated_seg_gray = AVM_seg_front_gray + AVM_seg_rear_gray;
            }
        // }
        seged_img = aggregated_img.clone();
        seged_free_img = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3);
        seged_rgb_img = cv::Mat::zeros(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT, CV_8UC3);

        // raw data parking line
        if (m_rgb_color) {
            cv::Mat img_hsv;
            cv::cvtColor(seged_img, img_hsv, COLOR_BGR2HSV);
            cv::Mat white_mask, white_image;
            

            cv::Scalar lower_white = Scalar(0, 0, 180); // sunny: 160, rainy: 30
            cv::Scalar upper_white = Scalar(180, 255, 255);

            cv::inRange(img_hsv, lower_white, upper_white, white_mask);


            for(int i = 0; i<aggregated_img.size().height;i++){
                for(int j=0; j<aggregated_img.size().width; j++){
                    // if((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==255)){
                    //     for(int k=-1; k<2; k++){
                    //         for(int l=-1; l<2; l++){
                    //             if (white_mask.at<uchar>(j+k, i+l) > 0){
                    //                 seged_rgb_img.at<cv::Vec3b>(j+k, i+l)[2] = 255;
                    //                 seged_rgb_img.at<cv::Vec3b>(j+k, i+l)[1] = 255;
                    //                 seged_rgb_img.at<cv::Vec3b>(j+k, i+l)[0] = 255;  
                    //             }
                    //         }
                    //     }
                    // }
                    if (((i-aggregated_img.size().height/2) * (i-aggregated_img.size().height/2) + (j-aggregated_img.size().width/2)*(j-aggregated_img.size().width/2)) <= (aggregated_img.size().height/2) * (aggregated_img.size().height/2)) {
                        if (white_mask.at<uchar>(j, i) > 0) {
                            seged_rgb_img.at<cv::Vec3b>(j, i)[2] = 255;
                            seged_rgb_img.at<cv::Vec3b>(j, i)[1] = 255;
                            seged_rgb_img.at<cv::Vec3b>(j, i)[0] = 255; 
                        }
                        if(((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==128) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==128) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==128))){
                            seged_rgb_img.at<cv::Vec3b>(j, i)[2] = 0;
                            seged_rgb_img.at<cv::Vec3b>(j, i)[1] = 0;
                            seged_rgb_img.at<cv::Vec3b>(j, i)[0] = 0; 
                        }
                        if(((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==0) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==255))){
                            seged_rgb_img.at<cv::Vec3b>(j, i)[2] = 0;
                            seged_rgb_img.at<cv::Vec3b>(j, i)[1] = 0;
                            seged_rgb_img.at<cv::Vec3b>(j, i)[0] = 0; 
                        }
                        // if(((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==0) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==0))){
                        //     seged_rgb_img.at<cv::Vec3b>(j, i)[2] = 0;
                        //     seged_rgb_img.at<cv::Vec3b>(j, i)[1] = 0;
                        //     seged_rgb_img.at<cv::Vec3b>(j, i)[0] = 0; 
                        // }
                        // if(((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==0) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==128))){
                        //     seged_rgb_img.at<cv::Vec3b>(j, i)[2] = 0;
                        //     seged_rgb_img.at<cv::Vec3b>(j, i)[1] = 0;
                        //     seged_rgb_img.at<cv::Vec3b>(j, i)[0] = 0; 
                        // }
                        // if(!((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==255)) && !((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==0) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==0))){
                        //     seged_rgb_img.at<cv::Vec3b>(j, i)[2] = 0;
                        //     seged_rgb_img.at<cv::Vec3b>(j, i)[1] = 0;
                        //     seged_rgb_img.at<cv::Vec3b>(j, i)[0] = 0; 
                        // }

                        // if(!((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==255))){
                        //     seged_rgb_img.at<cv::Vec3b>(j, i)[2] = 0;
                        //     seged_rgb_img.at<cv::Vec3b>(j, i)[1] = 0;
                        //     seged_rgb_img.at<cv::Vec3b>(j, i)[0] = 0; 
                        // }
                        // if(((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==255))){
                        //     seged_rgb_img.at<cv::Vec3b>(j, i)[2] = 255;
                        //     seged_rgb_img.at<cv::Vec3b>(j, i)[1] = 255;
                        //     seged_rgb_img.at<cv::Vec3b>(j, i)[0] = 255; 
                        // }
                    }
                    else {
                        seged_rgb_img.at<cv::Vec3b>(j, i)[2] = 0;
                        seged_rgb_img.at<cv::Vec3b>(j, i)[1] = 0;
                        seged_rgb_img.at<cv::Vec3b>(j, i)[0] = 0; 
                    }
                    
                }
            }
            seged_img = seged_rgb_img.clone();
            // imshow("seged_rgb_img:", seged_rgb_img);
            // waitKey(1);
        }
        else {
            for(int i = 0; i<aggregated_img.size().height;i++){
                for(int j=0; j<aggregated_img.size().width; j++){
                    // noise
                    if(((64 - m_AVM_IMG_WIDTH/2 ) * (8.5/m_REAL_OCCUPANCY_SIZE_X) + m_AVM_IMG_WIDTH/2 <i)&&(i<(85 - m_AVM_IMG_WIDTH/2 ) * (8.5/m_REAL_OCCUPANCY_SIZE_X) + m_AVM_IMG_WIDTH/2)&&((128 - m_AVM_IMG_WIDTH/2 ) * (8.5/m_REAL_OCCUPANCY_SIZE_X) + m_AVM_IMG_WIDTH/2 <j)&&(j<(137 - m_AVM_IMG_WIDTH/2 ) * (8.5/m_REAL_OCCUPANCY_SIZE_X) + m_AVM_IMG_WIDTH/2)){
                        seged_img.at<cv::Vec3b>(j, i)[2] = 0;
                        seged_img.at<cv::Vec3b>(j, i)[1] = 0;
                        seged_img.at<cv::Vec3b>(j, i)[0] = 0;  
                    }

                    if(((114- m_AVM_IMG_WIDTH/2 ) * (8.5/m_REAL_OCCUPANCY_SIZE_X) + m_AVM_IMG_WIDTH/2<i)&&(i<(137- m_AVM_IMG_WIDTH/2 ) * (8.5/m_REAL_OCCUPANCY_SIZE_X) + m_AVM_IMG_WIDTH/2)&&((128- m_AVM_IMG_WIDTH/2 ) * (8.5/m_REAL_OCCUPANCY_SIZE_X) + m_AVM_IMG_WIDTH/2<j)&&(j<(139- m_AVM_IMG_WIDTH/2 ) * (8.5/m_REAL_OCCUPANCY_SIZE_X) + m_AVM_IMG_WIDTH/2)){
                        seged_img.at<cv::Vec3b>(j, i)[2] = 0;
                        seged_img.at<cv::Vec3b>(j, i)[1] = 0;
                        seged_img.at<cv::Vec3b>(j, i)[0] = 0;  
                    }
                    if(((76- m_AVM_IMG_WIDTH/2 ) * (8.5/m_REAL_OCCUPANCY_SIZE_X) + m_AVM_IMG_WIDTH/2<i)&&(i<(101- m_AVM_IMG_WIDTH/2 ) * (8.5/m_REAL_OCCUPANCY_SIZE_X) + m_AVM_IMG_WIDTH/2)&&((68- m_AVM_IMG_WIDTH/2 ) * (8.5/m_REAL_OCCUPANCY_SIZE_X) + m_AVM_IMG_WIDTH/2<j)&&(j<(126- m_AVM_IMG_WIDTH/2 ) * (8.5/m_REAL_OCCUPANCY_SIZE_X) + m_AVM_IMG_WIDTH/2)){
                        seged_img.at<cv::Vec3b>(j, i)[2] = 0;
                        seged_img.at<cv::Vec3b>(j, i)[1] = 0;
                        seged_img.at<cv::Vec3b>(j, i)[0] = 0;  
                    }
                    // vehicles
                    if((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==128) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==128) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==128)){
                        seged_img.at<cv::Vec3b>(j, i)[2] = 0;
                            seged_img.at<cv::Vec3b>(j, i)[1] = 0;
                            seged_img.at<cv::Vec3b>(j, i)[0] = 0;  
                        }
                    if((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==0) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==128)){
                        seged_img.at<cv::Vec3b>(j, i)[2] = 0;
                        seged_img.at<cv::Vec3b>(j, i)[1] = 0;
                        seged_img.at<cv::Vec3b>(j, i)[0] = 0;  
                    }
                    if((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==0) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==0)){
                        seged_img.at<cv::Vec3b>(j, i)[2] = 0;
                        seged_img.at<cv::Vec3b>(j, i)[1] = 0;
                        seged_img.at<cv::Vec3b>(j, i)[0] = 0;  
                    }
                    if((aggregated_img.at<cv::Vec3b>(j, i)[2] <  150) || (aggregated_img.at<cv::Vec3b>(j, i)[1] > 125)  || (aggregated_img.at<cv::Vec3b>(j, i)[0] < 175)  ) //&& (SrcImg.at<cv::Vec3b>(jj, ii)[0] == 255) && (SrcImg.at<cv::Vec3b>(jj, ii)[2] < 140)  && 
                    {
                        seged_img.at<cv::Vec3b>(j, i)[2] = 0;
                        seged_img.at<cv::Vec3b>(j, i)[1] = 0;
                        seged_img.at<cv::Vec3b>(j, i)[0] = 0;  
                    }
                    if((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==128) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==0)){
                        seged_img.at<cv::Vec3b>(j, i)[2] = 0;
                        seged_img.at<cv::Vec3b>(j, i)[1] = 0;
                        seged_img.at<cv::Vec3b>(j, i)[0] = 0;  
                    }
                    }
                }

            // // raw data free space
            // for(int i = 0; i<aggregated_img.size().height;i++){
            //     for(int j=0; j<aggregated_img.size().width; j++){
            //         if(!((aggregated_img.at<cv::Vec3b>(j, i)[2] <  150) || (aggregated_img.at<cv::Vec3b>(j, i)[1] > 125)  || (aggregated_img.at<cv::Vec3b>(j, i)[0] < 175))  ) //&& (SrcImg.at<cv::Vec3b>(jj, ii)[0] == 255) && (SrcImg.at<cv::Vec3b>(jj, ii)[2] < 140)  && 
            //         {
            //             seged_free_img.at<cv::Vec3b>(j, i)[2] = 255;
            //             seged_free_img.at<cv::Vec3b>(j, i)[1] = 255;
            //             seged_free_img.at<cv::Vec3b>(j, i)[0] = 255;  
            //         }
            //         if((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==0) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==128)){
            //             seged_free_img.at<cv::Vec3b>(j, i)[2] = 255;
            //             seged_free_img.at<cv::Vec3b>(j, i)[1] = 255;
            //             seged_free_img.at<cv::Vec3b>(j, i)[0] = 255;  
            //         }
            //         if((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==128) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==0)){
            //             seged_free_img.at<cv::Vec3b>(j, i)[2] = 255;
            //             seged_free_img.at<cv::Vec3b>(j, i)[1] = 255;
            //             seged_free_img.at<cv::Vec3b>(j, i)[0] = 255;  
            //         }
            //         if((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==0) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==0)){
            //             seged_free_img.at<cv::Vec3b>(j, i)[2] = 255;
            //             seged_free_img.at<cv::Vec3b>(j, i)[1] = 255;
            //             seged_free_img.at<cv::Vec3b>(j, i)[0] = 255;  
            //         }
            //     }
            // }

            // cv::cvtColor(seged_img, img_gray, COLOR_BGR2GRAY);

            // seg data
            // for(int i = 0; i<aggregated_img.size().height;i++){
            //     for(int j=0; j<aggregated_img.size().width; j++){
            //         if((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==255)){
            //             seged_img.at<cv::Vec3b>(j, i)[2] = 255;
            //             seged_img.at<cv::Vec3b>(j, i)[1] = 255;
            //             seged_img.at<cv::Vec3b>(j, i)[0] = 255;  
            //         }
            //         else {
            //             seged_img.at<cv::Vec3b>(j, i)[2] = 0;
            //             seged_img.at<cv::Vec3b>(j, i)[1] = 0;
            //             seged_img.at<cv::Vec3b>(j, i)[0] = 0;  
            //         }
            //         // if((aggregated_side_seg_img.at<cv::Vec3b>(j, i)[2]==0) && (aggregated_side_seg_img.at<cv::Vec3b>(j, i)[1]==255) && (aggregated_side_seg_img.at<cv::Vec3b>(j, i)[0]==0)){
            //         //     seged_side_img.at<cv::Vec3b>(j, i)[2] = 0;
            //         //     seged_side_img.at<cv::Vec3b>(j, i)[1] = 0;
            //         //     seged_side_img.at<cv::Vec3b>(j, i)[0] = 0;  
            //         // }
            //         // if((aggregated_img.at<cv::Vec3b>(j, i)[2] <  150) || (aggregated_img.at<cv::Vec3b>(j, i)[1] > 125)  || (aggregated_img.at<cv::Vec3b>(j, i)[0] < 175)  ) //&& (SrcImg.at<cv::Vec3b>(jj, ii)[0] == 255) && (SrcImg.at<cv::Vec3b>(jj, ii)[2] < 140)  && 
            //         // {
            //         //     seged_img.at<cv::Vec3b>(j, i)[2] = 0;
            //         //     seged_img.at<cv::Vec3b>(j, i)[1] = 0;
            //         //     seged_img.at<cv::Vec3b>(j, i)[0] = 0;  
            //         // }
            //         // vehicles
            //         if((aggregated_seg_img.at<cv::Vec3b>(j, i)[2]==128) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[1]==128) && (aggregated_seg_img.at<cv::Vec3b>(j, i)[0]==128)){
            //             seged_img.at<cv::Vec3b>(j, i)[2] = 0;
            //             seged_img.at<cv::Vec3b>(j, i)[1] = 0;
            //             seged_img.at<cv::Vec3b>(j, i)[0] = 0;  
            //         }
            //         // noise
            //         // if((132<i)&&(i<187)&&(268<j)&&(j<282)){
            //         //     seged_img.at<cv::Vec3b>(j, i)[2] = 0;
            //         //     seged_img.at<cv::Vec3b>(j, i)[1] = 0;
            //         //     seged_img.at<cv::Vec3b>(j, i)[0] = 0;  
            //         // }

            //         // if((215<i)&&(i<274)&&(266<j)&&(j<286)){
            //         //     seged_img.at<cv::Vec3b>(j, i)[2] = 0;
            //         //     seged_img.at<cv::Vec3b>(j, i)[1] = 0;
            //         //     seged_img.at<cv::Vec3b>(j, i)[0] = 0;  
            //         // }
            //     }
            // }
        }

        // cv::Rect rect(m_AVM_IMG_WIDTH/4,m_AVM_IMG_HEIGHT/4,m_AVM_IMG_WIDTH*0.5,m_AVM_IMG_HEIGHT*0.5);
        // cropped_seged_img=seged_img(rect);
        // cv::resize(cropped_seged_img, cropped_seged_img, cv::Size(m_AVM_IMG_WIDTH, m_AVM_IMG_HEIGHT));

        // imshow("asdf:", seged_img);
        // imwrite(folderpath+to_string(imgcnt)+imgformat,aggregated_img);
        // imgcnt += 1;
        // setMouseCallback("asdf:", onMouse, reinterpret_cast<void*>(&seged_img));
        // waitKey(1);


        Pub_AVM_img.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", aggregated_img).toImageMsg());
        Pub_AVM_seg_img.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", aggregated_seg_img).toImageMsg());
        Pub_ICP_img.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", seged_img).toImageMsg());
        // Pub_ICP_img.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", cropped_seged_img).toImageMsg());
        // Pub_AVM_seg_img_gray.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", aggregated_seg_gray).toImageMsg());

        if (!m_flagParking) {
            occupancyGridmapPub(aggregated_seg_gray);
        }
            AVMpointCloud(aggregated_img);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}