#include <iostream>
#include <exception>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_broadcaster.h>
 
#include <ros/ros.h>
#include <ros/package.h>

#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include <float.h>
#include <math.h>
#include <highgui.h>

#include <phantom_ai/core/log.h>
#include <phantom_ai/core/yaml.h>
#include <phantom_ros/phantom_ros.h>
#include <phantom_ros/phantom_ros_node.h>
#include <parking/parking_ros_conversion.h>

//#include "parking/parking_ros_conversion.h"
#include <cv_bridge/cv_bridge.h>

#include <cv.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "opencv2/opencv.hpp"
#include <opencv2/imgproc.hpp>
 
#include <algorithm>
#include <string>
using namespace cv;
#define FRONT 0
#define REAR 1
#define LEFT 2
#define RIGHT 3

#define CMV_MAX_BUF 1024
#define MAX_POL_LENGTH 64

#define AVM_IMG_WIDTH  400
#define AVM_IMG_HEIGHT 400

#define INPUT_WIDTH 512
#define INPUT_HEIGHT 288

#define PADDING_CENTER_BOTTOM 128
#define PADDING_REAR_BOTTOM 128

#define PADDING_VALUE 128
bool Padding_UP = false;  //1 up, 0 down

#define FULL_IMG_RESOL_WIDTH  1920
#define FULL_IMG_RESOL_HEIGHT 1208

#define CROP_ROI_WIDTH   1920//(int)((double)512 * RESIZE_BAG_TO_ORG)
#define CROP_ROI_HEIGHT  1080//(int)((double)288 * RESIZE_BAG_TO_ORG)

#define REAL_OCCUPANCY_SIZE_X 16    // AVM_IMG_WIDTH 400PIX == 25Meter
#define REAL_OCCUPANCY_SIZE_Y 16    // AVM_IMG_HEIGHT 400PIX == 25Meter

#define PIXEL_PER_METER  AVM_IMG_WIDTH/REAL_OCCUPANCY_SIZE_X           //400PIX / 16m                     원래작업 : 16pix/m에서 25pix/m
#define METER_PER_PIXEL  (double)(1.0/(double)(PIXEL_PER_METER)) // 0.0625                                        // 0.0625

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

int flag = 0;

std::string calibration_front ="/home/daniel/phantom-os-2/src/phantom_ros/src/parking/calib_results_phantom_190_073_front.txt";
std::string calibration_left = "/home/daniel/phantom-os-2/src/phantom_ros/src/parking/calib_results_phantom_190_071_left.txt";
std::string calibration_right ="/home/daniel/phantom-os-2/src/phantom_ros/src/parking/calib_results_phantom_190_072_right.txt";
std::string calibration_rear = "/home/daniel/phantom-os-2/src/phantom_ros/src/parking/calib_results_phantom_190_074_rear.txt" ;

ros::Subscriber sub_svm_left_img, sub_svm_right_img, sub_svm_front_img, sub_svm_rear_img;
ros::Subscriber Sub_phantom_left_seg, Sub_phantom_right_seg, Sub_phantom_front_seg, Sub_phantom_rear_seg;
ros::Publisher pub_avm_left_img, pub_avm_right_img, pub_avm_front_img, pub_avm_rear_img;

cv::Mat avm_left = cv::Mat::zeros(AVM_IMG_WIDTH,AVM_IMG_HEIGHT, CV_8UC3);// = cv::Mat::zeros(450,450, CV_8UC3);
cv::Mat avm_right = cv::Mat::zeros(AVM_IMG_WIDTH,AVM_IMG_HEIGHT, CV_8UC3);// = cv::Mat::zeros(450,450, CV_8UC3);
cv::Mat avm_front = cv::Mat::zeros(AVM_IMG_WIDTH,AVM_IMG_HEIGHT, CV_8UC3);
cv::Mat avm_rear = cv::Mat::zeros(AVM_IMG_WIDTH,AVM_IMG_HEIGHT, CV_8UC3);

cv::Mat AVM_seg_front = cv::Mat::zeros(AVM_IMG_WIDTH,AVM_IMG_HEIGHT, CV_8UC3);
cv::Mat AVM_seg_rear = cv::Mat::zeros(AVM_IMG_WIDTH,AVM_IMG_HEIGHT, CV_8UC3);
cv::Mat AVM_seg_left = cv::Mat::zeros(AVM_IMG_WIDTH,AVM_IMG_HEIGHT, CV_8UC3);
cv::Mat AVM_seg_right = cv::Mat::zeros(AVM_IMG_WIDTH,AVM_IMG_HEIGHT, CV_8UC3);

cv::Mat input_img_left = cv::Mat::zeros(INPUT_WIDTH,INPUT_HEIGHT, CV_8UC3);// = cv::Mat::zeros(450,450, CV_8UC3);
cv::Mat input_img_right = cv::Mat::zeros(INPUT_WIDTH,INPUT_HEIGHT, CV_8UC3);// = cv::Mat::zeros(450,450, CV_8UC3);
cv::Mat input_img_front = cv::Mat::zeros(INPUT_WIDTH,INPUT_HEIGHT, CV_8UC3);
cv::Mat input_img_rear = cv::Mat::zeros(INPUT_WIDTH,INPUT_HEIGHT, CV_8UC3);

cv::Mat input_seg_front = cv::Mat::zeros(INPUT_WIDTH,INPUT_HEIGHT, CV_8UC3);
cv::Mat input_seg_rear = cv::Mat::zeros(INPUT_WIDTH,INPUT_HEIGHT, CV_8UC3);
cv::Mat input_seg_left = cv::Mat::zeros(INPUT_WIDTH,INPUT_HEIGHT, CV_8UC3);
cv::Mat input_seg_right = cv::Mat::zeros(INPUT_WIDTH,INPUT_HEIGHT, CV_8UC3);

cv::Mat undist_img_front_rear = cv::Mat::zeros(AVM_IMG_WIDTH,AVM_IMG_HEIGHT, CV_8UC3);
cv::Mat undist_img_left_right = cv::Mat::zeros(AVM_IMG_WIDTH,AVM_IMG_HEIGHT, CV_8UC3);
cv::Mat undist_seg_front_rear = cv::Mat::zeros(AVM_IMG_WIDTH,AVM_IMG_HEIGHT, CV_8UC3);
cv::Mat undist_seg_left_right = cv::Mat::zeros(AVM_IMG_WIDTH,AVM_IMG_HEIGHT, CV_8UC3);

#define M_DEG2RAD  3.1415926 / 180.0

//// Extrinsic Parameters
//double M_front_param[6] = {-0.238 * M_DEG2RAD,  20.154 * M_DEG2RAD,  -0.715 * M_DEG2RAD   ,  1.886, 0.037, 0.742 };
//double M_left_param[6] =  {-0.982 * M_DEG2RAD,  51.351 * M_DEG2RAD,  89.074 * M_DEG2RAD  ,   0.000, 1.041, 1.001 };
//double M_right_param[6] = {3.305 * M_DEG2RAD,  51.714 * M_DEG2RAD,  -87.275 * M_DEG2RAD  ,   0.000, -1.041, 0.985 };
//double M_back_param[6] =  {-0.577 * M_DEG2RAD, 32.868 * M_DEG2RAD,  179.834 * M_DEG2RAD ,   -2.992, 0.005, 0.893 };

// Extrinsic Parameters
double M_front_param[6] = {-0.238 * M_DEG2RAD,  20.154 * M_DEG2RAD,  -0.715 * M_DEG2RAD   ,  1.886, 0.037, 0.742 };
double M_left_param[6] =  {-0.982 * M_DEG2RAD,  51.351 * M_DEG2RAD,  89.074 * M_DEG2RAD  ,   0.000, 1.041, 0.951 };
double M_right_param[6] = {0.305 * M_DEG2RAD,  51.714 * M_DEG2RAD,  -87.275 * M_DEG2RAD  ,   0.000, -1.041, 0.935 };
double M_back_param[6] =  {-0.577 * M_DEG2RAD, 31.868 * M_DEG2RAD,  179.834 * M_DEG2RAD ,   -2.992, 0.005, 0.893 };


cv::Mat cv_front_test = cv::Mat::zeros(960, 540 , CV_8UC3);
cv::Mat cv_rear_test = cv::Mat::zeros(960, 540 , CV_8UC3);
cv::Mat cv_left_test = cv::Mat::zeros(960, 540 , CV_8UC3);
cv::Mat cv_right_test = cv::Mat::zeros(960, 540 , CV_8UC3);

struct XY_coord
{
   /* data */
   double x;
   double y;
};

struct ocam_model
{
  double pol[MAX_POL_LENGTH];    // the polynomial coefficients: pol[0] + x"pol[1] + x^2*pol[2] + ... + x^(N-1)*pol[N-1]
  int length_pol;                // length of polynomial
  double invpol[MAX_POL_LENGTH]; // the coefficients of the inverse polynomial
  int length_invpol;             // length of inverse polynomial
  double xc;         // row coordinate of the center
  double yc;         // column coordinate of the center
  double c;          // affine parameter
  double d;          // affine parameter
  double e;          // affine parameter
  int width;         // image width
  int height;        // image height
};

// Calibration Results
ocam_model front_model;
ocam_model  left_model;
ocam_model right_model;
ocam_model  rear_model;

//------------------------------------------------------------------------------
bool get_ocam_model(struct ocam_model *myocam_model, const char *filename)
{
 double *pol        = myocam_model->pol;
 double *invpol     = myocam_model->invpol;
 double *xc         = &(myocam_model->xc);
 double *yc         = &(myocam_model->yc);
 double *c          = &(myocam_model->c);
 double *d          = &(myocam_model->d);
 double *e          = &(myocam_model->e);
 int    *width      = &(myocam_model->width);
 int    *height     = &(myocam_model->height);
 int *length_pol    = &(myocam_model->length_pol);
 int *length_invpol = &(myocam_model->length_invpol);
 FILE *f;
 char buf[CMV_MAX_BUF];
 int i;

 //Open file
 if(!(f=fopen(filename,"r")))
 {
   printf("File %s cannot be opened\n", filename);
   return false;
 }

 //Read polynomial coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", length_pol);
 for (i = 0; i < *length_pol; i++)
 {
     fscanf(f," %lf",&pol[i]);
 }

 //Read inverse polynomial coefficients
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d", length_invpol);
 for (i = 0; i < *length_invpol; i++)
 {
     fscanf(f," %lf",&invpol[i]);
 }

 //Read center coordinates
 fscanf(f,"\n");
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf\n", xc, yc);

 //Read affine coefficients
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%lf %lf %lf\n", c,d,e);

 //Read image size
 fgets(buf,CMV_MAX_BUF,f);
 fscanf(f,"\n");
 fscanf(f,"%d %d", height, width);

 fclose(f);

 return true;
}

void printMat(double (&arr)[3][3])
{
  for (int i=0; i < 3 ; i++)
  {
    for (int j=0; j < 3 ; j++)
    {
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

void world2cam(double point2D[2], double point3D[3], struct ocam_model *myocam_model)
{
 double *invpol     = myocam_model->invpol;
 double xc          = (myocam_model->xc);
 double yc          = (myocam_model->yc);
 double c           = (myocam_model->c);
 double d           = (myocam_model->d);
 double e           = (myocam_model->e);
 int    width       = (myocam_model->width);
 int    height      = (myocam_model->height);
 int length_invpol  = (myocam_model->length_invpol);
 double norm        = sqrt(point3D[0]*point3D[0] + point3D[1]*point3D[1]);
 double theta       = atan(point3D[2]/norm);
 double t, t_i;
 double rho, x, y;
 double invnorm;
 int i;

  if (norm != 0)
  {
    invnorm = 1/norm;
    t  = theta;
    rho = invpol[0];
    t_i = 1;

    for (i = 1; i < length_invpol; i++)
    {
      t_i *= t;
      rho += t_i*invpol[i];
    }

    x = point3D[0]*invnorm*rho;
    y = point3D[1]*invnorm*rho;

    point2D[0] = x*c + y*d + xc;
    point2D[1] = x*e + y   + yc;
  }
  else
  {
    point2D[0] = xc;
    point2D[1] = yc;
  }
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

void seg2rgb(cv::Mat input_img, cv::Mat& output_img, cv::Mat& output_img_gray) 
{
  output_img = cv::Mat::zeros(input_img.rows, input_img.cols, CV_8UC3);
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
#define SEG_VEHICLE               6
#define SEG_CART                  7
#define SEG_WHEEL                 8
#define SEG_WHEEL_FRONT           9
#define SEG_WHEEL_REAR            10
#define SEG_GENERAL               11
#define SEG_BICYCLE               12
#define SEG_MOTORCYCLE            13
#define SEG_BICYCLIST             14
#define SEG_MOTORCYCLIST          15
#define SEG_PEDESTRIAN            16
#define SEG_FREESPACE             17
#define SEG_FREESPACE_HOLE        18
#define SEG_BOUNDARY              19
#define SEG_STROLLER              20
#define SEG_STOPPER               21
#define SEG_ANIMAL                22
#define SEG_CONSTRUCTION          23
#define SEG_INVALID               24
#define SEG_CLASSES_NUM           25
*/

            if ((int)input_img.at<uchar>(i, j) == 4 || (int)input_img.at<uchar>(i, j) == 6 || (int)input_img.at<uchar>(i, j) == 17) 
            {
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

void preProcessing(cv::Mat input_img, cv::Mat& output_img)
{
  cv::Size input_img_size = input_img.size();
  //resize, bagfile_size => (1920, 1080)
  cv::resize( input_img, input_img, cv::Size(CROP_ROI_WIDTH, CROP_ROI_HEIGHT), 0, 0, cv::INTER_LINEAR );
  // padding
  cv::Mat temp;
  cv::copyMakeBorder(input_img, temp, 0, 128, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
  
  /*
  if (input_img_size.width == 960)
  {
    cv::copyMakeBorder(input_img, temp, PADDING_VALUE - 4, 0, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
    cv::copyMakeBorder(temp, temp, 0, 4, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
  }
  else
  {
    cv::copyMakeBorder(input_img, temp, 0, PADDING_VALUE - 4, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
    cv::copyMakeBorder(temp, temp, 4, 0, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
  }*/
  cv::resize( temp, output_img, cv::Size(FULL_IMG_RESOL_WIDTH, FULL_IMG_RESOL_HEIGHT), 0, 0, cv::INTER_LINEAR );
}

void fill_avm_img(cv::Mat input_img, cv::Mat& output_img, int col, int row,
                    double x_,double y_, double *Dir_param, ocam_model *model,
                    bool gray, int Direction_Mode, bool line_plot)
{
  XY_coord xy;
  xy = InvProjGND(x_, y_, Dir_param[0], Dir_param[1], Dir_param[2], Dir_param[3], Dir_param[4] ,Dir_param[5], model, Direction_Mode);

  if( ((xy.x < FULL_IMG_RESOL_WIDTH) && (xy.x >= 0)) && ((xy.y < FULL_IMG_RESOL_HEIGHT) && (xy.y >= 0)) )
  {
    output_img.at<cv::Vec3b>(int(row), int(col))[0] = static_cast<uint8_t>(input_img.at<cv::Vec3b>(xy.y, xy.x)[0]);
    output_img.at<cv::Vec3b>(int(row), int(col))[1] = static_cast<uint8_t>(input_img.at<cv::Vec3b>(xy.y, xy.x)[1]);
    output_img.at<cv::Vec3b>(int(row), int(col))[2] = static_cast<uint8_t>(input_img.at<cv::Vec3b>(xy.y, xy.x)[2]);
  }
}

void Inverse_Warping(cv::Mat input_img, cv::Mat& output_img, int Direction_Mode, bool gray, bool line_plot)
{
  // Initialize
  cv::Mat Processed_img;
  double x_,y_;

  // Preprocess for image resizing
  preProcessing(input_img, Processed_img);

  output_img = cv::Mat::zeros(AVM_IMG_WIDTH, AVM_IMG_HEIGHT, CV_8UC3);

  // Camera intrinsic and extrinsic model
  double *Dir_param;
  ocam_model *model;

  if(Direction_Mode == FRONT)
  {
    Dir_param = M_front_param;
    model = &front_model;
    for(int row = 0; row < (AVM_IMG_HEIGHT / 2); row++){
      for(int col = 0; col < AVM_IMG_WIDTH; col++){
        // WORLD_coord TO IMG coord
        y_ = (REAL_OCCUPANCY_SIZE_X/2.0) - double (col) * METER_PER_PIXEL;
        x_ = (REAL_OCCUPANCY_SIZE_Y/2.0) - double (row) * METER_PER_PIXEL;

        fill_avm_img(Processed_img, output_img, col, row, x_, y_, Dir_param, model, gray, Direction_Mode, line_plot);
      }
    }
  }
  else if(Direction_Mode == REAR)
  {
    Dir_param = M_back_param;
    model = &rear_model;
    for(int row = AVM_IMG_HEIGHT; row > (AVM_IMG_HEIGHT / 2); row--)
    {
      for(int col = AVM_IMG_WIDTH; col > 0; col--)
      {
        // WORLD_coord TO IMG coord
        y_ = -(REAL_OCCUPANCY_SIZE_X/2.0) + double (AVM_IMG_WIDTH - col) * METER_PER_PIXEL;
        x_ = -(REAL_OCCUPANCY_SIZE_Y/2.0) + double (AVM_IMG_WIDTH - row) * METER_PER_PIXEL;

        fill_avm_img(Processed_img, output_img, col, row, x_, y_, Dir_param, model, gray, Direction_Mode, line_plot);
      }
    }
  }
  else if(Direction_Mode == LEFT)
  {
    Dir_param = M_left_param;
    model = &left_model;
    for(int row = AVM_IMG_HEIGHT; row> 0; row--)
    {
      for(int col = 0; col < (AVM_IMG_WIDTH / 2); col++)
      {
        // WORLD_coord TO IMG coord
        y_ = (REAL_OCCUPANCY_SIZE_X/2.0) - double (col) * METER_PER_PIXEL;
        x_ = -(REAL_OCCUPANCY_SIZE_Y/2.0) + double (AVM_IMG_WIDTH - row) * METER_PER_PIXEL;

        fill_avm_img(Processed_img, output_img, col, row, x_, y_, Dir_param, model, gray, Direction_Mode, line_plot);
      }
    }
  }
  else if(Direction_Mode == RIGHT)
  {
    Dir_param = M_right_param;
    model = &right_model;
    for(int row = 0; row< AVM_IMG_HEIGHT; row++)
    {
      for(int col = AVM_IMG_WIDTH; col > (AVM_IMG_WIDTH / 2); col--)
      {
        // WORLD_coord TO IMG coord
        y_ = -(REAL_OCCUPANCY_SIZE_X/2.0) + double (AVM_IMG_WIDTH - col) * METER_PER_PIXEL;
        x_ = (REAL_OCCUPANCY_SIZE_Y/2.0)  - double (row) * METER_PER_PIXEL;

        fill_avm_img(Processed_img, output_img, col, row, x_, y_, Dir_param, model, gray, Direction_Mode, line_plot);
      }
    }
  }
}

// Front
void CallbackPhantom_front(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Size size(msg->width, msg->height);
  cv::Mat cv_frame_resize_pad = cv::Mat(size, CV_8UC2, (void *)&msg->data[0]);
  cv::Mat cv_rgb;
  cv::cvtColor(cv_frame_resize_pad, cv_rgb, COLOR_YUV2BGR_YUYV);
  cv_rgb.copyTo(input_img_front);
  Inverse_Warping(cv_rgb, avm_front, FRONT, false, false);
  cv_front_test = avm_front;
}
// Rear
void CallbackPhantom_rear(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Size size(msg->width, msg->height);
  cv::Mat cv_frame_resize_pad = cv::Mat(size, CV_8UC2, (void *)&msg->data[0]);
  cv::Mat cv_rgb;
  cv::cvtColor(cv_frame_resize_pad, cv_rgb, COLOR_YUV2BGR_YUYV);
  cv_rgb.copyTo(input_img_rear);
  Inverse_Warping(cv_rgb, avm_rear, REAR, false, false);
  cv_rear_test = avm_rear;
}

// Left
void CallbackPhantom_left(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Size size(msg->width, msg->height);
  cv::Mat cv_frame_resize_pad = cv::Mat(size, CV_8UC2, (void *)&msg->data[0]);
  cv::Mat cv_rgb;
  cv::cvtColor(cv_frame_resize_pad, cv_rgb, COLOR_YUV2BGR_YUYV);
  cv_rgb.copyTo(input_img_left);
  Inverse_Warping(cv_rgb, avm_left, LEFT, false, false);
  cv_left_test = avm_left;
}
// Right
void CallbackPhantom_right(const sensor_msgs::ImageConstPtr& msg)
{
  cv::Size size(msg->width, msg->height);
  cv::Mat cv_frame_resize_pad = cv::Mat(size, CV_8UC2, (void *)&msg->data[0]);
  cv::Mat cv_rgb;
  cv::cvtColor(cv_frame_resize_pad, cv_rgb, COLOR_YUV2BGR_YUYV);
  cv_rgb.copyTo(input_img_right);
  Inverse_Warping(cv_rgb, avm_right, RIGHT, false, false);
  cv_right_test = avm_right;
}


//Left Segmentation
void CallbackPhantom_seg_left(const parking::ParkingPhantomnetData::ConstPtr& msg) 
{ 
    cv::Mat cv_frame_seg = cv_bridge::toCvCopy( msg->segmentation, msg->segmentation.encoding )->image;
    cv::Mat cv_frame_raw_new_gray = cv_bridge::toCvCopy( msg->viz, msg->segmentation.encoding )->image;
    cv::Mat cv_rgb;
    cv::Mat cv_gray;
    cv::Mat cv_rgb_pad;
    seg2rgb(cv_frame_seg, cv_rgb, cv_frame_raw_new_gray);
    cv::copyMakeBorder(cv_rgb, cv_rgb_pad, 16, 16, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
    cv_rgb_pad.copyTo(input_seg_left);
    Inverse_Warping(cv_rgb_pad, AVM_seg_left, LEFT, false, false);
}
//Right Segmentation
void CallbackPhantom_seg_right(const parking::ParkingPhantomnetData::ConstPtr& msg) 
{
    cv::Mat cv_frame_seg = cv_bridge::toCvCopy( msg->segmentation, msg->segmentation.encoding )->image;
    cv::Mat cv_frame_raw_new_gray = cv_bridge::toCvCopy( msg->viz, msg->segmentation.encoding )->image;
    cv::Mat cv_rgb;
    cv::Mat cv_gray;
    cv::Mat cv_rgb_pad;
    seg2rgb(cv_frame_seg, cv_rgb, cv_frame_raw_new_gray);
    cv::copyMakeBorder(cv_rgb, cv_rgb_pad, 16, 16, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
    cv_rgb_pad.copyTo(input_seg_right);
    Inverse_Warping(cv_rgb_pad, AVM_seg_right, RIGHT, false, false);
}

void CallbackPhantom_seg_front(const parking::ParkingPhantomnetData::ConstPtr& msg) 
{    
    cv::Mat cv_frame_seg = cv_bridge::toCvCopy( msg->segmentation, msg->segmentation.encoding )->image;
    cv::Mat cv_frame_raw_new_gray = cv_bridge::toCvCopy( msg->viz, msg->segmentation.encoding )->image;
    cv::Mat cv_rgb;
    cv::Mat cv_gray;
    cv::Mat cv_rgb_pad;
    seg2rgb(cv_frame_seg, cv_rgb, cv_frame_raw_new_gray);
    cv::copyMakeBorder(cv_rgb, cv_rgb_pad, 16, 16, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
    cv_rgb_pad.copyTo(input_seg_front);
    Inverse_Warping(cv_rgb_pad, AVM_seg_front, FRONT, false, false);
}

void CallbackPhantom_seg_rear(const parking::ParkingPhantomnetData::ConstPtr& msg) 
{
    cv::Mat cv_frame_seg = cv_bridge::toCvCopy( msg->segmentation, msg->segmentation.encoding )->image;
    cv::Mat cv_frame_raw_new_gray = cv_bridge::toCvCopy( msg->viz, msg->segmentation.encoding )->image;
    cv::Mat cv_rgb;
    cv::Mat cv_gray;
    cv::Mat cv_rgb_pad;
    seg2rgb(cv_frame_seg, cv_rgb, cv_frame_raw_new_gray);
    cv::copyMakeBorder(cv_rgb, cv_rgb_pad, 16, 16, 0, 0, cv::BORDER_CONSTANT, cv::Scalar(0,0,0) );
    cv_rgb_pad.copyTo(input_seg_rear);
    Inverse_Warping(cv_rgb_pad, AVM_seg_rear, REAR, false, false);
}

int main(int argc, char **argv)
{
  phantom_ros::PhantomRosNode::init(argc, argv, "parking_image_log_node");
//  ros::init(argc, argv, "parking_");
  ros::NodeHandle nodeHandle("~");

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
    if(!get_ocam_model(&front_model, calibration_front.c_str()) ||
    !get_ocam_model(&left_model, calibration_left.c_str()) ||
    !get_ocam_model(&right_model, calibration_right.c_str()) ||
    !get_ocam_model(&rear_model, calibration_rear.c_str()))
      return 2;
    flag =1;
  }

  sub_svm_left_img       = nodeHandle.subscribe("/csi_cam/side_left/image_raw", 1, CallbackPhantom_left);
  sub_svm_right_img      = nodeHandle.subscribe("/csi_cam/side_right/image_raw", 1, CallbackPhantom_right);
  Sub_phantom_left_seg    = nodeHandle.subscribe("/parking/phantomnet/side_left", 1 , CallbackPhantom_seg_left);
  Sub_phantom_right_seg   = nodeHandle.subscribe("/parking/phantomnet/side_right", 1 , CallbackPhantom_seg_right);

  sub_svm_front_img    = nodeHandle.subscribe("/csi_cam/front_center_svm/image_raw", 1 , CallbackPhantom_front);
  sub_svm_rear_img     = nodeHandle.subscribe("/csi_cam/rear_center_svm/image_raw", 1, CallbackPhantom_rear);
  Sub_phantom_front_seg    = nodeHandle.subscribe("/parking/phantomnet/front_center_svm", 1 , CallbackPhantom_seg_front);
  Sub_phantom_rear_seg   = nodeHandle.subscribe("/parking/phantomnet/rear_center_svm", 1 , CallbackPhantom_seg_rear);


  pub_avm_left_img          = nodeHandle.advertise<sensor_msgs::Image>("/avm_image_left", 1);
  pub_avm_right_img          = nodeHandle.advertise<sensor_msgs::Image>("/avm_image_right", 1);
  pub_avm_front_img          = nodeHandle.advertise<sensor_msgs::Image>("/avm_image_front", 1);
  pub_avm_rear_img          = nodeHandle.advertise<sensor_msgs::Image>("/avm_image_rear", 1);



  ros::Rate loop_rate(100);
  namedWindow("undistort_img_left_right");
  namedWindow("undistort_img_front_rear");
  namedWindow("undistort_seg_left_right");
  namedWindow("undistort_seg_front_rear");
  //namedWindow("input_img_left");
  //namedWindow("input_img_right");
  //namedWindow("input_img_front");
  //namedWindow("input_img_rear");
  //namedWindow("input_seg_left");
  //namedWindow("input_seg_right");
  //namedWindow("input_seg_front");
  //namedWindow("input_seg_rear");

  while(ros::ok())
  {
    ros::AsyncSpinner spinner(8);
    spinner.start();

    hconcat(avm_left(cv::Rect(0,0,avm_left.cols/2,avm_left.rows)),avm_right(cv::Rect(avm_right.cols/2,0,avm_right.cols/2,avm_right.rows)),undist_img_left_right);
    vconcat(avm_front(cv::Rect(0,0,avm_front.cols,avm_front.rows/2)),avm_rear(cv::Rect(0,avm_rear.rows/2,avm_rear.cols,avm_rear.rows/2)),undist_img_front_rear);
    
    hconcat(AVM_seg_left(cv::Rect(0,0,AVM_seg_left.cols/2,AVM_seg_left.rows)),AVM_seg_right(cv::Rect(AVM_seg_right.cols/2,0,AVM_seg_right.cols/2,AVM_seg_right.rows)),undist_seg_left_right);
    vconcat(AVM_seg_front(cv::Rect(0,0,AVM_seg_front.cols,AVM_seg_front.rows/2)),AVM_seg_rear(cv::Rect(0,AVM_seg_rear.rows/2,AVM_seg_rear.cols,AVM_seg_rear.rows/2)),undist_seg_front_rear);

    imshow("undistort_img_left_right", undist_img_left_right);
    imshow("undistort_img_front_rear", undist_img_front_rear);
    imshow("undistort_seg_left_right", undist_seg_left_right);
    imshow("undistort_seg_front_rear", undist_seg_front_rear);
    //imshow("input_img_left",input_img_left);
    //imshow("input_img_right",input_img_right);
    //imshow("input_img_front",input_img_front);
    //imshow("input_img_rear",input_img_rear);
    //imshow("input_seg_left",input_seg_left);
    //imshow("input_seg_right",input_seg_right);
    //imshow("input_seg_front",input_seg_front);
    //imshow("input_seg_rear",input_seg_rear);

    cv::waitKey(1);

    pub_avm_left_img.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", avm_left).toImageMsg());
    pub_avm_right_img.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", avm_right).toImageMsg());
    pub_avm_front_img.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", avm_front).toImageMsg());
    pub_avm_rear_img.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", avm_rear).toImageMsg());

    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
