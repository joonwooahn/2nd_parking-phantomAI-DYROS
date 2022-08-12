# parking-phantomAI-DYROS Deliverables (phantom_phase3_deliverables (2nd year project))
Code repository for automated parking project results between phantomAI and dyros lab.

***
## 1. Requirements
Whole system was tested at Ubuntu 16.04 with ROS-kinetic and requirements below.
* Python 3.8
* CARLA 0.9.7
* OpenCV 4.4.0 // YOU NEED TO ADJUST HEADER FILES DEPENDING ON THE VERSION
* C++14
* OMPL 1.5.0 (https://ompl.kavrakilab.org/installation.html) (not ROS version, install with ./install-ompl-ubuntu.sh)
* fast gicp (https://github.com/SMRT-AIST/fast_gicp) (ROS version, NO CUDA version)
* gtsam (4.x >) (https://gtsam.org)
***
## 2. Description
* dead_reckoning: Kinematic-bicycle-model based Dead Reckoning.
* dyros_controller: controller for tracking the parking path.
* informed_rrt_star_target_tree: "RRT* + Hybrid Curvature (HC) + Curvature Continuous TargetTree" implementation for parking path planning (re-plannable version)
* phantom_can: It is connected with the vehicle's CAN protocol, through **VEH** state machine. It receives the vehicle's state, and send a control signal (steering, acceleration) to the vehicle. This code is written with respect to ECIdriver for connecting PC and a vehicle. https://www.ixxat.com/technical-support/support/linux-driver-software
* undistorted_top_view: transform the cameras of the vehicle to a bird's-eye-view image (AVM) by Scaramuzza.
* usb_cam-develop (optional): uses the cam related packages for real-vehicle test.
* yolo_parking_v5: finds a nearest parking spot to the vehicle using YOLOv5 (bounding box detection algorithm).
(using bounding box) weight file: https://drive.google.com/file/d/18qxFFA_oHFXcSXT4WAdSS2n3xlRyO8ul/view?usp=sharing. 
Put this folder './weights_files/only_white/' from the above link into 'yolo_parking' directory.  
Edit a line 34 in 'yolo_parking/src/main_parking_space.py', in accordance with your directory.  
YOLOv5 https://github.com/ultralytics/yolov5/tree/v5.0 
(You can get detail descriptions of training your own custom data in above link)  
YOLO generate label data: https://github.com/AlexeyAB/Yolo_mark  
*parking_spot_detector_corner_point (using corner points): finds and tracks the parking spot using corner points. A detailed description of the algorithm is in **RITA21_Deep Learning based Parking Slot Detection and Tracking_PSDT-Net_first_ver.pdf**, and the code is based on this repository (https://github.com/younghyopark/parking_spot_corner_detector).  
(using corner points) weight file: https://drive.google.com/file/d/1dY08aWD9sz-sRfjG2G0RtuIYAOLf8nlV/view?usp=sharing. put weight files from the above link into 'runs_after_planning'. You may create the 'runs_after_planning' folder into './script/phantom_avm_rgb_weight/'.

## 3. How to Run
* $ roslaunch dyros_controller dyros_controller_phantom.launch
* $ cd ./yolo_parking_v5 && python3 main_parking_space.py
* During parking path tracking, if the parking spot by the corner point is recognized and judged to be correct, the vehicle is stopped and replanning is performed by pressing the 'r' and 'p' keys in the corner point recognition window.
