#! /usr/bin/env python3
# dyros-phantom@dyros:~/catkin_ws/src/yolo_parking/src$ chmod +x main_parking_space.py 
# 
import rospy
from std_msgs.msg import Float32MultiArray, Bool
from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseArray
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


import time
import sys
from pathlib import Path

import cv2
import torch
import  gi

gi.require_version('Gtk','2.0')

FILE = Path(__file__).absolute()
sys.path.append(FILE.parents[0].as_posix())  # add yolov5/ to path
from nav_msgs.msg import Path

from models.experimental import attempt_load
from utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
from utils.plots import colors, plot_one_box, plot_exact_point
from utils.paring_spot import pix2world, detect_parking_poses, poseArrayFunc, markerFuncXY, markerFuncColor
from utils.torch_utils import time_synchronized
# from torchvision.models import resnet18
import torchvision.transforms as T
import numpy as np
import math


# from deep_sort_pytorch.utils.parser import get_config
# from deep_sort_pytorch.deep_sort import DeepSort

# https://github.com/ultralytics/yolov5/issues/6948
    # /usr/local/lib/python3.8/dist-packages/torch/nn/modules$  sudo gedit upsampling.py    
        # def forward(self, input: Tensor) -> Tensor:
        # return F.interpolate(input, self.size, self.scale_factor, self.mode, self.align_corners,
        #                      #recompute_scale_factor=self.recompute_scale_factor
        #                      )

bridge = CvBridge()

def compute_color_for_id(label):
    """
    Simple function that adds fixed color depending on the id
    """
    palette = (2 ** 11 - 1, 2 ** 15 - 1, 2 ** 20 - 1)

    color = [int((p * (label ** 2 - label + 1)) % 255) for p in palette]
    return tuple(color)


class parking_spot_detection_corner_point():
    def __init__(self):
        # deep_sort_weights = ADDED_PATH_FOR_ROS+'deep_sort_pytorch/deep_sort/deep/checkpoint/ckpt.t7'
        # config_deepsort = ADDED_PATH_FOR_ROS+'deep_sort_pytorch/configs/deep_sort.yaml'

        self.img, self.seg_img = None, None
        self.imgsz = 512
        self.conf_thres =   0.1     #rospy.get_param("CONFIDENCE_THRESHOLD")
        self.iou_thres =    0.9     #rospy.get_param("IOU_THRESHOLD")
        self.view_img =     rospy.get_param("VIEW_IMG")
        self.max_det = 1000
        self.classes = None
        self.agnostic_nms = False #True
        self.augment = False
        self.img_w = rospy.get_param("AVM_IMG_WIDTH")
        self.img_h = rospy.get_param("AVM_IMG_HEIGHT")
        self.PIXEL_PER_METER = (float)(self.img_w)/rospy.get_param("REAL_OCCUPANCY_SIZE_X")
        self.PARKING_LINE_LONG_LEN  = rospy.get_param("PARKING_SPOT_LENGTH")   #4.9
        self.PARKING_LINE_SHORT_LEN = rospy.get_param("PARKING_SPOT_WIDTH")  #2.1

        self.ADDED_PATH_FOR_ROS = "/home/joonwooahn/catkin_ws/src/parking_spot_detector_corner_point/scripts/"

        self.m_carX = self.m_carY = self.m_carTH = 0.0
        self.m_flagParking = False
        # self.m_flagParking = True
        # yolo_weight = self.ADDED_PATH_FOR_ROS+'runs_before_planning/best.pt'
        # yolo_weight = self.ADDED_PATH_FOR_ROS+'runs_after_planning/best.pt'
        yolo_weight = self.ADDED_PATH_FOR_ROS+'phantom_avm_rgb_weight/best.pt'
        # yolo_weight = self.ADDED_PATH_FOR_ROS+'phantom_avm_rgb_weight/high_epoch/best.pt'
        
        self.parking_cands = PoseArray()    # parking spots pose array publish
        self.cornerPoints = Marker()        # corner points array publish
        self.cornerPoints.header.frame_id = self.parking_cands.header.frame_id = "map"
        self.cornerPoints.header.stamp = self.parking_cands.header.stamp = rospy.Time.now()

        self.cornerPoints.type = Marker.SPHERE_LIST
        self.cornerPoints.action = Marker.ADD
        self.cornerPoints.scale.x, self.cornerPoints.scale.y, self.cornerPoints.scale.z = (0.39, 0.39, 0.39)
        # self.cornerPoints.color.r, self.cornerPoints.color.g, self.cornerPoints.color.b = (1.0, 1.0, 0.0) 
        # self.cornerPoints.id = 0


        # print('--- PyTorch Version:', torch.__version__)
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        # print('--- Available CUDA ?:', USE_CUDA)
        print('--- GPU Name :', torch.cuda.get_device_name(), ', --- CUDA device Name:', self.device)
        # print('--- CUDA Index :', torch.cuda.current_device())
        # print('--- GPU Number :', torch.cuda.device_count())

        # print('--- loading yolo model from {}'.format(yolo_weight))
        self.model = attempt_load(yolo_weight, map_location=self.device)  # load FP32 model
        self.model.to(self.device)
        stride = int(self.model.stride.max())  # model stride
        self.imgsz = check_img_size(self.imgsz, s=stride)  # check image size
        self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names  # get class names
        self.model = self.model.cuda()
        self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once

        # # initialize deepsort
        # cfg = get_config()
        # cfg.merge_from_file(config_deepsort)
        # attempt_download(deep_sort_weights, repo='mikel-brostrom/Yolov5_DeepSort_Pytorch')
        # self.deepsort = DeepSort(cfg.DEEPSORT.REID_CKPT,
        #                     max_dist=cfg.DEEPSORT.MAX_DIST, min_confidence=cfg.DEEPSORT.MIN_CONFIDENCE,
        #                     nms_max_overlap=cfg.DEEPSORT.NMS_MAX_OVERLAP, max_iou_distance=cfg.DEEPSORT.MAX_IOU_DISTANCE,
        #                     max_age=cfg.DEEPSORT.MAX_AGE, n_init=cfg.DEEPSORT.N_INIT, nn_budget=cfg.DEEPSORT.NN_BUDGET, pad = cfg.DEEPSORT.PAD,
        #                     use_cuda=True)
    
    def Local2Global(self, Lx, Ly, Lth):
        gX = self.m_carX + (Lx * math.cos(self.m_carTH) - Ly * math.sin(self.m_carTH))
        gY = self.m_carY + (Lx * math.sin(self.m_carTH) + Ly * math.cos(self.m_carTH))
        gTh = Lth + self.m_carTH
        return gX, gY, gTh

    def localizationData_callback(self, msg):
        self.m_carX = msg.data[0]
        self.m_carY = msg.data[1]
        self.m_carTH = msg.data[2]
      
    def image_callback(self, msg):
        self.img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        # self.img = cv2.rotate(img, cv2.ROTATE_90_CLOCKWISE) 
        self.img_w, self.img_h = msg.width, msg.height
        self.run()

    def seg_image_callback(self, msg):
        self.seg_img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        # self.img_w, self.img_h = msg.width, msg.height

    # def parking_path_callback(self, msg):
    #     if self.m_flagParking == False:
    #         self.conf_thres = 0.05;  
    #         yolo_weight = self.ADDED_PATH_FOR_ROS+'runs_after_planning/best.pt'
    #         # print('--- loading yolo model from {}'.format(yolo_weight))
    #         self.model = attempt_load(yolo_weight, map_location=self.device)  # load FP32 model
    #         self.model.to(self.device)
    #         stride = int(self.model.stride.max())  # model stride
    #         self.imgsz = check_img_size(self.imgsz, s=stride)  # check image size
    #         self.names = self.model.module.names if hasattr(self.model, 'module') else self.model.names  # get class names
    #         self.model = self.model.cuda()
    #         self.model(torch.zeros(1, 3, self.imgsz, self.imgsz).to(self.device).type_as(next(self.model.parameters())))  # run once
    #         self.m_flagParking = True


    @torch.no_grad()
    def run(self):
        # Run inference
        if self.img is not None and self.seg_img is not None:
            self.cornerPoints.points.clear()
            self.cornerPoints.colors.clear()
            markerCnt = 0
            parking_poses = None
            cloest_center_pose = None
            out_points = []
            in_points = []

            t0 = time.time()

            im0 = self.img
            img = torch.from_numpy(self.img).to(self.device).unsqueeze(0)
            img = torch.transpose(img, 3, 1)
            img = torch.transpose(img, 2, 3)
            img = img.float()
            img /= 255.0  # 0 - 255 to 0.0 - 81.0
            if img.ndimension() == 3:
                img = img.unsqueeze(0)
            img = img.cuda()
            img = T.Resize(self.imgsz)(img)

            t1 = time_synchronized()
            pred = self.model(img, augment=self.augment)[0]
            # Apply NMS
            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)

            for i, det in enumerate(pred):  # detections per image
                if len(det):
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()

# #################### Only Point detections
                    for *xyxy, conf, cls in reversed(det):
                        # label = f'{self.names[int(cls)]} {conf:.1f}'
                        label = f'{conf:.1f}'
                        
                        if self.names[int(cls)] == 'out' or self.names[int(cls)] == 'in':
                            centerX, centerY = pix2world(int((int(xyxy[0])+int(xyxy[2]))/2.0), int((int(xyxy[1])+int(xyxy[3]))/2.0), self.img_w, self.img_h, self.PIXEL_PER_METER)
                            X, Y, _ = self.Local2Global(centerX, centerY, 0.0)
                            self.cornerPoints.points.append(markerFuncXY(X, Y)) #, 
                            self.cornerPoints.colors.append(markerFuncColor(int(cls), f'{conf:.1f}'))
                            markerCnt += 1
                    
                            center_point = plot_one_box(xyxy, im0, label=label, color=colors(int(cls), True), line_thickness=2, conf_thres = self.conf_thres)
                            if self.names[int(cls)] == 'out':
                                out_points.append(center_point)
                            if self.names[int(cls)] == 'in': 
                            #     center_point = plot_one_box(xyxy, im0, label=label, color=colors(int(cls), True), line_thickness=2, conf_thres = self.conf_thres)
                                in_points.append(center_point)
            self.pub_markers.publish(self.cornerPoints)
                
# #################### 

# #################### + Corner Point Tracking
                    # xywhs = xyxy2xywh(det[:, 0:4])
                    # confs = det[:, 4]
                    # clss  = det[:, 5]

                    # # pass detections to deepsort
                    # # deepsort_outputs = []
                    # deepsort_outputs = self.deepsort.update(xywhs.cpu(), confs.cpu(), clss, im0)

                    # # draw boxes for visualization
                    # if len(deepsort_outputs) > 0:
                    #     for j, (output, conf) in enumerate(zip(deepsort_outputs, confs)): 
                    #         bboxes = output[0:4]
                    #         id = output[4]
                    #         cls = output[5]

                    #         c = int(cls)  # integer class
                    #         label = f'{id} {self.names[c]}'
                    #         color = compute_color_for_id(id)
                    #         center_point = plot_one_box(bboxes, im0, label=label, color=color, line_thickness=2)
                    #         if self.names[c] == 'out':
                    #             out_points.append(center_point)
                    #         elif self.names[c] == 'in':
                    #             in_points.append(center_point)
####################
            parking_pub_flag = False
            ### to get parking pose
            if len(in_points) >= 2:
                # parking_pub_flag, X, Y, TH = detect_parking_poses(im0, in_points, dist_range = 0.2, img_w = self.img_w, img_h = self.img_h, PIXEL_PER_METER=self.PIXEL_PER_METER, case = 'in')

                # parking_pub_flag, X, Y, TH = detect_parking_poses(im0, self.seg_img, in_points, dist_range = 0.35, img_w = self.img_w, img_h = self.img_h,
                parking_poses, cloest_center_pose = detect_parking_poses(im0, self.seg_img, in_points, dist_range = 0.5,
                # parking_poses = detect_parking_pose(im0, in_points, dist_range = 0.35, img_w = self.img_w, img_h = self.img_h,
                                            PIXEL_PER_METER = self.PIXEL_PER_METER, case = 'in',
                                            AVM_IMG_WIDTH = self.img_w, AVM_IMG_HEIGHT = self.img_h,
                                            parking_long_line_length = self.PARKING_LINE_LONG_LEN, parking_short_line_length = self.PARKING_LINE_SHORT_LEN)
            # if len(out_points) >= 2:
            #     # parking_pub_flag, X, Y, TH = detect_parking_poses(im0, out_points, dist_range = 0.35, img_w = self.img_w, img_h = self.img_h, PIXEL_PER_METER=self.PIXEL_PER_METER, case = 'out')
            #     # parking_pub_flag, X, Y, TH = detect_parking_poses(im0, self.seg_img, out_points, dist_range = 0.35,
            #     parking_poses, cloest_center_pose = detect_parking_poses(im0, self.seg_img, out_points, dist_range = 0.5,
            #                                 PIXEL_PER_METER = self.PIXEL_PER_METER, case = 'out',
            #                                 AVM_IMG_WIDTH = self.img_w, AVM_IMG_HEIGHT = self.img_h,
            #                                 parking_long_line_length = self.PARKING_LINE_LONG_LEN, parking_short_line_length = self.PARKING_LINE_SHORT_LEN)
            
                
            self.parking_cands.poses.clear()
            if parking_poses is not None:
                for ii, parking_pose in enumerate(parking_poses):
                    if parking_pose is not None and parking_pose[0] is True:
                        X, Y, TH = self.Local2Global(parking_pose[1], parking_pose[2], parking_pose[3])
                        self.parking_cands.poses.append(poseArrayFunc(X, Y, TH))
            if parking_pub_flag:
                X, Y = pix2world(X, Y, self.img_w, self.img_h, self.PIXEL_PER_METER)
                
                

                # # if Y < 0:
                # #     TH = np.pi - TH
                # # else:
                # TH = 2*np.pi - TH                
                
                # TH -= np.pi/2.0

                # X, Y, TH = self.Local2Global(X, Y, TH)
                # self.parking_cands.poses.append(poseArrayFunc(X, Y, TH))
            
                if parking_poses[0][0] is True:
                    msg = Float32MultiArray()
                    msg.data = [cloest_center_pose[0], cloest_center_pose[1], cloest_center_pose[2], cloest_center_pose[3]]
                    self.pub_cloestCenterPose.publish(msg)

            self.parking_cand_pub.publish(self.parking_cands)

            if self.view_img:
                cv2.imshow("Corner Points", cv2.resize(im0, dsize=(self.img_w, self.img_h), interpolation=cv2.INTER_AREA))
                key = cv2.waitKey(1)  # 1 millisecond
            
                if key == ord('r') or key == ord('R') or key == ord('o') or key == ord('O'):
                    msg = Bool()
                    msg.data = True
                    self.pub_rePlanning.publish(msg)
                elif key == ord('+'):
                    msg = Bool()
                    msg.data = True
                    self.pub_switchingPass.publish(msg)
                elif key == ord('p') or key == ord('P'):
                    msg = Float32MultiArray()
                    msg.data = [0.0]
                    self.pub_canVelData.publish(msg)
            
            t1 = time.time()                            

            # print('time per detection : {:.5f}'.format(t1-t0))

    def listener(self):
        # rospy.Subscriber("/AVM_center_image", Image, self.image_callback)
        # rospy.Subscriber("/AVM_side_image", Image, self.image_callback)

        self.localizationData_sub = rospy.Subscriber("/LocalizationData", Float32MultiArray, self.localizationData_callback)
        self.avm_img_sub = rospy.Subscriber("/AVM_image", Image, self.image_callback)
        self.avm_seg_img_sub = rospy.Subscriber("/AVM_seg_image", Image, self.seg_image_callback)
        # self.parking_path_sub = rospy.Subscriber("/parkingPath", Path, self.parking_path_callback)

        self.parking_cand_pub = rospy.Publisher('/parking_cands', PoseArray, queue_size=1)
        self.pub_canVelData = rospy.Publisher('/CanVelData2', Float32MultiArray, queue_size=1)
        self.pub_markers = rospy.Publisher('/corner_points', Marker, queue_size=1)
        self.pub_rePlanning = rospy.Publisher("replanning", Bool, queue_size=1)
        # self.pub_cloestCenterPose = rospy.Publisher("/parking_cloest_center_pose", Float32MultiArray, queue_size=1)

        self.pub_switchingPass = rospy.Publisher("switching_pass", Bool, queue_size=1)
  
        # self.pub_avmImage = rospy.Publisher("/image_YOLO_result", Image, queue_size = 1)

        # rospy.spin()
        while not rospy.is_shutdown():
            # self.run()
            rospy.sleep(0.05)

def main():
    rospy.init_node('yolo_parking_corner_points')
    Parking_spot_detection_corner_point = parking_spot_detection_corner_point()
    Parking_spot_detection_corner_point.listener()

if __name__ == "__main__":
    main()