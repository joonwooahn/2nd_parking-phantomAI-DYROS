#!/usr/bin/env python3
# dyros-phantom@dyros:~/catkin_ws/src/yolo_parking_v5/src$ chmod +x main_parking_space.py 
# 

import rospy
from std_msgs.msg import Float32MultiArray, Bool
from geometry_msgs.msg import PoseArray
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
from models.common import DetectMultiBackend
from utils.general import check_img_size, non_max_suppression, scale_coords, xyxy2xywh
from utils.plots import colors, plot_one_box, plot_exact_point 
from utils.augmentations import letterbox
from utils.paring_spot import pix2world, markerFuncXY, markerFuncColor, detect_parking_poses, poseArrayFunc
from utils.torch_utils import select_device, time_sync
# from torchvision.models import resnet18

import torchvision.transforms as T
import numpy as np
import math

size_gain = 5
angle_range = 15
DEG2RAD = 3.141592 / 180.0

AVM_IMG_WIDTH =  150
AVM_IMG_HEIGHT = 150
REAL_OCCUPANCY_SIZE_X = 16.5
REAL_OCCUPANCY_SIZE_Y = 16.5
PIXEL_PER_METER = AVM_IMG_WIDTH/REAL_OCCUPANCY_SIZE_X


def convertBack(x, y, w, h):
    xmin = int(round(x - (w / 2)))
    xmax = int(round(x + (w / 2)))
    ymin = int(round(y - (h / 2)))
    ymax = int(round(y + (h / 2)))
    return xmin, ymin, xmax, ymax


def minus2zero(num):
    if num < 0:
        return 0
    else:
        return num


def getPt1Pt2(box):

    x, y, w, h = box[0],\
                box[1],\
                box[2],\
                box[3]

    xmin, ymin, xmax, ymax = convertBack(float(x), float(y), float(w), float(h))
    pt1 = (minus2zero(xmin-size_gain), minus2zero(ymin-size_gain))
    pt2 = (minus2zero(xmax+size_gain), minus2zero(ymax+size_gain))

    return pt1, pt2


def cvDrawLines(rho, theta, img):

    x0 = np.cos(theta)*rho
    y0 = np.sin(theta)*rho
    x1 = int(x0 + 333*(-np.sin(theta)))
    y1 = int(y0 + 333*(np.cos(theta)))
    x2 = int(x0 - 333*(-np.sin(theta)))
    y2 = int(y0 - 333*(np.cos(theta)))
    cv2.line(img, (x1,y1), (x2,y2), (0, 0, 255), 1, 4)

    return img


class parking_spot_detection_corner_point():

    def __init__(self):

        self.img = None
        self.imgsz = 512
        self.conf_thres = 0.1
        self.iou_thres = 0.1
        self.view_img = True # False
        self.classes = None
        self.agnostic_nms = False #True
        self.augment = False
        self.max_det = 1000
        self.img_w = 0
        self.img_h = 0
        self.half = False
        # self.PIXEL_PER_METER = (float)(self.img_w)/rospy.get_param("REAL_OCCUPANCY_SIZE_X")

        self.PIXEL_PER_METER = PIXEL_PER_METER
        self.m_carX = self.m_carY = self.m_carTH = 0.0
        self.m_flagParking = False

        # yolo_weight = rospy.get_param("ADDED_PATH_FOR_ROS")+'runs/best.pt'

        yolo_weight = './weight_files/only_white/best.pt'
        # yolo_weight = '/home/dyros-vehicle/catkin_ws/src/yolo_parking_spot_detect/scripts/weight_files/only_white/best.pt'
        self.parking_cands = PoseArray()    # parking spots pose array publish
        print('--- PyTorch Version:', torch.__version__)
        self.device = torch.device('cuda:0' if torch.cuda.is_available() else 'cpu')
        print('--- GPU Name :', torch.cuda.get_device_name(), ', --- CUDA device Name:', self.device)

        # Load model
        print('--- loading yolo model from {}'.format(yolo_weight))
        self.model = DetectMultiBackend(yolo_weight, device=self.device, dnn=False)
        self.stride, self.names, self.pt, self.jit, self.onnx, self.engine = self.model.stride, self.model.names, self.model.pt, self.model.jit, self.model.onnx, self.model.engine
        self.imgsz = check_img_size(self.imgsz, s=self.stride)  # check image size

        # Half
        self.half &= (self.pt or self.engine) and self.device.type != 'cpu'  # half precision only supported by PyTorch on CUDA
        if self.pt:
            self.model.model.half() if self.half else self.model.model.float()

        # Run inference
        # self.model.warmup(imgsz=(1, 3, *self.imgsz), half=self.half)  # warmup

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
        img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, -1)
        self.img = cv2.rotate(img, cv2.ROTATE_90_COUNTERCLOCKWISE) 
        # self.img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        # cv2.imshow('RGB**', self.img)
        # cv2.waitKey(1)
        self.img_w, self.img_h = msg.width, msg.height
        # print("SS : ", self.img_w, self.img_h)
        # self.run()


    # def parking_path_callback(self, msg):

    #     if self.m_flagParking == False:
    #         self.conf_thres = 0.3; 
    #         yolo_weight = rospy.get_param("ADDED_PATH_FOR_ROS")+'runs_yh_park/best.pt'
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
        dt, seen = [0.0, 0.0, 0.0], 0
        # Run inference
        parking_list = []
        parking_dist = []
        if self.img is not None:
            im0 = self.img
            img = letterbox(im0, self.imgsz, stride=self.stride, auto=self.pt and not self.jit)[0]

            # Convert
            img = img.transpose((2, 0, 1))[::-1]  # HWC to CHW, BGR to RGB
            img = np.ascontiguousarray(img)
            t1 = time_sync()
            img = torch.from_numpy(img).to(self.device)
            img = img.half() if self.half else img.float()  # uint8 to fp16/32
            img /= 255  # 0 - 255 to 0.0 - 1.0

            if len(img.shape) == 3:
                img = img[None]  # expand for batch dim

            t2 = time_sync()
            dt[0] += t2 - t1


            # prdict parking spaces

            pred = self.model(img, augment=self.augment, visualize=False)
            t3 = time_sync()
            dt[1] += t3 - t2


            # NMS

            pred = non_max_suppression(pred, self.conf_thres, self.iou_thres, self.classes, self.agnostic_nms, max_det=self.max_det)
            dt[2] += time_sync() - t3
            for i, det in enumerate(pred):  # detections per image
                if len(det):
                    det[:, :4] = scale_coords(img.shape[2:], det[:, :4], im0.shape).round()
                    for detected_box in det:
                        detected_box = detected_box.cpu().numpy()
                        pt1, pt2 = (int(detected_box[0]), int(detected_box[1])), (int(detected_box[2]), int(detected_box[3]))
                        center = (int((pt1[0]+pt2[0])/2.0), int((pt1[1]+pt2[1])/2.0)) # image's x and y. The center of the image is upper-left, and the right-direction will be +x, and the bottom-direction will be +y

                        X = (self.img_w * 0.5 - center[0])/self.PIXEL_PER_METER # 20.0 with respect to the vehicle-center frame, and division value is a widt
                        Y = (center[1] - self.img_h * 0.5)/self.PIXEL_PER_METER # 16.0

                        pt1, pt2 = getPt1Pt2((center[0], center[1], abs(pt2[0] - pt1[0]), abs(pt2[1] - pt1[1])))

                        ROI_image = im0.swapaxes(0, 1)[pt1[0] : pt2[0], pt1[1] : pt2[1], : ].swapaxes(0, 1)

                        if ROI_image.shape[0] > 0 and ROI_image.shape[1] > 0:
                            edges = cv2.Canny(ROI_image, 50, 111) # min max
                            lines = cv2.HoughLines(edges, rho=1, theta=np.pi/360, threshold=(int)(ROI_image.shape[0]*0.73))   # aorus // TODO: 공인 시험
                            # lines = cv2.HoughLines(edges, rho=1, theta=np.pi/360, threshold=(int)(ROI_image.shape[0]*0.97))   # Phantom

                            theta_last = 0.0
                            if lines is not None:
                                for i in range(len(lines)):
                                    for rho, theta in lines[i]:
                                        if (theta > (270 - angle_range ) * DEG2RAD and theta < (270 + angle_range) * DEG2RAD):
                                            theta_last = theta - np.pi

                                        if (theta > (90 - angle_range) * DEG2RAD) and theta < (90 + angle_range) * DEG2RAD:
                                            theta_last = theta

                                        cvDrawLines(rho, theta_last, ROI_image)

                                if Y < 0:
                                    theta_last = np.pi - theta_last
                                else:
                                    theta_last = 2*np.pi - theta_last

                                

                                if theta_last != 0.0:

                                    cv2.arrowedLine(im0, ((int)(center[0] + 13*np.sin(theta_last)), (int)(center[1] + 13*np.cos(theta_last))), \

                                                ((int)(center[0] - 13*np.sin(theta_last)), (int)(center[1] - 13*np.cos(theta_last))), (255,255,0), thickness=5, line_type=8, shift = 0, tipLength=0.39)

                            # self.parking_cands.poses.append(poseArrayFunc(X, Y, theta_last - np.pi / 2))
                            parking_list.append((X,Y, theta_last))
                            parking_dist.append((X-1.0)*(X-1.0) + Y*Y)
                        cv2.rectangle(im0, pt1, pt2, (0, 255, 0), 3)
                        cv2.circle(im0, center, 2, (0, 255, 0), thickness = 2, lineType = -1)
            # print(parking_list)
            # print(parking_dist)
            # print(min_idx)
            

            if (len(parking_list) > 0):
                min_idx = np.argmin(parking_dist)
                self.parking_cands.poses.append(poseArrayFunc(parking_list[min_idx][0], parking_list[min_idx][1], parking_list[min_idx][2] - np.pi / 2))
                self.parking_cand_pub.publish(self.parking_cands)
                self.parking_cands.poses.clear()


            # img_tmp0 = cv2.resize(self.img, dsize=(int(480), int(720)))
            # src_img = cv2.resize(img_tmp0, dsize=(int(480/2), int(720/2)))
            # src_img = cv2.resize(self.img, dsize=(int(720/2), int(480/2)))

            # t1 = time.time()                  

            # img_msg = Image()

            # img_msg.height = src_img.shape[0]

            # img_msg.width = src_img.shape[1]

            # img_msg.step = src_img.strides[0]

            # img_msg.encoding = 'bgr8'

            # img_msg.header.frame_id = 'map'

            # img_msg.header.stamp = rospy.Time.now()

            # img_msg.data = src_img.flatten().tolist()

            # self.pub_avmImage.publish(img_msg)     

            cv2.imshow('Parking', im0)
            key = cv2.waitKey(1)
            if key == ord('p'):
                msg = Float32MultiArray()
                msg.data = [0.0]
                self.pub_canVelData.publish(msg)     

            # print('time per detection : {:.5f}'.format(t1-t0))


    def listener(self):


        self.localizationData_sub = rospy.Subscriber("/LocalizationData", Float32MultiArray, self.localizationData_callback)
        # self.avm_img_sub = rospy.Subscriber("avm_usb_cam/image_raw", Image, self.image_callback)
        self.avm_seg_img_sub = rospy.Subscriber("/AVM_image", Image, self.image_callback)

        # self.parking_path_sub = rospy.Subscriber("/parkingPath", Path, self.parking_path_callback)

        self.parking_cand_pub = rospy.Publisher('/parking_cands', PoseArray, queue_size=1)
        self.pub_rePlanning = rospy.Publisher("replanning", Bool, queue_size=1)
        self.pub_avmImage = rospy.Publisher("/image_YOLO_result", Image, queue_size = 1)
        self.pub_canVelData = rospy.Publisher('/CanVelData2', Float32MultiArray, queue_size=1)
        

        # rospy.spin()

        while not rospy.is_shutdown():
            try:
                self.run()
            except KeyboardInterrupt:
                print("Done.")
                break

            # rospy.sleep(0.05)


def main():

    rospy.init_node('yolo_parking_spot_detection')

    Parking_spot_detection_corner_point = parking_spot_detection_corner_point()

    Parking_spot_detection_corner_point.listener()


if __name__ == "__main__":

    main()


