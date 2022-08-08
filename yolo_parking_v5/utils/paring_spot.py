# Plotting utils
import cv2
import math
import numpy as np

from std_msgs.msg import ColorRGBA
from geometry_msgs.msg import Pose, Point

def euler_to_quaternion(yaw, pitch, roll):
    qx = np.sin(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) - np.cos(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    qy = np.cos(roll/2) * np.sin(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.cos(pitch/2) * np.sin(yaw/2)
    qz = np.cos(roll/2) * np.cos(pitch/2) * np.sin(yaw/2) - np.sin(roll/2) * np.sin(pitch/2) * np.cos(yaw/2)
    qw = np.cos(roll/2) * np.cos(pitch/2) * np.cos(yaw/2) + np.sin(roll/2) * np.sin(pitch/2) * np.sin(yaw/2)
    return qx, qy, qz, qw

def pix2world(x_pixel, y_pixel, AVM_IMG_WIDTH, AVM_IMG_HEIGHT, PIXEL_PER_METER):
    y_world = (AVM_IMG_WIDTH * 0.5 - x_pixel)/PIXEL_PER_METER
    x_world = (AVM_IMG_HEIGHT * 0.5 - y_pixel)/PIXEL_PER_METER

    return x_world, y_world

def world2pix(x_world, y_world, AVM_IMG_WIDTH, AVM_IMG_HEIGHT, PIXEL_PER_METER):
    x_pixel = (int)(AVM_IMG_WIDTH*0.5 - y_world*PIXEL_PER_METER)
    y_pixel = (int)(AVM_IMG_HEIGHT*0.5 - x_world*PIXEL_PER_METER)

    return x_pixel, y_pixel

def poseArrayFunc(x, y, theta):
    temp = Pose()
    temp.position.x = x #[m]
    temp.position.y = y #[m]
    temp.position.z = 0.2  # prob
    temp.orientation.x, temp.orientation.y, temp.orientation.z, temp.orientation.w =  euler_to_quaternion(theta, 0.0, 0.0)
    return temp

def markerFuncXY(x, y):
    temp = Point()
    temp.x = x #[m]
    temp.y = y #[m]
    temp.z = 0.2  # prob
    return temp

def markerFuncColor(cls, conf):
    temp = ColorRGBA()
    temp.r, temp.g, temp.b = (1.0, 1.0, 1.0) if cls == 0 else (0.0, 0.0, 0.0)
    temp.a = np.clip(0.15 + float(conf)*float(conf), 0.0, 1.0) #1.0
    
    return temp

### filter lower bound to upper bound of parking spot's short line
# parking_short_line_length = 2.1
# parking_long_line_length = 4.9
arrow_len = 1.3 #27.0/PIXEL_PER_METER

# compute the parking spots (poses) using corner points obtained by YOLOv5
def detect_parking_poses(im, seg_im, points, dist_range, PIXEL_PER_METER, case, AVM_IMG_WIDTH, AVM_IMG_HEIGHT, parking_long_line_length, parking_short_line_length):
    parking_poses = []
    center_poses = []
    for ii, point in enumerate(points):
        points_tmp = points
        points_tmp[ii] = (999.0, 999.0)
        idx = np.argmin(np.array([np.linalg.norm(i[:2] - np.array(point)) for i in np.array(points_tmp)]))
        paired_point = points[idx]
        parking_short_line_length_tmp = np.linalg.norm(np.array(point) - np.array(paired_point))/PIXEL_PER_METER # pix2world

        # print(point, paired_point, parking_short_line_length_tmp-dist_range, parking_short_line_length_tmp, parking_short_line_length+dist_range)
        if parking_short_line_length-dist_range < parking_short_line_length_tmp and parking_short_line_length_tmp < parking_short_line_length+dist_range:
            pointX, pointY = pix2world(point[0], point[1], AVM_IMG_WIDTH, AVM_IMG_HEIGHT, PIXEL_PER_METER)
            paired_pointX, paired_pointY = pix2world(paired_point[0], paired_point[1], AVM_IMG_WIDTH, AVM_IMG_HEIGHT, PIXEL_PER_METER)

            center_pointX = pointX + (paired_pointX - pointX)/2.0
            center_pointY = pointY + (paired_pointY - pointY)/2.0

            # to make orthogonal line form two point and cross point between the orthogonal line and the two point line.
            a = (paired_pointY - pointY)/(paired_pointX - pointX) if paired_pointX != pointX else (paired_pointY - pointY)/0.0000001
            b = -a*pointX + pointY
            crossPointX = -a*b/(a*a+1.0)
            crossPointY = b/(a*a+1.0)
            parking_spotTH = math.atan2(-(crossPointY-0.0), -(crossPointX-0.0))

            parking_long_line_half_length = parking_long_line_length*0.5 if (case == 'out') else (parking_long_line_length*0.5*0.79)
            sign = -1.0 if case == 'out' else +1.0
            parking_spotX = center_pointX + sign*parking_long_line_half_length*math.cos(parking_spotTH)
            parking_spotY = center_pointY + sign*parking_long_line_half_length*math.sin(parking_spotTH)

            parking_spotDrawX = parking_spotX - 0.5*arrow_len*math.cos(parking_spotTH)
            parking_spotDrawY = parking_spotY - 0.5*arrow_len*math.sin(parking_spotTH)
            
            parking_spotEndX = parking_spotX + 0.5*arrow_len*math.cos(parking_spotTH)
            parking_spotEndY = parking_spotY + 0.5*arrow_len*math.sin(parking_spotTH)

            parking_spotX_pix, parking_spotY_pix = world2pix(parking_spotX, parking_spotY, AVM_IMG_WIDTH, AVM_IMG_HEIGHT, PIXEL_PER_METER)
            if (parking_spotY_pix < AVM_IMG_WIDTH and parking_spotX_pix < AVM_IMG_HEIGHT) and seg_im[parking_spotY_pix][parking_spotX_pix][0] != 4:
                parking_poses.append((True, parking_spotX, parking_spotY, parking_spotTH))
                center_poses.append((center_pointX, center_pointY, parking_spotTH, sign))

                # cv2.arrowedLine(im, (world2pix(parking_spotX, parking_spotY,       AVM_IMG_WIDTH, AVM_IMG_HEIGHT, PIXEL_PER_METER)),
                cv2.arrowedLine(im, (world2pix(parking_spotDrawX, parking_spotDrawY, AVM_IMG_WIDTH, AVM_IMG_HEIGHT, PIXEL_PER_METER)),
                                    (world2pix(parking_spotEndX, parking_spotEndY,   AVM_IMG_WIDTH, AVM_IMG_HEIGHT, PIXEL_PER_METER)),
                                    (0,255,255), thickness = 2, tipLength=0.5)
 
    sorted_parking_poses = sorted(parking_poses, key=lambda x: (np.linalg.norm(x[1:3] - np.array([0.0, 0.0])), x))
    if len(center_poses) > 0:
        cloest_center_pose = sorted(center_poses, key=lambda x: (np.linalg.norm(x[0:2] - np.array([0.0, 0.0])), x))[0]
    else:
        cloest_center_pose = None

    if len(sorted_parking_poses) > 0:
        return sorted_parking_poses, cloest_center_pose
    else:
        return None, None

    # if len(parking_poses) > 0:
    #     idx = np.argmin(np.array([np.linalg.norm(i[1:3] - np.array([0.0, 0.0])) for i in np.array(parking_poses)]))
    #     return parking_poses[idx]
    # else:
    #     return False, None, None, None


# # compute the parking spot (pose) using corner points obtained by YOLOv5
# def detect_parking_pose(im, points, dist_range, img_w, img_h, PIXEL_PER_METER, case, AVM_IMG_WIDTH, AVM_IMG_HEIGHT):
#     parking_pose = []
#     ### nearest point
#     idx = np.argmin(np.array([np.linalg.norm(i[:2] - np.array([img_w/2, img_h/2])) for i in np.array(points)]))
#     nearest_point = points[idx]
#     points[idx] = (99999.0, 99999.0)
    
#     ### secondly nearest point
#     idx = np.argmin(np.array([np.linalg.norm(i[:2] - np.array(nearest_point)) for i in np.array(points)]))
#     paired_nearest_point = points[idx]
#     parking_short_line_length_tmp = np.linalg.norm(np.array(nearest_point) - np.array(paired_nearest_point))/PIXEL_PER_METER # pix2world
#     # print(parking_short_line_length_tmp)

#     # ### filter lower bound to upper bound of parking spot's short line
#     if parking_short_line_length-dist_range < parking_short_line_length_tmp and parking_short_line_length_tmp < parking_short_line_length+dist_range:
#         pointX, pointY = pix2world(nearest_point[0], nearest_point[1], AVM_IMG_WIDTH, AVM_IMG_HEIGHT, PIXEL_PER_METER)
#         paired_nearest_pointX, paired_nearest_pointY = pix2world(paired_nearest_point[0], paired_nearest_point[1], AVM_IMG_WIDTH, AVM_IMG_HEIGHT, PIXEL_PER_METER)

#         paired_pointX = pointX + (paired_nearest_pointX - pointX)/2.0
#         paired_pointY = pointY + (paired_nearest_pointY - pointY)/2.0
        
#         # to make orthogonal line form two point and cross point between the orthogonal line and the two point line.
#         a = (paired_pointY - pointY)/(paired_pointX - pointX) if paired_pointX != pointX else (paired_pointY - pointY)/0.0000001
#         b = -a*pointX + pointY
#         crossPointX = -a*b/(a*a+1.0)
#         crossPointY = b/(a*a+1.0)
#         parking_spotTH = math.atan2(-(crossPointY-0.0), -(crossPointX-0.0))

#         parking_long_line_half_length = parking_long_line_length*0.5 if (case == 'out') else (parking_long_line_length*0.5*0.79)
#         sign = -1.0 if case == 'out' else +1.0
#         parking_spotX = paired_pointX + sign*parking_long_line_half_length*math.cos(parking_spotTH)
#         parking_spotY = paired_pointY + sign*parking_long_line_half_length*math.sin(parking_spotTH)
        
#         parking_spotDrawX = parking_spotX - 0.5*arrow_len*math.cos(parking_spotTH)
#         parking_spotDrawY = parking_spotY - 0.5*arrow_len*math.sin(parking_spotTH)
        
#         parking_spotEndX = parking_spotX + 0.5*arrow_len*math.cos(parking_spotTH)
#         parking_spotEndY = parking_spotY + 0.5*arrow_len*math.sin(parking_spotTH)

#         # cv2.arrowedLine(im, (world2pix(parking_spotX, parking_spotY,       AVM_IMG_WIDTH, AVM_IMG_HEIGHT, PIXEL_PER_METER)),
#         cv2.arrowedLine(im, (world2pix(parking_spotDrawX, parking_spotDrawY, AVM_IMG_WIDTH, AVM_IMG_HEIGHT, PIXEL_PER_METER)),
#                             (world2pix(parking_spotEndX, parking_spotEndY,   AVM_IMG_WIDTH, AVM_IMG_HEIGHT, PIXEL_PER_METER)),
#                             (0,255,255), thickness = 2, tipLength=0.5)

#         parking_pose.append((True, parking_spotX, parking_spotY, parking_spotTH))
#         return parking_pose
#     else:
#         return None
