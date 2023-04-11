# !/usr/bin/env python
import rospy
import roslib
import sys
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from std_msgs.msg import *
from sensor_msgs.msg import *
import os
from utils import *
import time
from geometry_msgs.msg import PoseStamped
from PyKDL import Frame, Rotation, Vector

#########################################################
##################################### Modified parameters
#########################################################
Kc = np.array([[902.444304254649, 0, 643.936533278185],
               [0, 902.08099534391, 358.417078510844],
               [0, 0, 1]])
depth = 435 # mm: depth between the plane and the camera
IoU_threshold = 0.65 # used for circle check

theta = -102 / 180 * np.pi
st = np.sin(theta)
ct = np.cos(theta)
RotZ = np.array([[ct, -st, 0, 0],
                [st, ct, 0, 0],
                [0, 0, 1, 0],
                [0, 0, 0, 1]])

# 颜色范围 HSV
lower_red_p1 = np.array([0, 43, 46])
upper_red_p1 = np.array([10, 255, 255])
lower_red_p2 = np.array([156, 43, 46])
upper_red_p2 = np.array([180, 255, 255])
#########################################################
#########################################################

SAVE_FILE_DIR = './data/sampled_images/' # hand2eye_calibration

if not os.path.exists(SAVE_FILE_DIR):
    os.makedirs(SAVE_FILE_DIR)

bridge = CvBridge()
joint_pos = None
rgb_frame = None
sample_count = 0

def jointStateCallback(jointState):
    global joint_pos
    joint_pos = np.array(jointState.position, dtype=float)

def imageCallback(img):
    global rgb_frame
    rgb_frame = bridge.imgmsg_to_cv2(img, "bgr8")
    W, H = rgb_frame.shape[:2]

rospy.init_node('board_pose_node', anonymous=True)
rospy.Subscriber('/camera/color/image_raw', Image, imageCallback, queue_size=1)
board_pub = rospy.Publisher('/robothon2023/curm2023_vision/board_pose', PoseStamped, queue_size=1)
# rospy.Subscriber('joint_states', JointState, jointStateCallback)
prev_time = time.time()
while not rospy.is_shutdown():
    # Init the Pose
    T_output = np.array([[1, 0, 0, 0],
                         [0, 1, 0, 0],
                         [0, 0, 1, -1]])

    if rgb_frame is not None:
        # crop_img = getRoi(rgb_frame, 124, 695, 370, 1150)
        crop_img = rgb_frame.copy()
        rgb_frame_plot = rgb_frame.copy()

        hsv, lab, bw = getAllColorSpaces(crop_img)

        # detect the buttons
        red_mask_p1 = cv2.inRange(hsv, lower_red_p1, upper_red_p1)
        red_mask_p2 = cv2.inRange(hsv, lower_red_p2, upper_red_p2)
        red_mask = cv2.bitwise_or(red_mask_p1, red_mask_p2)
        red_mask = cv2.medianBlur(red_mask, 5)

        try:
            # dilate
            # Processing the filted red region
            kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            dilate = cv2.dilate(red_mask, kernel, iterations=1)
            erode = cv2.erode(dilate, kernel, iterations=1)

            # rgb_frame_plot = rgb_frame.copy()
            num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(erode, connectivity=8)
            # labels
            area_sorted = np.argsort(stats[:, cv2.CC_STAT_AREA])
            red_screen = centroids[area_sorted[-2], :]
            red_circles = centroids[area_sorted[-4:-2], :]
            # visualize the attention mask
            Attention_mask = np.zeros_like(labels, dtype=np.uint8)
            for i in area_sorted[-4:-1]:
                temp = (labels == i).astype("uint8") * 255
                Attention_mask += temp

            cv2.imshow('Board_Attention_mask', Attention_mask)
            # check whether the current mask is a circle
            circles_list = [-4, -3]
            for circle_check_ind in circles_list:
                mask = (labels == area_sorted[circle_check_ind])
                area = stats[area_sorted[circle_check_ind], cv2.CC_STAT_AREA]
                radius = np.sqrt(area / 3.14)

                check_region = np.zeros_like(labels)
                red_circle = centroids[area_sorted[circle_check_ind], :]
                check_region[int(red_circle[1] - radius):int(red_circle[1] + radius),
                int(red_circle[0] - radius):int(red_circle[0] + radius)] = 1
                union = np.bitwise_or(mask, check_region)
                cross = np.bitwise_and(mask, check_region)
                IoU = np.sum(cross) / np.sum(union)
                assert IoU > IoU_threshold

            # determine the red button/port
            dis = np.linalg.norm(red_circles - red_screen, axis=1)
            red_button_ind = np.argmin(dis)
            red_port_ind = np.argmax(dis)
            red_button = red_circles[red_button_ind, :]
            red_port = red_circles[red_port_ind, :]
            print('red button: ', red_button, ' red_port: ', red_port, ' red_screen: ', red_screen)

            redButton_pt = deproj(Kc, red_button, depth)  # mm
            redPort_pt = deproj(Kc, red_port, depth)  # mm

            rb2rp = redPort_pt - redButton_pt

            x_axis = rb2rp / np.linalg.norm(rb2rp)
            z_axis = np.array([0, 0, -1])
            y_axis = np.cross(z_axis, x_axis)
            origin = redButton_pt

            T = np.vstack((x_axis, y_axis, z_axis, origin)).T  # , x_axis, y_axis, z_axis, origin

            T_output = T @ RotZ
            rgb_frame_plot = draw_coordinate_img(rgb_frame_plot, Kc, T_output)
            rgb_frame_plot = cv2.rectangle(rgb_frame_plot, (0, 0), (rgb_frame_plot.shape[1] - 1, rgb_frame_plot.shape[0] - 1), (0, 255, 0), 2)

        except AssertionError:
            rgb_frame_plot = cv2.rectangle(rgb_frame_plot, (0, 0), (rgb_frame_plot.shape[1] - 1, rgb_frame_plot.shape[0] - 1), (0, 0, 255), 2)
            print('AssertionError --> invalid estimation!')
        except IndexError:
            print('IndexError --> invalid estimation')
        except:
            print('Unknown reason .')
        else:
            pass

        curr_time = time.time()
        fps = int(1/(curr_time - prev_time))
        cv2.putText(rgb_frame_plot, f"FPS: {fps}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 0), 2)
        cv2.imshow('Realsense image', rgb_frame_plot)
        prev_time = curr_time

    Rot = Rotation(T_output[0, 0], T_output[0, 1], T_output[0, 2],
                   T_output[1, 0], T_output[1, 1], T_output[1, 2],
                   T_output[2, 0], T_output[2, 1], T_output[2, 2])
    t = Vector(T_output[0, 3], T_output[1, 3], T_output[2, 3])
    T_frame = Frame(Rot, t)
    board_pose_msg = frame_to_pose_stamped_msg(T_frame)
    board_pub.publish(board_pose_msg)
    key = cv2.waitKey(1)

    if key & 0xFF == ord('s'):
        sample_count = sample_count + 1
        cv2.imwrite(SAVE_FILE_DIR + "RImage" + str(sample_count).zfill(2) + ".jpg", rgb_frame)

    if key & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
