# coding=utf-8

# 用户可自定义调用前缀,样例中使用了 gx
import gxipy as gx
import cv2
import time
import sys
import numpy as np
from utils import *
from imutils.perspective import four_point_transform
from imutils import contours
import imutils
import rospy
from std_msgs.msg import Float32MultiArray

# 枚举设备。
# dev_info_list 是设备信息列表,列表的元素个数为枚举到的设备个数,列表元素是字典,其中包含设备索引(index)、ip 信息(ip)等设备信息
device_manager = gx.DeviceManager()
dev_num, dev_info_list = device_manager.update_device_list()
if dev_num == 0:
    sys.exit(1)
# 打开设备
# 获取设备基本信息列表
str_sn = dev_info_list[0].get("sn")
# 通过序列号打开设备
cam = device_manager.open_device_by_sn(str_sn)
# Set parameters
float_range = cam.ExposureTime.get_range()
float_max = float_range["max"]
# cam.ExposureTime.set(15000.0)
# 开始采集
cam.stream_on()
# 获取流通道个数
# 如果 int_channel_num == 1,设备只有一个流通道,列表 data_stream 元素个数为 1
# 如果 int_channel_num > 1,设备有多个流通道,列表 data_stream 元素个数大于 1
# 目前千兆网相机、USB3.0、USB2.0 相机均不支持多流通道。
# int_channel_num = cam.get_stream_channel_num()

# Init the ros-related functions
rospy.init_node('cumr2023_vision_screen')
screen_pub = rospy.Publisher('/robothon2023/curm2023_vision/screen_deltaX', Float32MultiArray, queue_size=1)

# 获取数据
# num 为采集图片次数
while True:
    # default deltaX: 4040 means missing detection, 0/1 means exist occlusion/no occlusion
    screen_deltaX = [4040, 4040, 0]
    # 从第 0 个流通道获取一幅图像
    t1 = time.time()
    raw_image = cam.data_stream[0].get_image()
    # 从彩色原始图像获取 RGB 图像
    rgb_image = raw_image.convert("RGB")
    if rgb_image is None:
        continue
    # 从 RGB 图像数据创建 numpy 数组
    numpy_image = rgb_image.get_numpy_array()
    rgb_frame = numpy_image[:, :, ::-1]
    if rgb_frame is None:
        continue
    # 显示并保存获得的 RGB 图片
    hsv, lab, bw = getAllColorSpaces(rgb_frame)

    # detect the buttons
    # 颜色范围 HSV
    lower_red_p1 = np.array([0, 43, 46])
    upper_red_p1 = np.array([20, 255, 255])
    lower_red_p2 = np.array([156, 43, 46])
    upper_red_p2 = np.array([180, 255, 255])

    try:
        # 红色过滤
        red_mask_p1 = cv2.inRange(hsv, lower_red_p1, upper_red_p1)
        red_mask_p2 = cv2.inRange(hsv, lower_red_p2, upper_red_p2)
        red_mask = cv2.bitwise_or(red_mask_p1, red_mask_p2)
        red_mask = cv2.medianBlur(red_mask, 5)

        # dilate
        # Processing the filted red region
        kernel = np.ones((5, 5), np.uint8)
        opening = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

        # handle the extra red button
        num_opening_mask, labels_opening_mask, stats_opening_mask, centroids_opening_mask = cv2.connectedComponentsWithStats(
            opening, connectivity=8)
        area_sorted_opening_mask = np.argsort(stats_opening_mask[:, cv2.CC_STAT_AREA])[::-1]
        opening = (labels_opening_mask == area_sorted_opening_mask[1]).astype("uint8") * 255

        rgb_frame_plot = rgb_frame.copy()

        blurred = cv2.GaussianBlur(opening, (5, 5), 0)
        edged = cv2.Canny(blurred, 150, 200, 100)
        cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 创建一个黑色背景
        mask = np.zeros_like(opening)

        # 将轮廓线内部填充为白色
        mask = cv2.fillPoly(mask, cnts[0], color=255)
        # print('mask size: ', np.sum(mask/255))
        # if np.sum(mask/255) < 35000:
        #     cv2.imwrite('test.png', rgb_frame)
        cv2.imshow('mask', mask)
        screen_ = mask - opening
        ret, screen_ = cv2.threshold(screen_, 128, 255, cv2.THRESH_BINARY) # aviod residual 1.0 effect caaused by direct substract here
        # contours
        # 进行连通域分析
        num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(screen_, connectivity=8)
        # labels
        area_sorted = np.argsort(stats[:, cv2.CC_STAT_AREA])
        Attention_mask = (labels == area_sorted[-2]).astype("uint8") * 255
        blurred = cv2.GaussianBlur(Attention_mask, (5, 5), 0)
        edged = cv2.Canny(blurred, 50, 200, 255)

        # find contours in the edge map, then sort them by their
        # size in descending order
        cnts = cv2.findContours(edged.copy(), cv2.RETR_EXTERNAL,
                                cv2.CHAIN_APPROX_SIMPLE)
        cnts = imutils.grab_contours(cnts)
        cnts = sorted(cnts, key=cv2.contourArea, reverse=True)
        displayCnt = None
        # loop over the contours
        for c in cnts:
            # approximate the contour
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.02 * peri, True)
            # print('approx len: ', len(approx))
            # if the contour has four vertices, then we have found
            # the thermostat display
            if len(approx) == 4:
                displayCnt = approx
                screen_deltaX[2] = 1
                break

        warped = four_point_transform(edged, displayCnt.reshape(4, 2))
        screen = four_point_transform(rgb_frame, displayCnt.reshape(4, 2))

        # check the direction of the screen
        if screen.shape[0] > screen.shape[1]:
            screen = cv2.rotate(screen, cv2.ROTATE_90_CLOCKWISE)

        # process the screen
        screen_hsv, screen_lab, screen_bw = getAllColorSpaces(screen)

        # detect the buttons
        # 颜色范围 HSV
        lower_red_p1 = np.array([0, 43, 46])
        upper_red_p1 = np.array([10, 255, 255])
        lower_red_p2 = np.array([156, 43, 46])
        upper_red_p2 = np.array([180, 255, 255])

        lower_yellow = np.array([26, 43, 46])
        upper_yellow = np.array([34, 255, 255])

        lower_cyan = np.array([34, 43, 46])
        upper_cyan = np.array([99, 255, 255])

        kernel = np.ones((3, 3), np.uint8)

        # 红色过滤
        screen_red_mask_p1 = cv2.inRange(screen_hsv, lower_red_p1, upper_red_p1)
        screen_red_mask_p2 = cv2.inRange(screen_hsv, lower_red_p2, upper_red_p2)
        screen_red_mask = cv2.bitwise_or(screen_red_mask_p1, screen_red_mask_p2)
        screen_red_mask = cv2.morphologyEx(screen_red_mask, cv2.MORPH_OPEN, kernel)

        screen_yellow_mask = cv2.inRange(screen_hsv, lower_yellow, upper_yellow)
        screen_yellow_mask = cv2.morphologyEx(screen_yellow_mask, cv2.MORPH_OPEN, kernel)

        screen_cyan_mask = cv2.inRange(screen_hsv, lower_cyan, upper_cyan)
        screen_cyan_mask = cv2.morphologyEx(screen_cyan_mask, cv2.MORPH_OPEN, kernel)

        # RED 进行连通域分析
        num_labels_screen_red_mask, labels_screen_red_mask, stats_screen_red_mask, centroids_screen_red_mask = cv2.connectedComponentsWithStats(
            screen_red_mask, connectivity=8)
        area_sorted_screen_red_mask = np.argsort(stats_screen_red_mask[:, cv2.CC_STAT_AREA])
        screen_red_mask = (labels_screen_red_mask == area_sorted_screen_red_mask[-2]).astype("uint8") * 255

        # YELLOW
        num_labels_screen_yellow_mask, labels_screen_yellow_mask, stats_screen_yellow_mask, centroids_screen_yellow_mask = cv2.connectedComponentsWithStats(
            screen_yellow_mask, connectivity=8)
        area_sorted_screen_yellow_mask = np.argsort(stats_screen_yellow_mask[:, cv2.CC_STAT_AREA])[::-1]
        for i in range(1, area_sorted_screen_yellow_mask.shape[0]):
            if centroids_screen_red_mask[area_sorted_screen_red_mask[-2],1] > screen_red_mask.shape[0]/2.0: # screen direction is same with the camera
                if centroids_screen_yellow_mask[area_sorted_screen_yellow_mask[i], 1] > centroids_screen_red_mask[area_sorted_screen_red_mask[-2], 1] and stats_screen_yellow_mask[area_sorted_screen_yellow_mask[i], cv2.CC_STAT_AREA] / stats_screen_red_mask[area_sorted_screen_red_mask[-2], 1] > 0.5:
                    screen_yellow_mask = (labels_screen_yellow_mask == area_sorted_screen_yellow_mask[i]).astype("uint8") * 255
                    break
            else: # screen direction is different with the camera (rotated with 180)
                if centroids_screen_yellow_mask[area_sorted_screen_yellow_mask[i], 1] < centroids_screen_red_mask[area_sorted_screen_red_mask[-2], 1] and stats_screen_yellow_mask[area_sorted_screen_yellow_mask[i], cv2.CC_STAT_AREA] / stats_screen_red_mask[area_sorted_screen_red_mask[-2], 1] > 0.5:
                    screen_yellow_mask = (labels_screen_yellow_mask == area_sorted_screen_yellow_mask[i]).astype("uint8") * 255
                    break

        # CYAN
        num_labels_screen_cyan_mask, labels_screen_cyan_mask, stats_screen_cyan_mask, centroids_screen_cyan_mask = cv2.connectedComponentsWithStats(
            screen_cyan_mask, connectivity=8)
        area_sorted_screen_cyan_mask = np.argsort(stats_screen_cyan_mask[:, cv2.CC_STAT_AREA])[::-1]
        screen_cyan_mask = np.zeros_like(screen_red_mask)
        cyan_exist_flag = False
        for i in range(1, area_sorted_screen_cyan_mask.shape[0]):
            if centroids_screen_red_mask[area_sorted_screen_red_mask[-2], 1] > screen_red_mask.shape[
                0] / 2.0:  # screen direction is same with the camera
                direction = 1
                if centroids_screen_cyan_mask[area_sorted_screen_cyan_mask[i], 1] > centroids_screen_red_mask[
                    area_sorted_screen_red_mask[-2], 1] and stats_screen_cyan_mask[area_sorted_screen_cyan_mask[i], cv2.CC_STAT_AREA] / stats_screen_red_mask[area_sorted_screen_red_mask[-2], 1] > 0.5:
                    screen_cyan_mask = (labels_screen_cyan_mask == area_sorted_screen_cyan_mask[i]).astype(
                        "uint8") * 255
                    cyan_exist_flag = True
                    break
            else:  # screen direction is different with the camera (rotated with 180)
                direction = -1
                if centroids_screen_cyan_mask[area_sorted_screen_cyan_mask[i], 1] < centroids_screen_red_mask[
                    area_sorted_screen_red_mask[-2], 1] and stats_screen_cyan_mask[area_sorted_screen_cyan_mask[i], cv2.CC_STAT_AREA] / stats_screen_red_mask[area_sorted_screen_red_mask[-2], 1] > 0.5:
                    screen_cyan_mask = (labels_screen_cyan_mask == area_sorted_screen_cyan_mask[i]).astype(
                        "uint8") * 255
                    cyan_exist_flag = True
                    break

        # if centroids_screen_yellow_mask[area_sorted_screen_yellow_mask[-2], 1] > centroids_screen_red_mask[
        #     area_sorted_screen_red_mask[-2], 1]:
        #     screen_yellow_mask = (labels_screen_yellow_mask == area_sorted_screen_yellow_mask[-2]).astype("uint8") * 255
        # elif centroids_screen_yellow_mask[area_sorted_screen_yellow_mask[-3], 1] > centroids_screen_red_mask[
        #     area_sorted_screen_red_mask[-3], 1]:
        #     screen_yellow_mask = (labels_screen_yellow_mask == area_sorted_screen_yellow_mask[-3]).astype("uint8") * 255

        red_contours, red_hierarchy = cv2.findContours(screen_red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        yellow_contours, yellow_hierarchy = cv2.findContours(screen_yellow_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # 获取第一个轮廓的最小包围三角形
        red_triangle = cv2.minEnclosingTriangle(red_contours[0])
        yellow_triangle = cv2.minEnclosingTriangle(yellow_contours[0])

        # 绘制最小包围三角形
        img_draw = screen.copy()
        img_draw = cv2.drawContours(img_draw, [red_triangle[1].astype(np.int32)], 0, (0, 0, 255), 2)
        img_draw = cv2.drawContours(img_draw, [yellow_triangle[1].astype(np.int32)], 0, (0, 255, 255), 2)

        # extract the indicator point
        red_indicator_pt = cal_indicator(red_triangle[1])
        yellow_indicator_pt = cal_indicator(yellow_triangle[1])

        img_draw = cv2.circle(img_draw, (int(red_indicator_pt[0]), int(red_indicator_pt[1])), 2, (0, 0, 255), 2)
        img_draw = cv2.circle(img_draw, (int(yellow_indicator_pt[0]), int(yellow_indicator_pt[1])), 2, (0, 255, 255), 2)

        img_draw = cv2.line(img_draw, (int(red_indicator_pt[0]), 0), (int(red_indicator_pt[0]), img_draw.shape[0]),
                            (0, 0, 255), 2)
        img_draw = cv2.line(img_draw, (int(yellow_indicator_pt[0]), 0),
                            (int(yellow_indicator_pt[0]), img_draw.shape[0]), (0, 255, 255), 2)

        screen_deltaX[0] = direction * (yellow_indicator_pt[0] - red_indicator_pt[0])

        # process the cyan data
        if cyan_exist_flag is True:
            cyan_contours, cyan_hierarchy = cv2.findContours(screen_cyan_mask, cv2.RETR_EXTERNAL,
                                                             cv2.CHAIN_APPROX_SIMPLE)
            cyan_triangle = cv2.minEnclosingTriangle(cyan_contours[0])
            img_draw = cv2.drawContours(img_draw, [cyan_triangle[1].astype(np.int32)], 0, (0, 255, 0), 2)
            cyan_indicator_pt = cal_indicator(cyan_triangle[1])
            img_draw = cv2.circle(img_draw, (int(cyan_indicator_pt[0]), int(cyan_indicator_pt[1])), 2, (0, 255, 0), 2)
            img_draw = cv2.line(img_draw, (int(cyan_indicator_pt[0]), 0),
                                (int(cyan_indicator_pt[0]), img_draw.shape[0]), (0, 255, 0), 2)

            screen_deltaX[1] = direction * (cyan_indicator_pt[0] - red_indicator_pt[0])

        cv2.imshow('Rectified Screen', img_draw)
    except AttributeError:
        print('invalid estimation !')
    except:
        print('Unknown reason .')
    else:
        pass

    screen_deltaX_msg = Float32MultiArray()
    screen_deltaX_msg.data = screen_deltaX
    screen_pub.publish(screen_deltaX_msg)

    t2 = time.time()
    cv2.imshow('DaHeng image', rgb_frame)
    print('image shape: ', rgb_frame.shape, 'estimation fps: ', 1/(t2-t1))

    key = cv2.waitKey(1)

    if key & 0xFF == ord('q'):
        break
    if key & 0xFF == ord('s'):
        cv2.imwrite('test.png', rgb_frame)

# 停止采集,关闭设备
cam.stream_off()
cam.close_device()
