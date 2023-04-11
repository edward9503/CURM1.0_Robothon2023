import cv2
import numpy as np
import rospy
from geometry_msgs.msg import PoseStamped

# plot the coordinates system in the needle
def draw_coordinate_img(img, Kc, T):
    """draw the coordinate system in the figure
    img: the figure to plot;
    Kc: the camera intrinsic parameters;
    T: the codrdinate refer to the image coordinate system;
    """
    HEIGHT, WIDTH, _ = img.shape
    len = 30  # display length for each axis
    R = T[:3, :3]
    t = T[:3, 3].reshape(-1, 1)
    proj_ori = np.dot(Kc, t)
    proj_ori = proj_ori / proj_ori[2]

    x = t + R[:, 0].reshape(-1, 1) * len
    proj_x = np.dot(Kc, x)
    proj_x = proj_x / proj_x[2]
    y = t + R[:, 1].reshape(-1, 1) * len
    proj_y = np.dot(Kc, y)
    proj_y = proj_y / proj_y[2]
    z = t + R[:, 2].reshape(-1, 1) * len
    proj_z = np.dot(Kc, z)
    proj_z = proj_z / proj_z[2]

    img = cv2.line(img, tuple(proj_ori[:2, 0].astype(int)), tuple(proj_x[:2, 0].astype(int)), (0, 0, 255), 4)
    img = cv2.line(img, tuple(proj_ori[:2, 0].astype(int)), tuple(proj_y[:2, 0].astype(int)), (0, 255, 0), 4)
    img = cv2.line(img, tuple(proj_ori[:2, 0].astype(int)), tuple(proj_z[:2, 0].astype(int)), (255, 0, 0), 4)

    return img

# calculate the points with fixed depth assumption
# dis, np.argmin(dis), red_circles
def deproj(Kc, ImgPt, depth):
    ImgPt = ImgPt.reshape(-1,)
    return np.linalg.pinv(Kc) @ np.array([ImgPt[0], ImgPt[1], 1]) * depth

def getRoi(image, min_row, max_row, min_col,max_col):
    mask = np.zeros(image.shape[:2],np.uint8)
    mask[min_row:max_row, min_col:max_col] = 255
    new_img = cv2.bitwise_and(image,image,mask = mask)
    return new_img

def getAllColorSpaces(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2Lab)
    bw  = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    return hsv, lab, bw

def cal_indicator(triangle_pts):
    pt = triangle_pts
    y_dis = np.array([abs(pt[0, 0, 1]-pt[1, 0, 1]), abs(pt[0, 0, 1]-pt[2, 0, 1]), abs(pt[1, 0, 1]-pt[2, 0, 1])])
    indicator = 2 - np.argmin(y_dis)
    return pt[indicator, 0, :]

def frame_to_pose_stamped_msg(frame):
    """

    :param frame:
    :return:
    """
    msg = PoseStamped()
    msg.header.stamp = rospy.Time.now()
    msg.pose.position.x = frame.p[0]
    msg.pose.position.y = frame.p[1]
    msg.pose.position.z = frame.p[2]

    msg.pose.orientation.x = frame.M.GetQuaternion()[0]
    msg.pose.orientation.y = frame.M.GetQuaternion()[1]
    msg.pose.orientation.z = frame.M.GetQuaternion()[2]
    msg.pose.orientation.w = frame.M.GetQuaternion()[3]

    return msg