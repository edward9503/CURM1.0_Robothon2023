import rospy
from tf2_msgs.msg import TFMessage 
import time
from PyKDL import Frame, Rotation, Vector
import numpy as np
T = None

def callback(in_data):
    global T
    for data in in_data.transforms:
        # print(data)
        if data.child_frame_id == "aruco_marker_frame":
            # print(data)
            rx, ry, rz, rw = data.transform.rotation.x, data.transform.rotation.y, data.transform.rotation.z, data.transform.rotation.w
            x,y,z = data.transform.translation.x, data.transform.translation.y, data.transform.translation.z
            T = Frame(Rotation.Quaternion(*[rx, ry, rz, rw]),  Vector(*[x,y,z]))
if __name__ == "__main__":
    rospy.init_node('print_tf', anonymous=True)
    sub = rospy.Subscriber("/tf",TFMessage,callback)
    time.sleep(2)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        if T is not None:
            pos = [T.p.x(),T.p.y(),T.p.z()]
            rpy = list(T.M.GetRPY())
            print("x,y,z:", pos, "  R,P,Y:", list(np.rad2deg(np.array(rpy))),end="\r")
            rate.sleep()
