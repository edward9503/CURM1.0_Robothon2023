#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from math import pi
import numpy as np

def fake_joint_cmd_msg(jnt_pos):
    assert len(jnt_pos) == 7
    msg = JointState()
    msg.name = ["joint1","joint2","joint3","joint4","joint5","joint6","joint7",]
    msg.position = jnt_pos
    return msg

def _main():
    hz = 100
    loop = 2
    init_pos = [0,0,0,0,0,0,0,]
    amp = pi/5
    pub = rospy.Publisher('/fake_joint_cmd', JointState, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(hz)
    t = 0
    while not rospy.is_shutdown():
        t+=1/hz
        pos = np.array(init_pos) + np.sin(2*pi*t/loop)*amp*np.ones(7)
        msg = fake_joint_cmd_msg(pos.tolist())
        pub.publish(msg)
        rate.sleep()
if __name__ == '__main__':
    try:
        _main()
    except rospy.ROSInterruptException:
        pass