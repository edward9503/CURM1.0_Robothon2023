#!/usr/bin/env python

# Ros related
import rospy
import rostopic
from std_msgs.msg import String

# Flexbe related
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached

# Math related
# from scipy.spatial.transform import Rotation as R
import numpy as np
from PyKDL import Frame, Rotation, Vector

from time import sleep


class ContactCalibration(EventState):
	'''
	State to use primitive MoveL in Flexiv

    -- offset_x 							float		offset x coordinate w.r.t. robot base
	-- offset_y 							float		offset y coordinate w.r.t. robot base
	-- offset_z 							float		offset z coordinate w.r.t. robot base
    -- offset_Rx 							float		offset x axis angle w.r.t. robot base in ZYX represntation
	-- offset_Ry 							float		offset y axis angle w.r.t. robot base in ZYX represntation
	-- offset_Rz 							float		offset z axis angle w.r.t. robot base in ZYX represntation
    -- blocking 							bool 		Blocks until a message is received.
    -- clear 								bool 		Drops last message on this topic on enter
                    						            in order to only handle message received since this state is active.
    ># target_T						        T/None		pose of target
    <= done 											Task has been done or state is not blocking.
    <= failed 											Task is failed.
    '''

	def __init__(self,contactDir='[0,0,1]',freeSpaceVel=0.01):
		super(ContactCalibration, self).__init__(outcomes=['done', 'failed'],
                                              		   input_keys=['red_button_pose', 'is_debug','is_sim'],
													   output_keys=['current_tcp'])
		self._arm_status_topic = '/arm_task_status'
		self._arm_cmd_topic = '/arm_primitive_cmd'
		self._arm_tcp_topic = '/arm_tcp_state'
		self._pub = ProxyPublisher({self._arm_cmd_topic: String})
		self._contactDir = contactDir
		self._freeSpaceVel = freeSpaceVel


		self._connected = False
 

	def execute(self, userdata):
		if userdata.is_sim:
			return 'done'
		else:
			if not self._connected:
				return 'failed'
			

			if self._sub.has_msg(self._arm_status_topic):
				# userdata.message = self._sub.get_last_msg(self._arm_status_topic)

				if self._sub.get_last_msg(self._arm_status_topic).data == "Done.":

					pose = self._sub_tcp.get_last_msg(self._arm_tcp_topic)
					T = self._PoseStamped2T(pose)
					printT = lambda _T, T_name: Logger.loghint("{}: x:{} y:{} z:{}, R:{}, P:{}, Y:{}".format(T_name, _T.p.x(),_T.p.y(),_T.p.z(),
																							np.rad2deg(list(_T.M.GetEulerZYX())[0]),
																							np.rad2deg(list(_T.M.GetEulerZYX())[1]),
																							np.rad2deg(list(_T.M.GetEulerZYX())[2]),
																							))
					if userdata.is_debug: printT(T, "current tcp w.r.t. base")
					userdata.current_tcp = T
					return 'done'
			

	def on_enter(self, userdata):
		self._connect()
		sleep(1.0)
		input_cmd_msg = String()
		input_cmd_msg.data = "AlignContact(contactDir="+self._contactDir+",freeSpaceVel="+str(self._freeSpaceVel)+")"
		input_cmd_msg.data = "AlignContact(contactDir=0 1 0,freeSpaceVel=0.02)"
		self._pub.publish(self._arm_cmd_topic, input_cmd_msg)

		
	def _connect(self):
		msg_type, msg_arm_status_topic, _ = rostopic.get_topic_class(self._arm_status_topic) 
		if msg_arm_status_topic == self._arm_status_topic:
			self._sub = ProxySubscriberCached({self._arm_status_topic: msg_type})
			self._connected = True
		else:
			self._connected = False

		 
		msg_type, msg_topic, _ = rostopic.get_topic_class(self._arm_tcp_topic) 
		if msg_topic == self._arm_tcp_topic:
			self._sub_tcp = ProxySubscriberCached({self._arm_tcp_topic: msg_type})
			self._connected = True
		else:
			self._connected = False
		Logger.loghint(f"connect: {self._connected}")

	def _list2str(self, ls):
		"""
		Convert a list to a string.

		Parameters
		----------
		ls : list
			Source list of any size.

		Returns
		----------
		str
			A string with format "ls[0] ls[1] ... ls[n] ", i.e. each value 
			followed by a space, including the last one.
		"""

		ret_str = ""
		for i in ls:
			ret_str += str(i) + " "
		return ret_str

	def _Quaternion2T(self, x,y, z, rx, ry, rz, rw):
		return Frame(Rotation.Quaternion(*[rx, ry, rz, rw]),  Vector(*[x,y,z]))
	def _PoseStamped2T(self, msg):
		""" Ros Message:PoseStamped to Frame"""
				
		x= msg.pose.position.x
		y= msg.pose.position.y
		z= msg.pose.position.z

		Qx = msg.pose.orientation.x
		Qy = msg.pose.orientation.y
		Qz = msg.pose.orientation.z
		Qw = msg.pose.orientation.w
		T = self._Quaternion2T(x, y, z, Qx,Qy,Qz,Qw)
		return T

	def _ZYX2T(self,x,y,z, Rx, Ry, Rz):
		return Frame(Rotation.EulerZYX(Rz, Ry, Rx),Vector(*[x,y,z]))
	def _T2ZYX(self,x,y,z, Rx, Ry, Rz):
		pos = [T.p.x(),T.p.y(),T.p.z()]
		rpy = list(T.M.GetZYX())
		return np.array(pos), np.array(rpy)




