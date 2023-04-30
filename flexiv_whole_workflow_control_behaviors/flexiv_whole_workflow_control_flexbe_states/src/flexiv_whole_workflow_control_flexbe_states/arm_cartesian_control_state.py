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


class ArmCartesianControlState(EventState):
	'''
	State to use primitive MoveL in Flexiv

    -- offset_x 							float		offset x coordinate w.r.t. robot base
	-- offset_y 							float		offset y coordinate w.r.t. robot base
	-- offset_z 							float		offset z coordinate w.r.t. robot base
    -- offset_Rx 							float		offset x axis angle w.r.t. robot base in ZYX represntation
	-- offset_Ry 							float		offset y axis angle w.r.t. robot base in ZYX represntation
	-- offset_Rz 							float		offset z axis angle w.r.t. robot base in ZYX represntation
	-- offset_reference                     string      local or wolrd
    -- blocking 							bool 		Blocks until a message is received.
    -- clear 								bool 		Drops last message on this topic on enter
                    						            in order to only handle message received since this state is active.
    ># target_T						        T/None		pose of target
    <= done 											Task has been done or state is not blocking.
    <= failed 											Task is failed.
    '''

	def __init__(self, offset_x=0.0,
						offset_y=0.0,
						offset_z=0.0,
						offset_Rx=0.0,
						offset_Ry=0.0,
						offset_Rz=0.0,
						offset_reference="local",
						maxvel=0.07,
						blocking=True, 
						clear=False):
		super(ArmCartesianControlState, self).__init__(outcomes=['done', 'failed'],
                                              		   input_keys=['target_T', 'is_debug','is_sim'])
		self._arm_status_topic = '/arm_task_status'
		self._arm_cmd_topic = '/arm_primitive_cmd'
		self._pub = ProxyPublisher({self._arm_cmd_topic: String})
		self._offset_pos = [offset_x, offset_y, offset_z]
		self._offset_rot = [offset_Rx, offset_Ry, offset_Rz]
		self._offset_reference = offset_reference
		self._blocking = blocking
		self._clear = clear
		self._maxvel = maxvel
		self._connected = False
		self._cmd_published = False
		

		if not self._connect():
			Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._arm_status_topic, self.name))

	def execute(self, userdata):
		if userdata.is_sim:
			# if userdata.is_debug: Logger.loghint('Successfully finished task.')
			# Logger.loginfo('[Sucess]: MoveL pos:{}, rot:{}'.format(' '.join([str(q) for q in self._offset_pos]),
			# 														' '.join([str(q) for q in str(self._offset_rot)])))
			# printT = lambda _T, T_name: Logger.loginfo("{}: x:{} y:{} z:{}, R:{}, P:{}, Y:{}".format(T_name, _T.p.x(),_T.p.y(),_T.p.z(),
			# 																				np.rad2deg(list(_T.M.GetRPY())[0]),
			# 																				np.rad2deg(list(_T.M.GetRPY())[1]),
			# 																				np.rad2deg(list(_T.M.GetRPY())[2]),
			# 																				))
			# if userdata.is_debug: printT(userdata.target_T, "TargetT ") 
			return 'done'
		else:
			if not self._connected:
				return 'failed'
			
			if not self._cmd_published:
				input_cmd_msg = String()
				# if userdata.target_T is None:
				# 	T = self._ZYX2T(*[0,0,0, 0,0,0])
				# else:
				T = userdata.target_T
				input_cmd_msg.data = self._arraryCmd_to_string(T_des=T)
					
				self._pub.publish(self._arm_cmd_topic, input_cmd_msg)
				self._cmd_published = True

			if self._sub.has_msg(self._arm_status_topic) or not self._blocking:
				# userdata.message = self._sub.get_last_msg(self._arm_status_topic)
				if self._sub.get_last_msg(self._arm_status_topic).data == "Done.":
					self._sub.remove_last_msg(self._arm_status_topic)
					sleep(1.0)    
					Logger.loginfo('Successfully finished task.')      
					return 'done'

	def on_enter(self, userdata):
		if userdata.is_sim:
			pass
		else:
			if not self._connected:
				if self._connect():
					Logger.loginfo('Successfully subscribed to previously failed topic %s' % self._arm_status_topic)
				else:
					Logger.logwarn('Topic %s still not available, giving up.' % self._arm_status_topic)

			if self._connected and self._clear and self._sub.has_msg(self._arm_status_topic):
				self._sub.remove_last_msg(self._arm_status_topic)
		

	def _connect(self):
		msg_type, msg_arm_status_topic, _ = rostopic.get_topic_class(self._arm_status_topic)
		if msg_arm_status_topic == self._arm_status_topic:
			self._sub = ProxySubscriberCached({self._arm_status_topic: msg_type})
			self._connected = True
			return True
		return False
	
	def _arraryCmd_to_string(self, T_des):
		if self._offset_reference == "local":
			T = T_des *  self._ZYX2T(*(self._offset_rot+self._offset_rot))
		elif self._offset_reference == "local":
			T = self._ZYX2T(*(self._offset_rot+self._offset_rot)) * T_des 
		else:
			raise NotImplementedError
		target_zyx_angle = list(T.M.GetEulerZYX())
		target_position = [T.p.x(), T.p.y(), T.p.z()]
		cmd_string = "MoveL(target=" + self._list2str(target_position) + self._list2str(target_zyx_angle) + "WORLD WORLD_ORIGIN, maxVel="+str(self._maxvel)+")"
		return cmd_string

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

	# def _RPY2T(self, x,y, z, R, P, Y):
	# 	return Frame(Rotation.RPY(*[R,P,Y]), Vector(*[x,y,z]))

	# def _T2RPY(self, T: Frame):
	# 	pos = [T.p.x(),T.p.y(),T.p.z()]
	# 	rpy = list(T.M.GetRPY())
	# 	return np.array(pos), np.array(rpy)


