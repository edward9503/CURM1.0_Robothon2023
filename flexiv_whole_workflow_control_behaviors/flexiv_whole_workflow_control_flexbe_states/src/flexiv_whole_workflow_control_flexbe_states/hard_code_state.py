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
import numpy as np

from time import sleep
from PyKDL import Frame, Rotation, Vector
import yaml



class HardCodeState(EventState):
	'''
	This state to track all key poses on box

    #> red_button_pose						T		red button pose w.r.t. robot base
    #> blue_button_pose						T		blue button pose w.r.t. robot base
    #> slider_pose							T		slider pose w.r.t. robot base
    #> red_hole_pose						T		red hole pose w.r.t. robot base
    #> black_hole_pose						T		black hole pose data w.r.t. robot base
    #> rotary_door_grasping_point_pose		T		grasping point pose data of rotary door w.r.t. robot base
    #> probe_grasping_point_pose			T		grasping point of probe w.r.t. robot base
	#> box_base_pose						T		box base pose w.r.t. robot base
	 
    <= done 											Task has been done or state is not blocking.
    <= failed 											Task is failed.
    '''

	def __init__(self,
					red_button_pose_local = [0,0,0,0,0,0],
					blue_button_pose_local = [0.0136,0,0, 0,0,0],   
					slider_pose_local = [-0.0827,0.0348,0, 0,0,0], 
					red_hole_pose_local = [-0.0113,0.0584,0, 0,0,0], 
					black_hole_pose_local =[0.0136,0.0583,0, 0,0,0], 
					rotary_door_grasping_point_pose_local = [0.0067,0.1468,0, 0,0,0], 
					rotary_door_upright_hover_pose = [-0.05326,0.1468,0.07017, 0,0,0], 
					probe_hole_local = [-0.04123,0.1468,0, 0,0,0],
					probe_grasping_point_pose_local = [0,0.2047,0, 0,0,0]
					):

		super(HardCodeState, self).__init__(input_keys=['box_base_pose','is_debug','is_sim'],
					outcomes=['done', 'failed'],
					output_keys=['red_button_pose', 'blue_button_pose', 
								'slider_pose', 'red_hole_pose', 
								'black_hole_pose', 'rotary_door_grasping_point_pose',
								'rotary_door_upright_hover_pose', 'probe_hole_pose',
								'probe_grasping_point_pose'])
		self._red_button_pose_local = self._ZYX2T(*red_button_pose_local) 
		self._blue_button_pose_local =self._ZYX2T(*blue_button_pose_local) 
		self._slider_pose_local = self._ZYX2T(*slider_pose_local) 
		self._red_hole_pose_local = self._ZYX2T(*red_hole_pose_local)
		self._black_hole_pose_local =self._ZYX2T(*black_hole_pose_local) 
		self._rotary_door_grasping_point_pose_local = self._ZYX2T(*rotary_door_grasping_point_pose_local)
		self._rotary_door_upright_hover_pose_local = self._ZYX2T(*rotary_door_upright_hover_pose)
		self._probe_hole_local = self._ZYX2T(*probe_hole_local) 
		self._probe_grasping_point_pose_local = self._ZYX2T(*probe_grasping_point_pose_local) 



	def execute(self, userdata):																						
		# The followings are only for testing at the current stage
		userdata.red_button_pose = userdata.box_base_pose * self._red_button_pose_local
		userdata.blue_button_pose = userdata.box_base_pose * self._blue_button_pose_local
		userdata.slider_pose = userdata.box_base_pose * self._slider_pose_local
		userdata.red_hole_pose = userdata.box_base_pose * self._red_hole_pose_local
		userdata.black_hole_pose = userdata.box_base_pose * self._black_hole_pose_local
		userdata.rotary_door_grasping_point_pose = userdata.box_base_pose * self._rotary_door_grasping_point_pose_local
		userdata.rotary_door_upright_hover_pose = userdata.box_base_pose * self._rotary_door_upright_hover_pose_local
		userdata.probe_hole_pose = userdata.box_base_pose * self._probe_hole_local
		userdata.probe_grasping_point_pose = userdata.box_base_pose * self._probe_grasping_point_pose_local
		
		printT = lambda _T, T_name: Logger.loghint("{}: x:{} y:{} z:{}, Rx:{}, Ry:{}, Rz:{}".format(T_name, _T.p.x(),_T.p.y(),_T.p.z(),
																			np.rad2deg(list(_T.M.GetEulerZYX())[2]),
																			np.rad2deg(list(_T.M.GetEulerZYX())[1]),
																			np.rad2deg(list(_T.M.GetEulerZYX())[0]),
																			))
		if userdata.is_debug: printT(userdata.box_base_pose, "box base w.r.t. robot base")
		if userdata.is_debug: printT(userdata.blue_button_pose, "blue button w.r.t. robot base")
		if userdata.is_debug: printT(userdata.red_hole_pose, "red hole w.r.t. robot base")
		if userdata.is_debug: printT(userdata.black_hole_pose, "black hole w.r.t. robot base")
		return 'done'

	def on_enter(self, userdata):
		pass


	#============some common functions
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

	# def _RPY2T(self, x,y, z, R, P, Y):
	# 	return Frame(Rotation.RPY(*[R,P,Y]), Vector(*[x,y,z]))

	# def _T2RPY(self, T: Frame):
	# 	pos = [T.p.x(),T.p.y(),T.p.z()]
	# 	rpy = list(T.M.GetRPY())
	# 	return np.array(pos), np.array(rpy)
	def _ZYX2T(self,x,y,z, Rx, Ry, Rz):
		return Frame(Rotation.EulerZYX(Rz, Ry, Rx),Vector(*[x,y,z]))
	def _T2ZYX(self,x,y,z, Rx, Ry, Rz):
		pos = [T.p.x(),T.p.y(),T.p.z()]
		rpy = list(T.M.GetZYX())
		return np.array(pos), np.array(rpy)