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



class CalculateTaskPoseState(EventState):
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
					probe_grasping_point_pose_local = [0,0.2047,0, 0,0,0]
					):

		super(CalculateTaskPoseState, self).__init__(input_keys=['is_debug','is_sim'],
					outcomes=['done', 'failed'],
					output_keys=['red_button_pose', 'blue_button_pose', 
								'slider_pose', 'red_hole_pose', 
								'black_hole_pose', 'rotary_door_grasping_point_pose', 
								'probe_grasping_point_pose','box_base_pose'])
		board_pose_topic="/robothon2023/curm2023_vision/board_pose"
		rs_cal_file="/home/ben/.ros/easy_handeye/flexiv_realsense_handeyecalibration_eye_on_base.yaml"
		self._board_pose_topic = board_pose_topic
		self._red_button_pose_local = self._ZYX2T(*red_button_pose_local) 
		self._blue_button_pose_local =self._ZYX2T(*blue_button_pose_local) 
		self._slider_pose_local = self._ZYX2T(*slider_pose_local) 
		self._red_hole_pose_local = self._ZYX2T(*red_hole_pose_local)
		self._black_hole_pose_local =self._ZYX2T(*black_hole_pose_local) 
		self._rotary_door_grasping_point_pose_local = self._ZYX2T(*rotary_door_grasping_point_pose_local) 
		self._probe_grasping_point_pose_local = self._ZYX2T(*probe_grasping_point_pose_local) 
		self._rs_cal_file = rs_cal_file

		# init vars
		self._connected = False


	def execute(self, userdata):
		if userdata.is_sim:
			if userdata.is_debug: Logger.loghint("test...")
			printT = lambda _T, T_name: Logger.loginfo("{}: x:{} y:{} z:{}, R:{}, P:{}, Y:{}".format(T_name, _T.p.x(),_T.p.y(),_T.p.z(),
																										np.rad2deg(list(_T.M.GetRPY())[0]),
																										np.rad2deg(list(_T.M.GetRPY())[1]),
																										np.rad2deg(list(_T.M.GetRPY())[2]),
																										))
			if userdata.is_debug: printT(self._ZYX2T(0,0,0, 0,0,0), "testT") 
			return "done"
		else:
			if not self._connected:
				if userdata.is_debug: Logger.loghint("failed: not getting cam ros topic...")
				return 'failed'

			if self._sub.has_msg(self._board_pose_topic):
				# if userdata.is_debug: Logger.loghint(self._sub.get_last_msg(self._board_pose_topic).pose.position.z)
	

				with open(self._rs_cal_file, "r") as stream:
					try:
						_obj = yaml.safe_load(stream)['transformation']
						pose_list = [_obj['x'],_obj['y'],_obj['z'], _obj['qx'],_obj['qy'],_obj['qz'],_obj['qw']]
						T_RobB_CamB = self._Quaternion2T(*pose_list)

						_pose = self._sub.get_last_msg(self._board_pose_topic)
						_pose.pose.position.x = _pose.pose.position.x/1000
						_pose.pose.position.y = _pose.pose.position.y/1000
						_pose.pose.position.z = _pose.pose.position.z/1000  
						T_CamB_BoxB = self._PoseStamped2T(_pose)
						userdata.box_base_pose = T_RobB_CamB * T_CamB_BoxB
					except yaml.YAMLError as exc:
						return 'failed'
				Logger.loginfo('All poses are obtained. Start the whole tasks.')

				# The followings are only for testing at the current stage
				userdata.red_button_pose = userdata.box_base_pose * self._red_button_pose_local
				userdata.blue_button_pose = userdata.box_base_pose * self._blue_button_pose_local
				userdata.slider_pose = userdata.box_base_pose * self._slider_pose_local
				userdata.red_hole_pose = userdata.box_base_pose * self._red_hole_pose_local
				userdata.black_hole_pose = userdata.box_base_pose * self._black_hole_pose_local
				userdata.rotary_door_grasping_point_pose = userdata.box_base_pose * self._rotary_door_grasping_point_pose_local
				userdata.probe_grasping_point_pose = userdata.box_base_pose * self._probe_grasping_point_pose_local


				printT = lambda _T, T_name: Logger.loghint("{}: x:{} y:{} z:{}, R:{}, P:{}, Y:{}".format(T_name, _T.p.x(),_T.p.y(),_T.p.z(),
																							np.rad2deg(list(_T.M.GetRPY())[0]),
																							np.rad2deg(list(_T.M.GetRPY())[1]),
																							np.rad2deg(list(_T.M.GetRPY())[2]),
																							))
				if userdata.is_debug: printT(userdata.box_base_pose, "box base w.r.t. robot base")
				if userdata.is_debug: printT(userdata.blue_button_pose, "blue button w.r.t. robot base")
				if userdata.is_debug: printT(userdata.red_hole_pose, "red hole w.r.t. robot base")
				if userdata.is_debug: printT(userdata.black_hole_pose, "box base w.r.t. robot base")

				return 'done'

	def on_enter(self, userdata):
		if userdata.is_sim:
			self._connected = True
		else: 
			if self._connect():
				Logger.loginfo('Successfully subscribed to previously failed topic %s' % self._board_pose_topic)
				pass
			else:
				# Logger.logwarn('Topic %s still not available or the camera is not capturing the board.\n' 
				# 			'Please remember to run the vision nodes before running the state machines or adjust the camera...' % self._board_pose_topic)
				Logger.logwarn('cannot find for cam ros topic..')

	def _connect(self):
		msg_type, msg_board_pose_topic, _ = rostopic.get_topic_class(self._board_pose_topic)
		if msg_board_pose_topic == self._board_pose_topic:
			self._sub = ProxySubscriberCached({self._board_pose_topic: msg_type})
			sleep(1.0) # need to wait to get msg after initalize  
			if self._sub.get_last_msg(self._board_pose_topic).pose.position.z == -1.0:
				self._connected = False
				return False
			else:
				self._connected = True
				return True
		return False

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