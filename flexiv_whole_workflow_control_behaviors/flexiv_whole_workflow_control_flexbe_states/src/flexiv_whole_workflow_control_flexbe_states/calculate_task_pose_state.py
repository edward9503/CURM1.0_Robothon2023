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
	This state is used to set the desired joint angles for the flexiv robot arm

    #> red_button_pose						object		The calibrated red button pose data
    #> blue_button_pose						object		The calibrated blue button pose data
    #> slider_pose							object		The calibrated slider pose data
    #> red_hole_pose						object		The calibrated red hole pose data
    #> black_hole_pose						object		The calibrated black hole pose data
    #> rotary_door_grasping_point_pose		object		The calibrated grasping point pose data of rotary door
    #> probe_grasping_point_pose			object		The calibrated grasping point data of probe
	 
    <= done 											Task has been done or state is not blocking.
    <= failed 											Task is failed.
    '''

	def __init__(self):
		super(CalculateTaskPoseState, self).__init__(outcomes=['done', 'failed'],
                                              output_keys=['red_button_pose', 'blue_button_pose', 
							   							   'slider_pose', 'red_hole_pose', 
														   'black_hole_pose', 'rotary_door_grasping_point_pose', 'probe_grasping_point_pose','T_RobB_BoxB'])
		self._board_pose_topic = '/robothon2023/curm2023_vision/board_pose'
		# self._arm_cmd_topic = '/arm_primitive_cmd'
		# self._pub = ProxyPublisher({self._arm_cmd_topic: String})
		self._red_button_pose_local = self._RPY2T(*[0,0,0, 0,0,0])  #np.identity(4)
		self._blue_button_pose_local =self._RPY2T(*[0.0136,0,0, 0,0,0])   #np.array([[1., 0., 0., 0.0136], [0., 1., 0., 0.], [0., 0., 1., 0.], [0., 0., 0., 1.]])
		self._slider_pose_local = self._RPY2T(*[-0.0827,0.0348,0, 0,0,0]) #np.array([[1., 0., 0., -0.0827], [0., 1., 0., 0.0348], [0., 0., 1., 0.], [0., 0., 0., 1.]])
		self._red_hole_pose_local = self._RPY2T(*[-0.0113,0.0584,0, 0,0,0]) #np.array([[1., 0., 0., -0.0113], [0., 1., 0., 0.0584], [0., 0., 1., 0.], [0., 0., 0., 1.]])
		self._black_hole_pose_local =self._RPY2T(*[0.0136,0.0583,0, 0,0,0])  #np.array([[1., 0., 0., 0.0136], [0., 1., 0., 0.0583], [0., 0., 1., 0.], [0., 0., 0., 1.]])
		self._rotary_door_grasping_point_pose_local = self._RPY2T(*[0.0067,0.1468,0, 0,0,0]) #np.array([[1., 0., 0., 0.0067], [0., 1., 0., 0.1468], [0., 0., 1., 0.], [0., 0., 0., 1.]])
		self._probe_grasping_point_pose_local = self._RPY2T(*[0,0.2047,0, 0,0,0]) #np.array([[1., 0., 0., 0.], [0., 1., 0., 0.2047], [0., 0., 1., 0.], [0., 0., 0., 1.]])
		self._connected = False

		if not self._connect():
			Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._board_pose_topic, self.name))

	def execute(self, userdata):
		if not self._connected:
			# userdata.red_button_pose = None
			# userdata.blue_button_pose = None
			# userdata.slider_pose = None
			# userdata.red_hole_pose = None
			# userdata.black_hole_pose = None
			# userdata.rotary_door_grasping_point_pose = None
			# userdata.probe_grasping_point_pose = None
			return 'failed'

		if self._sub.has_msg(self._board_pose_topic):
			rospy.loginfo(self._sub.get_last_msg(self._board_pose_topic).pose.position.z)
  

			with open("/home/ben/.ros/easy_handeye/flexiv_realsense_handeyecalibration_eye_on_base.yaml", "r") as stream:
				try:
					_obj = yaml.safe_load(stream)['transformation']
					pose_list = [_obj['x'],_obj['y'],_obj['z'], _obj['qx'],_obj['qy'],_obj['qz'],_obj['qw']]
					T_RobB_CamB = self._Quaternion2T(*pose_list)

					_pose = self._sub.get_last_msg(self._board_pose_topic)
					# if _pose is None:
					# 	Logger.loginfo("xxxx")
					# 	return 'failed'
					_pose.pose.position.x = _pose.pose.position.x/1000
					_pose.pose.position.y = _pose.pose.position.y/1000
					_pose.pose.position.z = _pose.pose.position.z/1000  
					T_CamB_BoxB = self._PoseStamped2T(_pose)
					userdata.T_RobB_BoxB = T_RobB_CamB * T_CamB_BoxB
				except yaml.YAMLError as exc:
					return 'failed'
			Logger.loginfo('All poses are obtained. Start the whole tasks.')

			# The followings are only for testing at the current stage
			userdata.red_button_pose = userdata.T_RobB_BoxB * self._red_button_pose_local
			userdata.blue_button_pose = userdata.T_RobB_BoxB * self._blue_button_pose_local
			userdata.slider_pose = userdata.T_RobB_BoxB * self._slider_pose_local
			userdata.red_hole_pose = userdata.T_RobB_BoxB * self._red_hole_pose_local
			userdata.black_hole_pose = userdata.T_RobB_BoxB * self._black_hole_pose_local
			userdata.rotary_door_grasping_point_pose = userdata.T_RobB_BoxB * self._rotary_door_grasping_point_pose_local
			userdata.probe_grasping_point_pose = userdata.T_RobB_BoxB * self._probe_grasping_point_pose_local

			_T = userdata.T_RobB_BoxB
			Logger.loginfo("box base w.r.t. robot base: x:{} y:{} z:{}, R:{}, P:{}, Y:{}".format(_T.p.x(),_T.p.y(),_T.p.z(),
																									np.rad2deg(list(_T.M.GetRPY())[0]),
																									np.rad2deg(list(_T.M.GetRPY())[1]),
																									np.rad2deg(list(_T.M.GetRPY())[2]),
																									))
			_T = userdata.blue_button_pose
			Logger.loginfo("blue w.r.t. robot base: x:{} y:{} z:{}, R:{}, P:{}, Y:{}".format(_T.p.x(),_T.p.y(),_T.p.z(),
																									np.rad2deg(list(_T.M.GetRPY())[0]),
																									np.rad2deg(list(_T.M.GetRPY())[1]),
																									np.rad2deg(list(_T.M.GetRPY())[2]),
																									))
			_T = userdata.red_hole_pose
			Logger.loginfo("red hole w.r.t. robot base: x:{} y:{} z:{}, R:{}, P:{}, Y:{}".format(_T.p.x(),_T.p.y(),_T.p.z(),
																									np.rad2deg(list(_T.M.GetRPY())[0]),
																									np.rad2deg(list(_T.M.GetRPY())[1]),
																									np.rad2deg(list(_T.M.GetRPY())[2]),
																									))
			_T = userdata.black_hole_pose
			Logger.loginfo("black hole w.r.t. robot base: x:{} y:{} z:{}, R:{}, P:{}, Y:{}".format(_T.p.x(),_T.p.y(),_T.p.z(),
																									np.rad2deg(list(_T.M.GetRPY())[0]),
																									np.rad2deg(list(_T.M.GetRPY())[1]),
																									np.rad2deg(list(_T.M.GetRPY())[2]),
																									))
			_T = T_CamB_BoxB
			Logger.loginfo("box base w.r.t. cam base: x:{} y:{} z:{}, R:{}, P:{}, Y:{}".format(_T.p.x(),_T.p.y(),_T.p.z(),
																									np.rad2deg(list(_T.M.GetRPY())[0]),
																									np.rad2deg(list(_T.M.GetRPY())[1]),
																									np.rad2deg(list(_T.M.GetRPY())[2]),
																									))
			return 'done'

	def on_enter(self, userdata):
		if not self._connected:
			if self._connect():
				Logger.loginfo('Successfully subscribed to previously failed topic %s' % self._board_pose_topic)
			else:
				Logger.logwarn('Topic %s still not available or the camera is not capturing the board.\n' 
		   					   'Please remember to run the vision nodes before running the state machines or adjust the camera...' % self._board_pose_topic)

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

	def _RPY2T(self, x,y, z, R, P, Y):
		return Frame(Rotation.RPY(*[R,P,Y]), Vector(*[x,y,z]))

	def _T2RPY(self, T: Frame):
		pos = [T.p.x(),T.p.y(),T.p.z()]
		rpy = list(T.M.GetRPY())
		return np.array(pos), np.array(rpy)