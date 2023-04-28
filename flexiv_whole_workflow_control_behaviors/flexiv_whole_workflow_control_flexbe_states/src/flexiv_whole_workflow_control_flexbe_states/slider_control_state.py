#!/usr/bin/env python

# Ros related
import rospy
import rostopic
from std_msgs.msg import String, Float32MultiArray


# Flexbe related
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached

# Math related
from scipy.spatial.transform import Rotation as R
import numpy as np
from PyKDL import Frame, Rotation, Vector

from time import sleep


class SliderControlState(EventState):
	'''
	This state is used to set the desired joint angles for the flexiv robot arm

    -- task 								string		The name of the task to be executed.
    -- z_offset 							float		The vertical offset for the ee position above the task position.
														Can be used to move the ee to a task-ready position.
    -- blocking 							bool 		Blocks until a message is received.
    -- clear 								bool 		Drops last message on this topic on enter
                    						            in order to only handle message received since this state is active.
    ># red_button_pose						object		The calibrated red button pose data
    ># blue_button_pose						object		The calibrated blue button pose data
    ># slider_pose							object		The calibrated slider pose data
    ># red_hole_pose						object		The calibrated red hole pose data
    ># black_hole_pose						object		The calibrated black hole pose data
    ># rotary_door_grasping_point_pose		object		The calibrated grasping point pose data of rotary door
    ># probe_grasping_point_pose			object		The calibrated grasping point data of probe
    <= done 											Task has been done or state is not blocking.
    <= failed 											Task is failed.
    '''

	def __init__(self, z_offset=0.0, blocking=True, clear=False):
		super(SliderControlState, self).__init__(outcomes=['done', 'failed'],
                                              		   input_keys=['red_button_pose', 'blue_button_pose', 
							   							   		   'slider_pose', 'red_hole_pose', 
														   		   'black_hole_pose', 'rotary_door_grasping_point_pose', 
																   'probe_grasping_point_pose', 'T_RobB_BoxB'])
		self._arm_status_topic = '/arm_task_status'
		self._arm_cmd_topic = '/arm_primitive_cmd'
		self._screen_info_topic = '/robothon2023/curm2023_vision/screen_deltaX'
		self._pub = ProxyPublisher({self._arm_cmd_topic: String})
		self._z_offset = z_offset
		self._blocking = blocking
		self._clear = clear
		self._connected = False
		self._cmd_published = False
		self._current_task_nr = 1

		if not self._connect():
			Logger.logwarn('Topic %s and %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._arm_status_topic, self._screen_info_topic, self.name))

	def execute(self, userdata):
		if not self._connected:
			return 'failed'
		
		if self._current_task_nr == 1:
			if self._screen_info_sub.get_last_msg(self._screen_info_topic).data[0] == 4040.0:
				Logger.logwarn('The screen is not detected. \n'
		   					   'Please move the camera or board...')
				return 'failed'
			else:
				slide_error_local = Vector(self._screen_info_sub.get_last_msg(self._screen_info_topic).data[0] * scale_factor, 0, 0)
				slide_error_global = userdata.T_RobB_BoxB.M * slide_error_local
				self._current_ee_position += slide_error_global
				input_cmd_msg = String()
				input_cmd_msg.data = self._arraryCmd_to_string(Frame(userdata.slider_pose.M, self._current_ee_position))
				self._pub.publish(self._arm_cmd_topic, input_cmd_msg)
				while self._arm_status_sub.get_last_msg(self._arm_status_topic).data != "Done.":
					rospy.loginfo('I am doing')
					continue
				self._arm_status_sub.remove_last_msg(self._arm_status_topic)


		
		if not self._cmd_published:
			input_cmd_msg = String()
			if self._task == 'red_button':
				input_cmd_msg.data = self._arraryCmd_to_string(userdata.red_button_pose)
			elif self._task == 'blue_button':
				input_cmd_msg.data = self._arraryCmd_to_string(userdata.blue_button_pose)
			elif self._task == 'slider':
				input_cmd_msg.data = self._arraryCmd_to_string(userdata.slider_pose)
			elif self._task == 'red_hole':
				input_cmd_msg.data = self._arraryCmd_to_string(userdata.red_hole_pose)
			elif self._task == 'black_hole':
				input_cmd_msg.data = self._arraryCmd_to_string(userdata.black_hole_pose)
			elif self._task == 'rotary_door':
				input_cmd_msg.data = self._arraryCmd_to_string(userdata.rotary_door_grasping_point_pose)
			elif self._task == 'probe':
				input_cmd_msg.data = self._arraryCmd_to_string(userdata.probe_grasping_point_pose)
			else:
				Logger.logwarn('None task can be detected.\n'
		   					   'Please stop the whole process and edit your task name again...')
				return 'failed'
				
			self._pub.publish(self._arm_cmd_topic, input_cmd_msg)
			self._cmd_published = True

		if self._arm_status_sub.has_msg(self._arm_status_topic) or not self._blocking:
			if self._arm_status_sub.get_last_msg(self._arm_status_topic).data == "Done.":
				self._arm_status_sub.remove_last_msg(self._arm_status_topic)
				sleep(1.0)    
				Logger.loginfo('Successfully finished task.')      
				return 'done'

	def on_enter(self, userdata):
		if not self._connected:
			if self._connect():
				Logger.loginfo('Successfully subscribed to previously failed topic %s and %s' % (self._arm_status_topic, self._screen_info_topic))
			else:
				Logger.logwarn('Topic %s and %s still not available, giving up.' % (self._arm_status_topic, self._screen_info_topic))

		if self._connected and self._clear and self._arm_status_sub.has_msg(self._arm_status_topic):
			self._arm_status_sub.remove_last_msg(self._arm_status_topic)

		self._current_ee_position = userdata.slider_pose

		# publish user input command
		# input_cmd_msg = String()
		# input_cmd_msg.data = self._input_cmd
		# self._pub.publish(self._arm_cmd_topic, input_cmd_msg)

	def _connect(self):
		msg_arm_status_type, msg_arm_status_topic, _ = rostopic.get_topic_class(self._arm_status_topic)
		msg_screen_info_type, msg_screen_info_topic, _ = rostopic.get_topic_class(self._screen_info_topic)
		if msg_arm_status_topic == self._arm_status_topic and msg_screen_info_topic == self._screen_info_topic:
			self._arm_status_sub = ProxySubscriberCached({self._arm_status_topic: msg_arm_status_type})
			self._screen_info_sub = ProxySubscriberCached({self._screen_info_topic: msg_screen_info_type})
			self._connected = True
			return True
		return False
	
	def _arraryCmd_to_string(self, T):
		target_zyx_angle = list(T.M.GetEulerZYX())
		target_position = [T.p.x(), T.p.y(), T.p.z() + self._z_offset]
		cmd_string = "MoveL(target=" + self._list2str(target_position) + self._list2str(target_zyx_angle) + "WORLD WORLD_ORIGIN, maxVel=0.05)"
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
	
	def _RPY2T(self, x,y, z, R, P, Y):
		return Frame(Rotation.RPY(*[R,P,Y]), Vector(*[x,y,z]))
