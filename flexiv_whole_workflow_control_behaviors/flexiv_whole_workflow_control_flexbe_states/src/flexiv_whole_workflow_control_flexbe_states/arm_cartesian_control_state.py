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
from scipy.spatial.transform import Rotation as R
import numpy as np

from time import sleep


class ArmCartesianControlState(EventState):
	'''
	This state is used to set the desired joint angles for the flexiv robot arm

    -- task 								string		The name of the task to be executed.
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

	def __init__(self, task, blocking=True, clear=False):
		super(ArmCartesianControlState, self).__init__(outcomes=['done', 'failed'],
                                              		   input_keys=['red_button_pose', 'blue_button_pose', 
							   							   		   'slider_pose', 'red_hole_pose', 
														   		   'black_hole_pose', 'rotary_door_grasping_point_pose', 'probe_grasping_point_pose'])
		self._arm_status_topic = '/arm_task_status'
		self._arm_cmd_topic = '/arm_primitive_cmd'
		self._pub = ProxyPublisher({self._arm_cmd_topic: String})
		self._task = task
		self._blocking = blocking
		self._clear = clear
		self._connected = False
		self._cmd_published = False

		if not self._connect():
			Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._arm_status_topic, self.name))

	def execute(self, userdata):
		if not self._connected:
			return 'failed'
		
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

		if self._sub.has_msg(self._arm_status_topic) or not self._blocking:
            # userdata.message = self._sub.get_last_msg(self._arm_status_topic)
			if self._sub.get_last_msg(self._arm_status_topic).data == "Done.":
				self._sub.remove_last_msg(self._arm_status_topic)
				sleep(1.0)    
				Logger.loginfo('Successfully finished task.')      
				return 'done'

	def on_enter(self, userdata):
		if not self._connected:
			if self._connect():
				Logger.loginfo('Successfully subscribed to previously failed topic %s' % self._arm_status_topic)
			else:
				Logger.logwarn('Topic %s still not available, giving up.' % self._arm_status_topic)

		if self._connected and self._clear and self._sub.has_msg(self._arm_status_topic):
			self._sub.remove_last_msg(self._arm_status_topic)

		# publish user input command
		# input_cmd_msg = String()
		# input_cmd_msg.data = self._input_cmd
		# self._pub.publish(self._arm_cmd_topic, input_cmd_msg)

	def _connect(self):
		msg_type, msg_arm_status_topic, _ = rostopic.get_topic_class(self._arm_status_topic)
		if msg_arm_status_topic == self._arm_status_topic:
			self._sub = ProxySubscriberCached({self._arm_status_topic: msg_type})
			self._connected = True
			return True
		return False
	
	def _arraryCmd_to_string(self, target_pose):
		r = R.from_matrix(target_pose[0:3, 0:3])
		target_zyx_angle = r.as_euler('zyx', True).tolist()
		target_position = [target_pose[0, 3], target_pose[1, 3], target_pose[2, 3]]
		cmd_string = "MoveL(target=" + self._list2str(target_position) + self._list2str(target_zyx_angle) + "WORLD WORLD_ORIGIN, maxVel=0.3)"
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
