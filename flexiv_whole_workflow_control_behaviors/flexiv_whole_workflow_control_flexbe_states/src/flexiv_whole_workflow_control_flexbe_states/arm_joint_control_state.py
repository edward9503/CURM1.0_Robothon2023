#!/usr/bin/env python
import rospy

import rostopic
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import String

from time import sleep


class ArmJointControlState(EventState):
	'''
	This state is used to set the desired joint angles for the flexiv robot arm

    -- input_cmd 	string		The joint command for the robot arm.
    -- blocking 	bool 		Blocks until a message is received.
    -- clear 		bool 		Drops last message on this topic on enter
                                in order to only handle message received since this state is active.
    #> message		object		Latest message on the given topic of the respective type.
    <= done 					Task has been done or state is not blocking.
    <= failed 					Task is failed.
    '''

	def __init__(self, input_cmd, blocking=True, clear=False):
		super(ArmJointControlState, self).__init__(outcomes=['done', 'failed'],
                                              output_keys=['message'])
		self._arm_status_topic = '/arm_task_status'
		self._arm_cmd_topic = '/arm_primitive_cmd'
		self._pub = ProxyPublisher({self._arm_cmd_topic: String})
		self._input_cmd = input_cmd
		self._blocking = blocking
		self._clear = clear
		self._connected = False

		if not self._connect():
			Logger.logwarn('Topic %s for state %s not yet available.\n'
                           'Will try again when entering the state...' % (self._arm_status_topic, self.name))

	def execute(self, userdata):
		if not self._connected:
			userdata.message = None
			return 'failed'

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
		input_cmd_msg = String()
		input_cmd_msg.data = self._input_cmd
		self._pub.publish(self._arm_cmd_topic, input_cmd_msg)

	def _connect(self):
		msg_type, msg_arm_status_topic, _ = rostopic.get_topic_class(self._arm_status_topic)
		if msg_arm_status_topic == self._arm_status_topic:
			self._sub = ProxySubscriberCached({self._arm_status_topic: msg_type})
			self._connected = True
			return True
		return False
		
