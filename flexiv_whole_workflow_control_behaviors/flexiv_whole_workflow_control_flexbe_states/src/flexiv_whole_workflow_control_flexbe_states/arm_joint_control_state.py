#!/usr/bin/env python
import rospy

import rostopic
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import String

from PyKDL import Frame, Rotation, Vector

from time import sleep


class ArmJointControlState(EventState):
	'''
	State to use primitive MoveJ in Flexiv

    -- input_cmd 	string		The joint command for the robot arm.
    -- blocking 	bool 		Blocks until a message is received.
    -- clear 		bool 		Drops last message on this topic on enter
                                in order to only handle message received since this state is active.
    #> message		object		Latest message on the given topic of the respective type.
    <= done 					Task has been done or state is not blocking.
    <= failed 					Task is failed.
    '''

	def __init__(self, q1,q2,q3,q4,q5,q6,q7, 
			max_cartesian_vel=0.2, 
			blocking=True, 
			clear=False):
		super(ArmJointControlState, self).__init__(input_keys=['is_debug','is_sim'],
			outcomes=['done', 'failed'])
		arm_status_topic = '/arm_task_status',
		arm_cmd_topic = '/arm_primitive_cmd'
		self._arm_status_topic = arm_status_topic
		self._arm_cmd_topic = arm_cmd_topic
		self._pub = ProxyPublisher({self._arm_cmd_topic: String})
		# self._input_cmd = input_cmd
		self._qs = [q1,q2,q3,q4,q5,q6,q7,]
		self._max_cartesian_vel= max_cartesian_vel
		self._blocking = blocking
		self._clear = clear

		# variables
		self._connected = False


			
			# Logger.logwarn('Topic %s for state %s not yet available.\n'
            #                'Will try again when entering the state...' % (self._arm_status_topic, self.name))

	def execute(self, userdata):
		if self.userdata.is_sim:
			self._connected = True
			Logger.loginfo('[Sucess]: MoveJ {}'.format(' '.join(self._qs)))
			return 'done'   
		else:
			if not self._connected:
				userdata.message = None
				return 'failed'

			if self._sub.has_msg(self._arm_status_topic) or not self._blocking:
				# userdata.message = self._sub.get_last_msg(self._arm_status_topic)
				if self._sub.get_last_msg(self._arm_status_topic).data == "Done.":
					self._sub.remove_last_msg(self._arm_status_topic)
					sleep(1.0)    
					Logger.loginfo('[Sucess]: MoveJ {}'.format(' '.join(self._qs)))      
					return 'done'

	def on_enter(self, userdata):
		if self.userdata.is_sim:
			self._connected = True
		else:
			if not self._connected:
				if self._connect():
					Logger.loginfo('Successfully subscribed to previously failed topic %s' % self._arm_status_topic)
				else:
					Logger.logwarn('Topic %s still not available, giving up.' % self._arm_status_topic)

			if self._connected and self._clear and self._sub.has_msg(self._arm_status_topic):
				self._sub.remove_last_msg(self._arm_status_topic)

		# publish user input command
		input_cmd_msg = String()
		_input_cmd = "MoveJ(target=" + ' '.join(self._qs)+ ", maxVel="+str(self._max_cartesian_vel)+")"
		input_cmd_msg.data = _input_cmd
		self._pub.publish(self._arm_cmd_topic, input_cmd_msg)

	def _connect(self):
		msg_type, msg_arm_status_topic, _ = rostopic.get_topic_class(self._arm_status_topic)
		if msg_arm_status_topic == self._arm_status_topic:
			self._sub = ProxySubscriberCached({self._arm_status_topic: msg_type})
			self._connected = True
			return True
			
		Logger.logwarn("fail to connect ros topic {}".format(self._arm_status_topic))
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

		
