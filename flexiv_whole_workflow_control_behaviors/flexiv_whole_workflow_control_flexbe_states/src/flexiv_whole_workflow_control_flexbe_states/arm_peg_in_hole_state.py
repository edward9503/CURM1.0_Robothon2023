#!/usr/bin/env python
import rospy

import rostopic
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from flexbe_core.proxy import ProxySubscriberCached
from std_msgs.msg import String

from PyKDL import Frame, Rotation, Vector

from time import sleep


class ArmPegInHoleState(EventState):
	'''
	State to use primitive MoveJ in Flexiv

    -- contactAxis 				string 		contact (force control) axis in TCP frame, it has to be one of TCP principal axes
    -- searchAxis 				string 		search axis in TCP frame, it has to be perpendicular to the contact axis
    -- contactForce 			string 		contact force when searching on the surface
    -- radius 					string 		search area radius
    -- startDensity 			string 		the number of times the geometry pattern will be drawn in the start cycle
    -- timeFactor 				string 		time factor for how long it takes for theta to go through 2PI (setting this value too small when the radius is large, 
											i.e., drawing a large geometric pattern very fast, may cause the robot to exceed its capabilities and stop); 
											in manual mode, this factor is doubled regardless of the speed percentage
    -- wiggleRange 				string 		TCP wiggle range along the contact axis for finding the right angle when searching, 
											if this parameter is set to a large value, make sure to increase the wigglePeriod, 
											otherwise, the robot may exceed its capabilities
    -- wigglePeriod 			string 		time period for TCP to wiggle along the contact axis back and force one time 
											(setting this value too small will likely cause the robot to exceed its capabilities and stop, 
											especially when the wiggle motion is not along the Z-axis of the robot flange); 
											in manual mode, this factor is doubled regardless of the speed percentage
    -- randomFactor 			string 		a factor in [0, 1] controlling how much randomness will be added to the slide trajectory
    -- startSearchImmediately 	string 		start search immediately without waiting for contact force to reach its set value; 
											in that case, state parameter forceDrop will not be reliable
    -- searchStiffnessRatio 	string 		a factor in [0.1, 1] controlling the stiffness of the search translation motion; 
											setting this value too small, the robot may not follow the search trajectory very well, 
											especially when the surface friction or contact force is very large
	-- blocking 				bool 		Blocks until a message is received.
    -- clear 					bool 		Drops last message on this topic on enter
                    			            in order to only handle message received since this state is active.
    <= done 								Task has been done or state is not blocking.
    <= failed 								Task is failed.
    '''

	def __init__(self, contactAxis = "0 0 1",
					   searchAxis = "1 0 0",
					   contactForce = "5",
					   radius = "0.015",
					   startDensity = "2",
					   timeFactor = "2",
					   wiggleRange = "0",
					   wigglePeriod = "0.3",
					   randomFactor = "0",
					   startSearchImmediately = "0",
					   searchStiffnessRatio = "1",
					   blocking=True, 
					   clear=False):
		
		super(ArmPegInHoleState, self).__init__(input_keys=['is_debug','is_sim'],
			outcomes=['done', 'failed'])
		arm_status_topic = '/arm_task_status'
		arm_cmd_topic = '/arm_primitive_cmd'
		self._arm_status_topic = arm_status_topic
		self._arm_cmd_topic = arm_cmd_topic
		self._pub = ProxyPublisher({self._arm_cmd_topic: String})
		self._contactAxis = contactAxis
		self._searchAxis = searchAxis
		self._contactForce = contactForce
		self._radius = radius
		self._startDensity = startDensity
		self._timeFactor = timeFactor
		self._wiggleRange = wiggleRange
		self._wigglePeriod = wigglePeriod
		self._randomFactor = randomFactor
		self._startSearchImmediately = startSearchImmediately
		self._searchStiffnessRatio = searchStiffnessRatio
		self._blocking = blocking
		self._clear = clear

		# variables
		self._connected = False

	def execute(self, userdata): 
		if userdata.is_sim:
			self._connected = True
			# assert False
			Logger.loginfo('[Sucess]: SlideSpiral.')
			return 'done'   
		else:
			if not self._connected:
				# userdata.message = None
				return 'failed'

			if self._sub.has_msg(self._arm_status_topic) or not self._blocking:
				if self._sub.get_last_msg(self._arm_status_topic).data == "Done.":
					self._sub.remove_last_msg(self._arm_status_topic)
					sleep(1.0)    
					Logger.loginfo('[Sucess]: SlideSpiral.')   
					return 'done'

	def on_enter(self, userdata):
		if userdata.is_sim:
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
			_input_cmd = "SlideSpiral(contactAxis=" + self._contactAxis + ", searchAxis=" + self._searchAxis\
						 + ", contactForce=" + self._contactForce + ", radius=" + self._radius\
						 + ", startDensity=" + self._startDensity + ", timeFactor=" + self._timeFactor\
						 + ", wiggleRange=" + self._wiggleRange + ", wigglePeriod=" + self._wigglePeriod\
						 + ", randomFactor=" + self._randomFactor + ", startSearchImmediately=" + self._startSearchImmediately\
						 + ", searchStiffnessRatio=" + self._searchStiffnessRatio + ")"
			input_cmd_msg.data = _input_cmd
			self._pub.publish(self._arm_cmd_topic, input_cmd_msg)
			if userdata.is_debug: Logger.loghint("finish enter")

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

		
