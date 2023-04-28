#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from control_msgs.msg import GripperCommand
from time import sleep

class GripperControlState(EventState):
    '''
    Publishes a position command (control_msgs/GripperCommand) message on a gripper topic.

    -- topic    string  The name of the topic which should be published on. 

    >= value            Value of desired gripper position.

    <= done             Done publishing.
    '''

    def __init__(self, topic_name):
        super(GripperControlState, self).__init__(outcomes=['done'], input_keys=['value'], output_keys=['gripper_involved'])
        self._topic = topic_name
        self._pub = ProxyPublisher({self._topic: GripperCommand})

    def execute(self, userdata):
        # sleep for 0.5 secconds, waiting for gripper reaching the desired position
        sleep(0.5)
        userdata.gripper_involved = True
        return 'done'

    def on_enter(self, userdata):
        val = GripperCommand()
        val.max_effort = 155
        val.position = userdata.value
        self._pub.publish(self._topic, val)