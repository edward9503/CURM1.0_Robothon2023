#!/usr/bin/env python
from flexbe_core import EventState, Logger
from flexbe_core.proxy import ProxyPublisher
from control_msgs.msg import GripperCommand
from time import sleep

class GripperControlState(EventState):
    '''
    Publishes a position command (control_msgs/GripperCommand) message on a gripper topic.

    -- pos_ratio        gripper position range from [0,1], 0 for close, 1 for open, example: 0.4
    >= value            Value of desired gripper position.

    <= done             Done publishing.
    '''

    def __init__(self, pos_ratio):
        super(GripperControlState, self).__init__(outcomes=['done'], input_keys=['is_sim','is_debug'])
        self._topic = "/gripper_command"
        self._pub = ProxyPublisher({self._topic: GripperCommand})
        self._pos_ratio = pos_ratio

    def execute(self, userdata):
        # sleep for 0.5 secconds, waiting for gripper reaching the desired position
        sleep(0.5)
        Logger.loginfo("[Sucess]: MoveGripper")
        return 'done'

    def on_enter(self, userdata):


        old_min = 0
        old_max = 1
        new_min = 90
        new_max = 155
        _in = 1- self._pos_ratio
        new_pos = (_in-old_min)/(old_max-old_min)*(new_max-new_min) + new_min

        if userdata.is_debug: Logger.loghint("gripper_ratio {}, gripper command: {}".format(self._pos_ratio, new_pos))
        if not userdata.is_sim:
            val = GripperCommand()
            val.position = new_pos # 155 close, 90 open
            val.max_effort = 155    
            self._pub.publish(self._topic, val)