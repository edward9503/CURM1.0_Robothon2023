#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexiv_whole_workflow_control_flexbe_states.arm_cartesian_control_state import ArmCartesianControlState
from flexiv_whole_workflow_control_flexbe_states.arm_joint_control_state import ArmJointControlState
from flexiv_whole_workflow_control_flexbe_states.calculate_task_pose_state import CalculateTaskPoseState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
import numpy as np
# [/MANUAL_IMPORT]


'''
Created on Fri Apr 21 2023
@author: Shengzhi Wang
'''
class robothon2023_workflowSM(Behavior):
	'''
	This is the behavior for the complete task
	'''


	def __init__(self):
		super(robothon2023_workflowSM, self).__init__()
		self.name = 'robothon2023_workflow'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:141 y:560, x:496 y:359
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.probe_grasping_point_pose = np.identity(4)
		_state_machine.userdata.rotary_door_grasping_point_pose = np.identity(4)
		_state_machine.userdata.black_hole_pose = np.identity(4)
		_state_machine.userdata.red_hole_pose = np.identity(4)
		_state_machine.userdata.slider_pose = np.identity(4)
		_state_machine.userdata.red_button_pose = np.identity(4)
		_state_machine.userdata.blue_button_pose = np.identity(4)

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:577 y:69
			OperatableStateMachine.add('calculate_task_pose_state',
										CalculateTaskPoseState(),
										transitions={'done': 'joint_cmd_1', 'failed': 'calculate_task_pose_state'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'red_button_pose': 'red_button_pose', 'blue_button_pose': 'blue_button_pose', 'slider_pose': 'slider_pose', 'red_hole_pose': 'red_hole_pose', 'black_hole_pose': 'black_hole_pose', 'rotary_door_grasping_point_pose': 'rotary_door_grasping_point_pose', 'probe_grasping_point_pose': 'probe_grasping_point_pose'})

			# x:628 y:208
			OperatableStateMachine.add('joint_cmd_1',
										ArmJointControlState(input_cmd="MoveJ(target=-98.75 -16.01 9.44 124.92 -2.56 47.66 49.25)", blocking=True, clear=False),
										transitions={'done': 'joint_cmd_2', 'failed': 'joint_cmd_1'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'message': 'message'})

			# x:72 y:178
			OperatableStateMachine.add('joint_cmd_2',
										ArmJointControlState(input_cmd="MoveJ(target=-98.75 -16.01 9.44 104.92 -2.56 47.66 49.25)", blocking=True, clear=False),
										transitions={'done': 'Test_cartesian', 'failed': 'joint_cmd_2'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'message': 'message'})

			# x:206 y:416
			OperatableStateMachine.add('Test_cartesian',
										ArmCartesianControlState(task='red_hole', z_offset=1.0, blocking=True, clear=False),
										transitions={'done': 'finished', 'failed': 'Test_cartesian'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'red_button_pose': 'red_button_pose', 'blue_button_pose': 'blue_button_pose', 'slider_pose': 'slider_pose', 'red_hole_pose': 'red_hole_pose', 'black_hole_pose': 'black_hole_pose', 'rotary_door_grasping_point_pose': 'rotary_door_grasping_point_pose', 'probe_grasping_point_pose': 'probe_grasping_point_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
