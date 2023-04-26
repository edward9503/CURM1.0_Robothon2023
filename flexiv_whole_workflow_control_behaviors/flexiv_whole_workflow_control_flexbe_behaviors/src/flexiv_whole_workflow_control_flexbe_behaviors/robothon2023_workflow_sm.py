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
		# x:141 y:560, x:548 y:569
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.probe_grasping_point_pose = np.identity(4)
		_state_machine.userdata.rotary_door_grasping_point_pose = np.identity(4)
		_state_machine.userdata.black_hole_pose = np.identity(4)
		_state_machine.userdata.red_hole_pose = np.identity(4)
		_state_machine.userdata.slider_pose = np.identity(4)
		_state_machine.userdata.red_button_pose = np.array([[1, 0, 0, -0.0513], [0, 1, 0, -0.006], [0, 0, 1, 0.093], [0, 0, 0, 1]])
		_state_machine.userdata.blue_button_pose = np.array([[1, 0, 0, -0.0487], [0, 1, 0, 0.0098], [0, 0, 1, 0.0928], [0, 0, 0, 1]])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:577 y:69
			OperatableStateMachine.add('calculate_task_pose_state',
										CalculateTaskPoseState(),
										transitions={'done': 'joint_to_task_ready_position', 'failed': 'calculate_task_pose_state'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'red_button_pose': 'red_button_pose', 'blue_button_pose': 'blue_button_pose', 'slider_pose': 'slider_pose', 'red_hole_pose': 'red_hole_pose', 'black_hole_pose': 'black_hole_pose', 'rotary_door_grasping_point_pose': 'rotary_door_grasping_point_pose', 'probe_grasping_point_pose': 'probe_grasping_point_pose'})

			# x:632 y:358
			OperatableStateMachine.add('go_above_blue_button',
										ArmCartesianControlState(task='blue_button', z_offset=0.3, blocking=True, clear=False),
										transitions={'done': 'press_blue_button', 'failed': 'go_above_blue_button'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'red_button_pose': 'red_button_pose', 'blue_button_pose': 'blue_button_pose', 'slider_pose': 'slider_pose', 'red_hole_pose': 'red_hole_pose', 'black_hole_pose': 'black_hole_pose', 'rotary_door_grasping_point_pose': 'rotary_door_grasping_point_pose', 'probe_grasping_point_pose': 'probe_grasping_point_pose'})

			# x:24 y:347
			OperatableStateMachine.add('go_above_red_button',
										ArmCartesianControlState(task='red_button', z_offset=0.3, blocking=True, clear=False),
										transitions={'done': 'press_red_button', 'failed': 'go_above_red_button'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'red_button_pose': 'red_button_pose', 'blue_button_pose': 'blue_button_pose', 'slider_pose': 'slider_pose', 'red_hole_pose': 'red_hole_pose', 'black_hole_pose': 'black_hole_pose', 'rotary_door_grasping_point_pose': 'rotary_door_grasping_point_pose', 'probe_grasping_point_pose': 'probe_grasping_point_pose'})

			# x:628 y:208
			OperatableStateMachine.add('joint_to_task_ready_position',
										ArmJointControlState(input_cmd="MoveJ(target=-98.75 -16.01 9.44 124.92 -2.56 47.66 49.25)", blocking=True, clear=False),
										transitions={'done': 'go_above_blue_button', 'failed': 'joint_to_task_ready_position'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'message': 'message'})

			# x:632 y:513
			OperatableStateMachine.add('press_blue_button',
										ArmCartesianControlState(task='blue_button', z_offset=0.1, blocking=True, clear=False),
										transitions={'done': 'go_above_red_button', 'failed': 'press_blue_button'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'red_button_pose': 'red_button_pose', 'blue_button_pose': 'blue_button_pose', 'slider_pose': 'slider_pose', 'red_hole_pose': 'red_hole_pose', 'black_hole_pose': 'black_hole_pose', 'rotary_door_grasping_point_pose': 'rotary_door_grasping_point_pose', 'probe_grasping_point_pose': 'probe_grasping_point_pose'})

			# x:26 y:440
			OperatableStateMachine.add('press_red_button',
										ArmCartesianControlState(task='red_button', z_offset=0.1, blocking=True, clear=False),
										transitions={'done': 'finished', 'failed': 'press_red_button'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'red_button_pose': 'red_button_pose', 'blue_button_pose': 'blue_button_pose', 'slider_pose': 'slider_pose', 'red_hole_pose': 'red_hole_pose', 'black_hole_pose': 'black_hole_pose', 'rotary_door_grasping_point_pose': 'rotary_door_grasping_point_pose', 'probe_grasping_point_pose': 'probe_grasping_point_pose'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
