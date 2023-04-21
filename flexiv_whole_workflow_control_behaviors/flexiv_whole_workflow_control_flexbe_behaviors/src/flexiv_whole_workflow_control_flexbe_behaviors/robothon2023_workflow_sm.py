#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from flexiv_whole_workflow_control_flexbe_states.arm_joint_control_state import ArmJointControlState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

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
		# x:89 y:340, x:496 y:359
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:208 y:40
			OperatableStateMachine.add('joint_cmd_1',
										ArmJointControlState(input_cmd="MoveJ(target=-98.75 -16.01 9.44 124.92 -2.56 47.66 49.25)", blocking=True, clear=False),
										transitions={'done': 'joint_cmd_2', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'message': 'message'})

			# x:36 y:133
			OperatableStateMachine.add('joint_cmd_2',
										ArmJointControlState(input_cmd="MoveJ(target=-98.75 -16.01 9.44 104.92 -2.56 47.66 49.25)", blocking=True, clear=False),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'message': 'message'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
