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
from flexiv_whole_workflow_control_flexbe_states.gripper_control_state import GripperControlState
from flexiv_whole_workflow_control_flexbe_states.hard_code_state import HardCodeState
from flexiv_whole_workflow_control_flexbe_states.slider_control_state import SliderControlState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]
from PyKDL import Frame, Rotation, Vector
import math
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
		# x:141 y:560, x:548 y:592
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.probe_grasping_point_pose = None
		_state_machine.userdata.rotary_door_grasping_point_pose = Frame(Rotation.EulerZYX(math.radians(45), math.radians(0), math.radians(0)),Vector(-0.291641,0.526752,0.102372))
		_state_machine.userdata.black_hole_pose = None
		_state_machine.userdata.red_hole_pose = None
		_state_machine.userdata.slider_pose = None
		_state_machine.userdata.red_button_pose = None
		_state_machine.userdata.blue_button_pose = None
		_state_machine.userdata.T_RobB_BoxB = Frame(Rotation.EulerZYX(math.radians(45), math.radians(0), math.radians(0)),Vector(-0.231681,0.526752,0.172542))
		_state_machine.userdata.is_sim = False
		_state_machine.userdata.is_debug = True
		_state_machine.userdata.probe_hole_pose = None
		_state_machine.userdata.rotary_door_upright_hover_pose = Frame(Rotation.EulerZYX(math.radians(45), math.radians(0), math.radians(0)),Vector(-0.231681,0.526752,0.172542))

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:472, x:130 y:472
		_sm_5_wrap_cable_and_replace_probe_2_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['blue_button_pose', 'is_sim', 'is_debug'])

		with _sm_5_wrap_cable_and_replace_probe_2_0:
			# x:208 y:167
			OperatableStateMachine.add('hover blue button',
										ArmCartesianControlState(offset_x=0.0, offset_y=0.0, offset_z=0.0, offset_Rx=0.0, offset_Ry=0.0, offset_Rz=0.0, offset_reference="local", maxvel=0.07, blocking=True, clear=False),
										transitions={'done': 'press blue button', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_T': 'blue_button_pose', 'is_debug': 'is_debug', 'is_sim': 'is_sim'})

			# x:663 y:250
			OperatableStateMachine.add('press blue button',
										ArmCartesianControlState(offset_x=0.0, offset_y=0.0, offset_z=0.0, offset_Rx=0.0, offset_Ry=0.0, offset_Rz=0.0, offset_reference="local", maxvel=0.07, blocking=True, clear=False),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_T': 'blue_button_pose', 'is_debug': 'is_debug', 'is_sim': 'is_sim'})


		# x:745 y:485, x:94 y:456
		_sm_4_open_door_and_probe_circuit_1 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['is_sim', 'is_debug', 'rotary_door_upright_hover_pose', 'rotary_door_grasping_point_pose'])

		with _sm_4_open_door_and_probe_circuit_1:
			# x:208 y:167
			OperatableStateMachine.add('hover door grasping point',
										ArmCartesianControlState(offset_x=0.0, offset_y=0.0, offset_z=0.06, offset_Rx=0.0, offset_Ry=0.0, offset_Rz=0.0, offset_reference="global", maxvel=0.07, blocking=True, clear=False),
										transitions={'done': 'approach door grasping point', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_T': 'rotary_door_grasping_point_pose', 'is_debug': 'is_debug', 'is_sim': 'is_sim'})

			# x:663 y:346
			OperatableStateMachine.add('open door',
										ArmCartesianControlState(offset_x=0.0, offset_y=0.0, offset_z=0.0, offset_Rx=0.0, offset_Ry=0.0, offset_Rz=0.0, offset_reference="global", maxvel=0.07, blocking=True, clear=False),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_T': 'rotary_door_upright_hover_pose', 'is_debug': 'is_debug', 'is_sim': 'is_sim'})

			# x:663 y:250
			OperatableStateMachine.add('approach door grasping point',
										ArmCartesianControlState(offset_x=0.0, offset_y=0.0, offset_z=0.0, offset_Rx=0.0, offset_Ry=0.0, offset_Rz=0.0, offset_reference="global", maxvel=0.07, blocking=True, clear=False),
										transitions={'done': 'open door', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_T': 'rotary_door_grasping_point_pose', 'is_debug': 'is_debug', 'is_sim': 'is_sim'})


		# x:30 y:365, x:130 y:365
		_sm_3_plug_probe_to_test_port_2 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['blue_button_pose', 'is_sim', 'is_debug'])

		with _sm_3_plug_probe_to_test_port_2:
			# x:208 y:167
			OperatableStateMachine.add('hover blue button',
										ArmCartesianControlState(offset_x=0.0, offset_y=0.0, offset_z=0.0, offset_Rx=0.0, offset_Ry=0.0, offset_Rz=0.0, offset_reference="local", maxvel=0.07, blocking=True, clear=False),
										transitions={'done': 'press blue button', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_T': 'blue_button_pose', 'is_debug': 'is_debug', 'is_sim': 'is_sim'})

			# x:663 y:250
			OperatableStateMachine.add('press blue button',
										ArmCartesianControlState(offset_x=0.0, offset_y=0.0, offset_z=0.0, offset_Rx=0.0, offset_Ry=0.0, offset_Rz=0.0, offset_reference="local", maxvel=0.07, blocking=True, clear=False),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_T': 'blue_button_pose', 'is_debug': 'is_debug', 'is_sim': 'is_sim'})


		# x:30 y:365, x:130 y:365
		_sm_2_move_slider_3 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['blue_button_pose', 'is_sim', 'is_debug'])

		with _sm_2_move_slider_3:
			# x:208 y:167
			OperatableStateMachine.add('hover blue button',
										ArmCartesianControlState(offset_x=0.0, offset_y=0.0, offset_z=0.0, offset_Rx=0.0, offset_Ry=0.0, offset_Rz=0.0, offset_reference="local", maxvel=0.07, blocking=True, clear=False),
										transitions={'done': 'press blue button', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_T': 'blue_button_pose', 'is_debug': 'is_debug', 'is_sim': 'is_sim'})

			# x:663 y:250
			OperatableStateMachine.add('press blue button',
										ArmCartesianControlState(offset_x=0.0, offset_y=0.0, offset_z=0.0, offset_Rx=0.0, offset_Ry=0.0, offset_Rz=0.0, offset_reference="local", maxvel=0.07, blocking=True, clear=False),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_T': 'blue_button_pose', 'is_debug': 'is_debug', 'is_sim': 'is_sim'})


		# x:30 y:472, x:130 y:472
		_sm_1_press_start_button_4 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['blue_button_pose', 'is_sim', 'is_debug'])

		with _sm_1_press_start_button_4:
			# x:208 y:167
			OperatableStateMachine.add('hover blue button',
										ArmCartesianControlState(offset_x=0.0, offset_y=0.0, offset_z=0.0, offset_Rx=0.0, offset_Ry=0.0, offset_Rz=0.05, offset_reference="local", maxvel=0.07, blocking=True, clear=False),
										transitions={'done': 'press blue button', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_T': 'blue_button_pose', 'is_debug': 'is_debug', 'is_sim': 'is_sim'})

			# x:663 y:250
			OperatableStateMachine.add('press blue button',
										ArmCartesianControlState(offset_x=0.0, offset_y=0.0, offset_z=0.0, offset_Rx=0.0, offset_Ry=0.0, offset_Rz=0.01, offset_reference="local", maxvel=0.07, blocking=True, clear=False),
										transitions={'done': 'finished', 'failed': 'failed'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_T': 'blue_button_pose', 'is_debug': 'is_debug', 'is_sim': 'is_sim'})



		with _state_machine:
			# x:372 y:103
			OperatableStateMachine.add('magicstate',
										HardCodeState(red_button_pose_local=[0,0,0,0,0,0], blue_button_pose_local=[0.0136,0,0,0,0,0], slider_pose_local=[-0.0827,0.0348,0,0,0,0], red_hole_pose_local=[-0.0113,0.0584,0,0,0,0], black_hole_pose_local=[0.0136,0.0583,0,0,0,0], rotary_door_grasping_point_pose_local=[0.0067,0.1468,0,0,0,0], rotary_door_upright_hover_pose=[-0.05326,0.1468,0.07017,0,0,0], probe_hole_local=[-0.04123,0.1468,0,0,0,0], probe_grasping_point_pose_local=[0,0.2047,0,0,0,0]),
										transitions={'done': 'finished', 'failed': 'magicstate'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'box_base_pose': 'T_RobB_BoxB', 'is_debug': 'is_debug', 'is_sim': 'is_sim', 'red_button_pose': 'red_button_pose', 'blue_button_pose': 'blue_button_pose', 'slider_pose': 'slider_pose', 'red_hole_pose': 'red_hole_pose', 'black_hole_pose': 'black_hole_pose', 'rotary_door_grasping_point_pose': 'rotary_door_grasping_point_pose', 'rotary_door_upright_hover_pose': 'rotary_door_upright_hover_pose', 'probe_hole_pose': 'probe_hole_pose', 'probe_grasping_point_pose': 'probe_grasping_point_pose'})

			# x:982 y:308
			OperatableStateMachine.add('2_move slider',
										_sm_2_move_slider_3,
										transitions={'finished': '3_plug probe to test port', 'failed': '2_move slider'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'blue_button_pose': 'blue_button_pose', 'is_sim': 'is_sim', 'is_debug': 'is_debug'})

			# x:1243 y:295
			OperatableStateMachine.add('3_plug probe to test port',
										_sm_3_plug_probe_to_test_port_2,
										transitions={'finished': '4_open door and probe circuit', 'failed': '3_plug probe to test port'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'blue_button_pose': 'blue_button_pose', 'is_sim': 'is_sim', 'is_debug': 'is_debug'})

			# x:1289 y:469
			OperatableStateMachine.add('4_open door and probe circuit',
										_sm_4_open_door_and_probe_circuit_1,
										transitions={'finished': 'finished', 'failed': '4_open door and probe circuit'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'is_sim': 'is_sim', 'is_debug': 'is_debug', 'rotary_door_upright_hover_pose': 'rotary_door_upright_hover_pose', 'rotary_door_grasping_point_pose': 'rotary_door_grasping_point_pose'})

			# x:972 y:535
			OperatableStateMachine.add('5_wrap cable and replace probe_2',
										_sm_5_wrap_cable_and_replace_probe_2_0,
										transitions={'finished': '5_wrap cable and replace probe_2', 'failed': '5_wrap cable and replace probe_2'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'blue_button_pose': 'blue_button_pose', 'is_sim': 'is_sim', 'is_debug': 'is_debug'})

			# x:807 y:54
			OperatableStateMachine.add('Calibrate Board Location',
										CalculateTaskPoseState(red_button_pose_local=[0,0,0,0,0,0], blue_button_pose_local=[0.0136,0,0,0,0,0], slider_pose_local=[-0.0827,0.0348,0,0,0,0], red_hole_pose_local=[-0.0113,0.0584,0,0,0,0], black_hole_pose_local=[0.0136,0.0583,0,0,0,0], rotary_door_grasping_point_pose_local=[0.0067,0.1468,0,0,0,0], rotary_door_upright_hover_pose=[0.0067,0.1468,0,0,0,0], probe_hole_local=[0.0067,0.1468,0,0,0,0], probe_grasping_point_pose_local=[0,0.2047,0,0,0,0]),
										transitions={'done': 'finished', 'failed': 'Calibrate Board Location'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'is_debug': 'is_debug', 'is_sim': 'is_sim', 'red_button_pose': 'red_button_pose', 'blue_button_pose': 'blue_button_pose', 'slider_pose': 'slider_pose', 'red_hole_pose': 'red_hole_pose', 'black_hole_pose': 'black_hole_pose', 'rotary_door_grasping_point_pose': 'rotary_door_grasping_point_pose', 'rotary_door_upright_hover_pose': 'rotary_door_upright_hover_pose', 'probe_hole_pose': 'probe_hole_pose', 'probe_grasping_point_pose': 'probe_grasping_point_pose', 'box_base_pose': 'box_base_pose'})

			# x:318 y:22
			OperatableStateMachine.add('Move Ready Pose',
										ArmJointControlState(q1=-81, q2=-25, q3=28, q4=139, q5=-5, q6=79, q7=94, max_cartesian_vel=0.04, blocking=True, clear=False),
										transitions={'done': 'Calibrate Board Location', 'failed': 'Move Ready Pose'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'is_debug': 'is_debug', 'is_sim': 'is_sim'})

			# x:69 y:172
			OperatableStateMachine.add('Test_slider',
										SliderControlState(z_offset=0.0, blocking=True, clear=False),
										transitions={'done': 'Test_slider', 'failed': 'Test_slider'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'red_button_pose': 'red_button_pose', 'blue_button_pose': 'blue_button_pose', 'slider_pose': 'slider_pose', 'red_hole_pose': 'red_hole_pose', 'black_hole_pose': 'black_hole_pose', 'rotary_door_grasping_point_pose': 'rotary_door_grasping_point_pose', 'probe_grasping_point_pose': 'probe_grasping_point_pose', 'T_RobB_BoxB': 'T_RobB_BoxB'})

			# x:228 y:625
			OperatableStateMachine.add('close_gripper',
										GripperControlState(pos_ratio=0),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'is_sim': 'is_sim', 'is_debug': 'is_debug'})

			# x:842 y:769
			OperatableStateMachine.add('go_above_blue_button',
										ArmCartesianControlState(offset_x=0.0, offset_y=0.0, offset_z=0.05, offset_Rx=0.0, offset_Ry=0.0, offset_Rz=0.0, offset_reference="local", maxvel=0.07, blocking=True, clear=False),
										transitions={'done': 'press_blue_button', 'failed': 'go_above_blue_button'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_T': 'blue_button_pose', 'is_debug': 'is_debug', 'is_sim': 'is_sim'})

			# x:483 y:755
			OperatableStateMachine.add('go_above_red_button',
										ArmCartesianControlState(offset_x=0.0, offset_y=0.0, offset_z=0.05, offset_Rx=0.0, offset_Ry=0.0, offset_Rz=0.0, offset_reference="local", maxvel=0.07, blocking=True, clear=False),
										transitions={'done': 'press_red_button', 'failed': 'press_blue_button'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_T': 'red_button_pose', 'is_debug': 'is_debug', 'is_sim': 'is_sim'})

			# x:667 y:843
			OperatableStateMachine.add('press_blue_button',
										ArmCartesianControlState(offset_x=0.0, offset_y=0.0, offset_z=0.01, offset_Rx=0.0, offset_Ry=0.0, offset_Rz=0.0, offset_reference="local", maxvel=0.07, blocking=True, clear=False),
										transitions={'done': 'go_above_red_button', 'failed': 'press_blue_button'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_T': 'blue_button_pose', 'is_debug': 'is_debug', 'is_sim': 'is_sim'})

			# x:259 y:829
			OperatableStateMachine.add('press_red_button',
										ArmCartesianControlState(offset_x=0.0, offset_y=0.0, offset_z=0.01, offset_Rx=0.0, offset_Ry=0.0, offset_Rz=0.0, offset_reference="local", maxvel=0.07, blocking=True, clear=False),
										transitions={'done': 'close_gripper', 'failed': 'press_red_button'},
										autonomy={'done': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'target_T': 'red_button_pose', 'is_debug': 'is_debug', 'is_sim': 'is_sim'})

			# x:928 y:207
			OperatableStateMachine.add('1_press start button',
										_sm_1_press_start_button_4,
										transitions={'finished': 'finished', 'failed': '1_press start button'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'blue_button_pose': 'blue_button_pose', 'is_sim': 'is_sim', 'is_debug': 'is_debug'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
