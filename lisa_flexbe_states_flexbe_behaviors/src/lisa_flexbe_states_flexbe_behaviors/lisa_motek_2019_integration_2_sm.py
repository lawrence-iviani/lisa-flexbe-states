#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from fzi_flexbe_states.update_pathloader_settings_service import UpdatePathloaderSettingsService
from fzi_flexbe_states.execute_trajectory_action_state import ExecuteTrajectoryActionState
from fzi_flexbe_states.load_trajectory_state import LoadTrajectoryState
from sim2log_flexbe_states.wait_for_trigger_state import WaitForTriggerState
from fzi_flexbe_states.switch_controllers import SwitchControllers
from flexbe_states.wait_state import WaitState
from flexbe_states.publisher_bool_state import PublisherBoolState
from sim2log_flexbe_states.concatenate_string_state import ConcatenateStringState
from lisa_flexbe_states_flexbe_states.lisa_wait_for_intent_state import LisaWaitForState
from lisa_flexbe_states_flexbe_states.lisa_utter_action_state import LisaUtterActionState
from lisa_flexbe_states_flexbe_states.lisa_utter_action_state_input_key import LisaUtterActionStateInputKey
from fzi_flexbe_states.concatenate_strings_state import ConcatenateStringsState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jul 20 2020
@author: Lawrence Iviani
'''
class LisaMotek2019Integration2SM(Behavior):
	'''
	Lisa test playground on top of 
Behavior for the Motek demo 2019 Simulation
Added bidirectional speech
	'''


	def __init__(self):
		super(LisaMotek2019Integration2SM, self).__init__()
		self.name = 'Lisa Motek 2019 Integration 2'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
        
        # [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1102 y:490, x:722 y:440
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.bool_false = False
		_state_machine.userdata.bool_true = True
		_state_machine.userdata.subtrajectory_start_index = -1
		_state_machine.userdata.subtrajectory_end_index = -1
		_state_machine.userdata.mode = 0
		_state_machine.userdata.home_trajectory = 'home.traj'
		_state_machine.userdata.best_effort = 1
		_state_machine.userdata.strict = 2
		_state_machine.userdata.joint_controller = ['joint_trajectory_controller']
		_state_machine.userdata.cartesian_controller = ['cartesian_motion_controller']
		_state_machine.userdata.placing_trajectory = 'placing_'
		_state_machine.userdata.velocity = 10
		_state_machine.userdata.acceleration = 10
		_state_machine.userdata.max_effort = 100
		_state_machine.userdata.open_width = 50
		_state_machine.userdata.close_width = 10
		_state_machine.userdata.trajectory_suffix = '.traj'
		_state_machine.userdata.utter_placement_termnated = 'Trajectory is terminated'
		_state_machine.userdata.utter_delimiter = " "
		_state_machine.userdata.utter_error_traj = "An error happend during trajectory execution,  result is "
		_state_machine.userdata.utter_grasping_part = "Grasping Part "

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


		with _state_machine:
			# x:244 y:183
			OperatableStateMachine.add('Set Patholoader Settings',
										UpdatePathloaderSettingsService(pathloader_namespace='/pathloader'),
										transitions={'succeeded': 'Publish Gripper Open', 'failed': 'failed', 'error': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'velocity': 'velocity', 'acceleration': 'acceleration'})

			# x:561 y:23
			OperatableStateMachine.add('Execute Home Trajectory',
										ExecuteTrajectoryActionState(action_namespace='/pathloader'),
										transitions={'succeeded': 'utter_home_done', 'preempted': 'concatenate_error_to_utter', 'aborted': 'concatenate_error_to_utter'},
										autonomy={'succeeded': Autonomy.Off, 'preempted': Autonomy.Off, 'aborted': Autonomy.Off},
										remapping={'mode': 'mode', 'repeat': 'bool_false', 'subtrajectory_start_index': 'subtrajectory_start_index', 'subtrajectory_end_index': 'subtrajectory_end_index', 'result_message': 'result_message'})

			# x:279 y:598
			OperatableStateMachine.add('Load Placing Trajectory',
										LoadTrajectoryState(load_trajectory_service='/pathloader/loadTrajectory'),
										transitions={'succeeded': 'Execute Placing Trajectory', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'trajectory_name': 'trajectory_name', 'invert_trajectory': 'bool_false', 'raw_path': 'raw_path'})

			# x:27 y:604
			OperatableStateMachine.add('Execute Placing Trajectory',
										ExecuteTrajectoryActionState(action_namespace='/pathloader'),
										transitions={'succeeded': 'concatenate utter result ', 'preempted': 'concatenate_load_failed2', 'aborted': 'concatenate_load_failed2'},
										autonomy={'succeeded': Autonomy.Off, 'preempted': Autonomy.Off, 'aborted': Autonomy.Off},
										remapping={'mode': 'mode', 'repeat': 'bool_false', 'subtrajectory_start_index': 'subtrajectory_start_index', 'subtrajectory_end_index': 'subtrajectory_end_index', 'result_message': 'result_message'})

			# x:1303 y:310
			OperatableStateMachine.add('Wait For Trigger',
										WaitForTriggerState(trigger_service='/enable_placing'),
										transitions={'continue': 'Wait Before Grasping', 'preempt': 'Wait For Trigger'},
										autonomy={'continue': Autonomy.Off, 'preempt': Autonomy.Off})

			# x:233 y:39
			OperatableStateMachine.add('Load Home Trajectory',
										LoadTrajectoryState(load_trajectory_service='/pathloader/loadTrajectory'),
										transitions={'succeeded': 'Execute Home Trajectory', 'failed': 'utter_load_failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'trajectory_name': 'home_trajectory', 'invert_trajectory': 'bool_false', 'raw_path': 'raw_path'})

			# x:1286 y:14
			OperatableStateMachine.add('Switch to Cartesian Controller',
										SwitchControllers(controller_manager_namespace='/controller_manager'),
										transitions={'succeeded': 'Wait For Intent Graspping Part', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'stop_list': 'joint_controller', 'start_list': 'cartesian_controller', 'strictness': 'strict'})

			# x:1126 y:578
			OperatableStateMachine.add('Switch to Joint Controller',
										SwitchControllers(controller_manager_namespace='/controller_manager'),
										transitions={'succeeded': 'Get Placing Trajectory Name', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'stop_list': 'cartesian_controller', 'start_list': 'joint_controller', 'strictness': 'strict'})

			# x:1133 y:358
			OperatableStateMachine.add('Wait Before Grasping',
										WaitState(wait_time=3.0),
										transitions={'done': 'Publish Gripper Closed'},
										autonomy={'done': Autonomy.Off})

			# x:1322 y:576
			OperatableStateMachine.add('Wait After Grasping',
										WaitState(wait_time=1.0),
										transitions={'done': 'Switch to Joint Controller'},
										autonomy={'done': Autonomy.Off})

			# x:1290 y:441
			OperatableStateMachine.add('Publish Gripper Closed',
										PublisherBoolState(topic='/gripper_state'),
										transitions={'done': 'Wait After Grasping'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'bool_true'})

			# x:64 y:43
			OperatableStateMachine.add('Publish Gripper Open',
										PublisherBoolState(topic='/gripper_state'),
										transitions={'done': 'Load Home Trajectory'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'bool_false'})

			# x:889 y:581
			OperatableStateMachine.add('Get Placing Trajectory Name',
										ConcatenateStringState(),
										transitions={'succeeded': 'utter_trajectory'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'prefix_string': 'placing_trajectory', 'input_string': 'part_type', 'suffix_string': 'trajectory_suffix', 'output_string': 'trajectory_name', 'utter_string': 'utter_string'})

			# x:1281 y:118
			OperatableStateMachine.add('Wait For Intent Graspping Part',
										LisaWaitForState(intent_name='Grasp'),
										transitions={'continue': 'conc_part_selected', 'preempt': 'failed'},
										autonomy={'continue': Autonomy.Off, 'preempt': Autonomy.Off},
										remapping={'part_type': 'part_type'})

			# x:942 y:15
			OperatableStateMachine.add('utter_home_done',
										LisaUtterActionState(sentence="Ready for grasping"),
										transitions={'uttered_all': 'Switch to Cartesian Controller', 'timeout': 'failed', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'result_message': 'result_message'})

			# x:597 y:595
			OperatableStateMachine.add('utter_trajectory',
										LisaUtterActionStateInputKey(),
										transitions={'uttered_all': 'Load Placing Trajectory', 'timeout': 'failed', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'sentence': 'utter_string', 'result_message': 'result_message'})

			# x:7 y:328
			OperatableStateMachine.add('concatenate utter result ',
										ConcatenateStringsState(),
										transitions={'succeeded': 'utter_result_placing', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'string1': 'utter_placement_termnated', 'string2': 'result_message', 'delimiter': 'utter_delimiter', 'result_string': 'result_string_utter'})

			# x:532 y:127
			OperatableStateMachine.add('concatenate_error_to_utter',
										ConcatenateStringsState(),
										transitions={'succeeded': 'utter_error_home_traj', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'string1': 'utter_error_traj', 'string2': 'result_message', 'delimiter': 'utter_delimiter', 'result_string': 'err_result_string'})

			# x:845 y:119
			OperatableStateMachine.add('utter_error_home_traj',
										LisaUtterActionStateInputKey(),
										transitions={'uttered_all': 'failed', 'timeout': 'failed', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'sentence': 'err_result_string', 'result_message': 'result_message'})

			# x:325 y:115
			OperatableStateMachine.add('utter_load_failed',
										LisaUtterActionState(sentence="Error loading home trajectory"),
										transitions={'uttered_all': 'failed', 'timeout': 'failed', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'result_message': 'result_message'})

			# x:131 y:401
			OperatableStateMachine.add('concatenate_load_failed2',
										ConcatenateStringsState(),
										transitions={'succeeded': 'utter_error_placing_traj', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'string1': 'utter_error_traj', 'string2': 'result_message', 'delimiter': 'utter_delimiter', 'result_string': 'err_result_string'})

			# x:258 y:490
			OperatableStateMachine.add('utter_error_placing_traj',
										LisaUtterActionStateInputKey(),
										transitions={'uttered_all': 'failed', 'timeout': 'failed', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'sentence': 'err_result_string', 'result_message': 'result_message'})

			# x:17 y:228
			OperatableStateMachine.add('utter_result_placing',
										LisaUtterActionStateInputKey(),
										transitions={'uttered_all': 'Publish Gripper Open', 'timeout': 'failed', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'sentence': 'result_string_utter', 'result_message': 'result_message'})

			# x:1088 y:268
			OperatableStateMachine.add('utter_selected_part',
										LisaUtterActionStateInputKey(),
										transitions={'uttered_all': 'Wait Before Grasping', 'timeout': 'failed', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'sentence': 'result_string', 'result_message': 'result_message'})

			# x:1267 y:195
			OperatableStateMachine.add('conc_part_selected',
										ConcatenateStringsState(),
										transitions={'succeeded': 'utter_selected_part', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'string1': 'utter_grasping_part', 'string2': 'part_type', 'delimiter': 'utter_delimiter', 'result_string': 'result_string'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
