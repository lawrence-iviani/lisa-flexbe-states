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
from fzi_flexbe_states.log_userdata_state import LogUserdataState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Jul 14 2020
@author: Lawrence Iviani
'''
class LisaMotek2019Integration1SM(Behavior):
	'''
	Lisa test playground on top of 
Behavior for the Motek demo 2019 Simulation
	'''


	def __init__(self):
		super(LisaMotek2019Integration1SM, self).__init__()
		self.name = 'Lisa Motek 2019 Integration 1'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
        
        # [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:73 y:608, x:539 y:365
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
		_state_machine.userdata.velocity = 0.1
		_state_machine.userdata.acceleration = 0.2
		_state_machine.userdata.max_effort = 100
		_state_machine.userdata.open_width = 50
		_state_machine.userdata.close_width = 10
		_state_machine.userdata.trajectory_suffix = '.traj'

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]


		with _state_machine:
			# x:74 y:137
			OperatableStateMachine.add('Set Patholoader Settings',
										UpdatePathloaderSettingsService(pathloader_namespace='/pathloader'),
										transitions={'succeeded': 'Publish Gripper Open', 'failed': 'failed', 'error': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'velocity': 'velocity', 'acceleration': 'acceleration'})

			# x:497 y:31
			OperatableStateMachine.add('Execute Home Trajectory',
										ExecuteTrajectoryActionState(action_namespace='/pathloader'),
										transitions={'succeeded': 'log resulting home', 'preempted': 'failed', 'aborted': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'preempted': Autonomy.Off, 'aborted': Autonomy.Off},
										remapping={'mode': 'mode', 'repeat': 'bool_false', 'subtrajectory_start_index': 'subtrajectory_start_index', 'subtrajectory_end_index': 'subtrajectory_end_index', 'result_message': 'result_message'})

			# x:272 y:525
			OperatableStateMachine.add('Load Placing Trajectory',
										LoadTrajectoryState(load_trajectory_service='/pathloader/loadTrajectory'),
										transitions={'succeeded': 'Execute Placing Trajectory', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'trajectory_name': 'trajectory_name', 'invert_trajectory': 'bool_false', 'raw_path': 'raw_path'})

			# x:45 y:485
			OperatableStateMachine.add('Execute Placing Trajectory',
										ExecuteTrajectoryActionState(action_namespace='/pathloader'),
										transitions={'succeeded': 'log result placing', 'preempted': 'failed', 'aborted': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'preempted': Autonomy.Off, 'aborted': Autonomy.Off},
										remapping={'mode': 'mode', 'repeat': 'bool_false', 'subtrajectory_start_index': 'subtrajectory_start_index', 'subtrajectory_end_index': 'subtrajectory_end_index', 'result_message': 'result_message'})

			# x:1281 y:255
			OperatableStateMachine.add('Wait For Trigger',
										WaitForTriggerState(trigger_service='/enable_placing'),
										transitions={'continue': 'Wait Before Grasping', 'preempt': 'Wait For Trigger'},
										autonomy={'continue': Autonomy.Off, 'preempt': Autonomy.Off})

			# x:309 y:39
			OperatableStateMachine.add('Load Home Trajectory',
										LoadTrajectoryState(load_trajectory_service='/pathloader/loadTrajectory'),
										transitions={'succeeded': 'Execute Home Trajectory', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'trajectory_name': 'home_trajectory', 'invert_trajectory': 'bool_false', 'raw_path': 'raw_path'})

			# x:837 y:17
			OperatableStateMachine.add('Switch to Cartesian Controller',
										SwitchControllers(controller_manager_namespace='/controller_manager'),
										transitions={'succeeded': 'Wait For Intent Graspping Part', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'stop_list': 'joint_controller', 'start_list': 'cartesian_controller', 'strictness': 'strict'})

			# x:873 y:576
			OperatableStateMachine.add('Switch to Joint Controller',
										SwitchControllers(controller_manager_namespace='/controller_manager'),
										transitions={'succeeded': 'Get Placing Trajectory Name', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'stop_list': 'cartesian_controller', 'start_list': 'joint_controller', 'strictness': 'strict'})

			# x:1059 y:255
			OperatableStateMachine.add('Wait Before Grasping',
										WaitState(wait_time=3.0),
										transitions={'done': 'Publish Gripper Closed'},
										autonomy={'done': Autonomy.Off})

			# x:1073 y:559
			OperatableStateMachine.add('Wait After Grasping',
										WaitState(wait_time=1.0),
										transitions={'done': 'Switch to Joint Controller'},
										autonomy={'done': Autonomy.Off})

			# x:1065 y:389
			OperatableStateMachine.add('Publish Gripper Closed',
										PublisherBoolState(topic='/gripper_state'),
										transitions={'done': 'Wait After Grasping'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'bool_true'})

			# x:85 y:43
			OperatableStateMachine.add('Publish Gripper Open',
										PublisherBoolState(topic='/gripper_state'),
										transitions={'done': 'Load Home Trajectory'},
										autonomy={'done': Autonomy.Off},
										remapping={'value': 'bool_false'})

			# x:641 y:583
			OperatableStateMachine.add('Get Placing Trajectory Name',
										ConcatenateStringState(),
										transitions={'succeeded': 'log trajectory_name'},
										autonomy={'succeeded': Autonomy.Off},
										remapping={'prefix_string': 'placing_trajectory', 'input_string': 'part_type', 'suffix_string': 'trajectory_suffix', 'output_string': 'trajectory_name'})

			# x:1087 y:48
			OperatableStateMachine.add('Wait For Intent Graspping Part',
										LisaWaitForState(intent_name='/intent'),
										transitions={'continue': 'log_part_type', 'preempt': 'failed'},
										autonomy={'continue': Autonomy.Off, 'preempt': Autonomy.Off},
										remapping={'part_type': 'part_type'})

			# x:1106 y:136
			OperatableStateMachine.add('log_part_type',
										LogUserdataState(severity=Logger.REPORT_HINT),
										transitions={'done': 'Wait Before Grasping'},
										autonomy={'done': Autonomy.Off},
										remapping={'userdata_key': 'part_type'})

			# x:528 y:459
			OperatableStateMachine.add('log trajectory_name',
										LogUserdataState(severity=Logger.REPORT_HINT),
										transitions={'done': 'Load Placing Trajectory'},
										autonomy={'done': Autonomy.Off},
										remapping={'userdata_key': 'trajectory_name'})

			# x:93 y:337
			OperatableStateMachine.add('log result placing',
										LogUserdataState(severity=Logger.REPORT_HINT),
										transitions={'done': 'Publish Gripper Open'},
										autonomy={'done': Autonomy.Off},
										remapping={'userdata_key': 'result_message'})

			# x:675 y:16
			OperatableStateMachine.add('log resulting home',
										LogUserdataState(severity=Logger.REPORT_HINT),
										transitions={'done': 'Switch to Cartesian Controller'},
										autonomy={'done': Autonomy.Off},
										remapping={'userdata_key': 'result_message'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
