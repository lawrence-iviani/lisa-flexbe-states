#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from lisa_flexbe_states_flexbe_states.lisa_utter_action_state import LisaUtterActionState
from fzi_flexbe_states.log_userdata_state import LogUserdataState
from lisa_flexbe_states_flexbe_states.lisa_wait_for_intent_state import LisaWaitForState
from lisa_flexbe_states_flexbe_states.lisa_example_action_state import LisaExampleActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Jul 15 2020
@author: lawrence ibiani
'''
class TestWaitLisaSM(Behavior):
	'''
	A testing for wait lisa a vocal message
	'''


	def __init__(self):
		super(TestWaitLisaSM, self).__init__()
		self.name = 'Test Wait Lisa'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:1098 y:202, x:130 y:321
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.dishwasher = 1

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:47 y:42
			OperatableStateMachine.add('utter',
										LisaUtterActionState(sentence='something to say'),
										transitions={'uttered_all': 'lisa action state dishes', 'timeout': 'failed', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'result_message': 'result_message'})

			# x:927 y:43
			OperatableStateMachine.add('log_userdata',
										LogUserdataState(severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'userdata_key': 'part_type'})

			# x:711 y:43
			OperatableStateMachine.add('wait 1',
										LisaWaitForState(intent_name='/lisa/intent'),
										transitions={'continue': 'log_userdata', 'preempt': 'failed'},
										autonomy={'continue': Autonomy.Off, 'preempt': Autonomy.Off},
										remapping={'part_type': 'part_type'})

			# x:354 y:41
			OperatableStateMachine.add('lisa action state dishes',
										LisaExampleActionState(dishes_to_do=41),
										transitions={'cleaned_some': 'wait 1', 'cleaned_enough': 'wait 1', 'command_error': 'failed'},
										autonomy={'cleaned_some': Autonomy.Off, 'cleaned_enough': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'dishwasher': 'dishwasher', 'cleaned': 'cleaned'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
