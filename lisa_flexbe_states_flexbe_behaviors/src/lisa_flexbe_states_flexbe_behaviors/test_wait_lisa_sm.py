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
from lisa_flexbe_states_flexbe_states.lisa_utter_and_wait_for_intent_action_state import LisaWaitForState
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
		# x:761 y:216, x:362 y:358
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:58 y:85
			OperatableStateMachine.add('utter_something',
										LisaUtterActionState(sentence='Tell me which part to grasp'),
										transitions={'uttered_all': 'wait_intent', 'timeout': 'failed', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'result_message': 'result_message'})

			# x:569 y:115
			OperatableStateMachine.add('log_outcome',
										LogUserdataState(severity=Logger.REPORT_HINT),
										transitions={'done': 'finished'},
										autonomy={'done': Autonomy.Off},
										remapping={'userdata_key': 'payload'})

			# x:308 y:87
			OperatableStateMachine.add('wait_intent',
										LisaWaitForState(intent_name='Grasp'),
										transitions={'continue': 'log_outcome', 'preempt': 'failed'},
										autonomy={'continue': Autonomy.Off, 'preempt': Autonomy.Off},
										remapping={'payload': 'payload', 'original_sentence': 'original_sentence'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
