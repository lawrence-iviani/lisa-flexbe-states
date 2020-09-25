#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from lisa_flexbe_states_flexbe_states.lisa_utter_and_wait_for_intent_action_state import LisaUtterAndWaitForIntentState
from lisa_flexbe_states_flexbe_states.lisa_utter_action_state import LisaUtterActionState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Wed Sep 23 2020
@author: Lawrence Iviani
'''
class lisa_example_basic_statesSM(Behavior):
	'''
	a basic example how to use specific states
	'''


	def __init__(self):
		super(lisa_example_basic_statesSM, self).__init__()
		self.name = 'lisa_example_basic_states'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		intent_name = 'Grasp'
		text_utter_action = 'A sentence to be pronuced outside of any dialogue via an actionlib'
		# x:721 y:170, x:333 y:241
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:136 y:58
			OperatableStateMachine.add('UtterAndWait',
										LisaUtterAndWaitForIntentState(intent_name=intent_name, text_to_utter='', wait_time=0),
										transitions={'intent_recognized': 'UtterAction', 'intent_not_recognized': 'UtterAndWait', 'preempt': 'failed', 'timeouted': 'utter_timeout'},
										autonomy={'intent_recognized': Autonomy.Off, 'intent_not_recognized': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off},
										remapping={'payload': 'payload', 'original_sentence': 'original_sentence'})

			# x:564 y:60
			OperatableStateMachine.add('UtterAction',
										LisaUtterActionState(sentence=text_utter_action),
										transitions={'uttered_all': 'finished', 'timeout': 'failed', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'result_message': 'result_message'})

			# x:142 y:321
			OperatableStateMachine.add('utter_timeout',
										LisaUtterActionState(sentence='Timeout in uttering'),
										transitions={'uttered_all': 'UtterAndWait', 'timeout': 'failed', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'result_message': 'result_message'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
