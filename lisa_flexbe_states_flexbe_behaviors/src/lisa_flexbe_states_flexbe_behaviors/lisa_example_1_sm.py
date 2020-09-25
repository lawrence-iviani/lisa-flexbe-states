#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from lisa_flexbe_states_flexbe_states.lisa_utter_and_wait_for_intent_state import LisaUtterAndWaitForIntentState
from lisa_flexbe_states_flexbe_states.lisa_utter_state import LisaUtterState
from lisa_flexbe_states_flexbe_states.lisa_utter_actionlib_state import LisaUtterActionState
from fzi_flexbe_states.log_userdata_state import LogUserdataState
from fzi_flexbe_states.concatenate_strings_state import ConcatenateStringsState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Sep 24 2020
@author: Lawrence Iviani
'''
class Lisa_Example_1SM(Behavior):
	'''
	A basic showcase of interaction states for Lisa
	'''


	def __init__(self):
		super(Lisa_Example_1SM, self).__init__()
		self.name = 'Lisa_Example_1'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:30 y:341, x:413 y:470
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.intent_recognized_string = "Recognized intent, payload is "
		_state_machine.userdata.delimiter_composition_strings = ", "
		_state_machine.userdata.intent_recognized_string = ""

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:212 y:20
			OperatableStateMachine.add('utter_and_wait',
										LisaUtterAndWaitForIntentState(intent_name='Grasp', text_to_utter='Tell me which part to grasp', context_id=None, intents=[], wait_time=15),
										transitions={'intent_recognized': 'log_intent', 'intent_not_recognized': 'Utter_not_recogn', 'preempt': 'finished', 'timeouted': 'utter_timeout', 'error': 'log_error'},
										autonomy={'intent_recognized': Autonomy.Off, 'intent_not_recognized': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'payload': 'payload', 'original_sentence': 'original_sentence', 'error_reason': 'error_reason'})

			# x:956 y:62
			OperatableStateMachine.add('utter',
										LisaUtterState(text_to_utter='haloooo', context_id=None, wait_time=2),
										transitions={'done': 'finished', 'preempt': 'failed', 'timeouted': 'utter_timeout', 'error': 'log_error'},
										autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'error_reason': 'error_reason'})

			# x:893 y:382
			OperatableStateMachine.add('UtterErrorExit',
										LisaUtterActionState(sentence='Error on exit'),
										transitions={'uttered_all': 'finished', 'timeout': 'utter_timeout', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'result_message': 'result_message'})

			# x:124 y:404
			OperatableStateMachine.add('utter_timeout',
										LisaUtterState(text_to_utter='A time out has been happend', context_id=None, wait_time=3),
										transitions={'done': 'finished', 'preempt': 'failed', 'timeouted': 'failed', 'error': 'log_error'},
										autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'error_reason': 'error_reason'})

			# x:549 y:13
			OperatableStateMachine.add('log_intent',
										LogUserdataState(severity=Logger.REPORT_HINT),
										transitions={'done': 'Compose String'},
										autonomy={'done': Autonomy.Off},
										remapping={'userdata_key': 'payload'})

			# x:929 y:216
			OperatableStateMachine.add('log_error',
										LogUserdataState(severity=Logger.REPORT_HINT),
										transitions={'done': 'UtterErrorExit'},
										autonomy={'done': Autonomy.Off},
										remapping={'userdata_key': 'error_reason'})

			# x:245 y:93
			OperatableStateMachine.add('Utter_not_recogn',
										LisaUtterState(text_to_utter='Intent not recognized', context_id=None, wait_time=5),
										transitions={'done': 'finished', 'preempt': 'failed', 'timeouted': 'utter_timeout', 'error': 'log_error'},
										autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'error_reason': 'error_reason'})

			# x:746 y:13
			OperatableStateMachine.add('Compose String',
										ConcatenateStringsState(),
										transitions={'succeeded': 'utter', 'failed': 'failed'},
										autonomy={'succeeded': Autonomy.Off, 'failed': Autonomy.Off},
										remapping={'string1': 'intent_recognized_string', 'string2': 'payload', 'delimiter': 'delimiter_composition_strings', 'result_string': 'intent_recognized_string'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
