#!/usr/bin/env python
# -*- coding: utf-8 -*-
###########################################################
#               WARNING: Generated code!                  #
#              **************************                 #
# Manual changes may get lost if file is generated again. #
# Only code inside the [MANUAL] tags will be kept.        #
###########################################################

from flexbe_core import Behavior, Autonomy, OperatableStateMachine, ConcurrencyContainer, PriorityContainer, Logger
from lisa_flexbe_states_flexbe_states.lisa_utter_state import LisaUtterState
from lisa_flexbe_states_flexbe_states.lisa_utter_actionlib_state import LisaUtterActionState
from lisa_flexbe_states_flexbe_states.lisa_utter_and_wait_for_intent_state import LisaUtterAndWaitForIntentState
from flexbe_states.check_condition_state import CheckConditionState
from lisa_flexbe_states_flexbe_states.lisa_extract_payload_key import LisaGetPayloadKeyState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 25 2020
@author: lawrence iviani
'''
class test_multipleSM(Behavior):
	'''
	a test of interactions with several repeated blocks
	'''


	def __init__(self):
		super(test_multipleSM, self).__init__()
		self.name = 'test_multiple'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		wait_time_utter = 5
		context_id = "test_multiple"
		intent_1 = ["GetTime"]
		intent_2 = ["YesNo"]
		suspend_time = 1.5
		wait_time_interaction = 10
		# x:633 y:607, x:643 y:65
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.utter_1 = "Utterance example 1"
		_state_machine.userdata.utter_2 = "Utterance example 2, a little bit longer"
		_state_machine.userdata.utter_repeat = "Repeat the test"
		_state_machine.userdata.utter_and_intent_1 = "Intent is Get Time"
		_state_machine.userdata.utter_and_intent_2 = "Intent is Continue Yes or no"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:62 y:59
			OperatableStateMachine.add('Utter_1',
										LisaUtterState(context_id=context_id, wait_time=wait_time_utter, suspend_time=suspend_time),
										transitions={'done': 'UtterAndWaitForIntent_1', 'preempt': 'finished', 'timeouted': 'UtterAndWaitForIntent_1', 'error': 'failed'},
										autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'utter_1', 'error_reason': 'error_reason'})

			# x:1173 y:38
			OperatableStateMachine.add('UtterActionLib',
										LisaUtterActionState(text_to_utter='Intent Not Recognized', wait_time=0),
										transitions={'uttered_all': 'finished', 'timeout': 'failed', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'error_reason': 'error_reason'})

			# x:596 y:287
			OperatableStateMachine.add('Utter_2',
										LisaUtterState(context_id=context_id, wait_time=wait_time_utter, suspend_time=suspend_time),
										transitions={'done': 'UtterAndWaitForIntent_2', 'preempt': 'finished', 'timeouted': 'UtterAndWaitForIntent_2', 'error': 'failed'},
										autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'utter_2', 'error_reason': 'error_reason'})

			# x:1045 y:374
			OperatableStateMachine.add('UtterAndWaitForIntent_2',
										LisaUtterAndWaitForIntentState(context_id=context_id, intents=intent_2, wait_time=wait_time_interaction),
										transitions={'intent_recognized': 'get_answer', 'intent_not_recognized': 'utter_not_recogn_2', 'preempt': 'finished', 'timeouted': 'utter_not_recogn_2', 'error': 'failed'},
										autonomy={'intent_recognized': Autonomy.Off, 'intent_not_recognized': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'utter_and_intent_2', 'payload': 'payload', 'original_sentence': 'original_sentence', 'error_reason': 'error_reason', 'intent_recognized': 'intent_recognized'})

			# x:17 y:609
			OperatableStateMachine.add('UtterNoTimeout',
										LisaUtterState(context_id=context_id, wait_time=0, suspend_time=0),
										transitions={'done': 'Utter_1', 'preempt': 'finished', 'timeouted': 'Utter_1', 'error': 'failed'},
										autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'utter_repeat', 'error_reason': 'error_reason'})

			# x:1373 y:612
			OperatableStateMachine.add('check_finish',
										CheckConditionState(predicate=lambda x: x=="Yes"),
										transitions={'true': 'UtterActionLib', 'false': 'UtterNoTimeout'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'answer'})

			# x:171 y:264
			OperatableStateMachine.add('utter_not_recogn_1',
										LisaUtterActionState(text_to_utter="Intent 1 not recognized try again", wait_time=wait_time_utter),
										transitions={'uttered_all': 'UtterAndWaitForIntent_1', 'timeout': 'UtterAndWaitForIntent_1', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'error_reason': 'error_reason'})

			# x:937 y:513
			OperatableStateMachine.add('utter_not_recogn_2',
										LisaUtterActionState(text_to_utter="Intent 2 not recognized try again", wait_time=wait_time_utter),
										transitions={'uttered_all': 'UtterAndWaitForIntent_2', 'timeout': 'UtterAndWaitForIntent_2', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'error_reason': 'error_reason'})

			# x:289 y:121
			OperatableStateMachine.add('UtterAndWaitForIntent_1',
										LisaUtterAndWaitForIntentState(context_id=context_id, intents=intent_1, wait_time=wait_time_interaction),
										transitions={'intent_recognized': 'Utter_2', 'intent_not_recognized': 'utter_not_recogn_1', 'preempt': 'finished', 'timeouted': 'utter_not_recogn_1', 'error': 'failed'},
										autonomy={'intent_recognized': Autonomy.Off, 'intent_not_recognized': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'utter_and_intent_1', 'payload': 'payload', 'original_sentence': 'original_sentence', 'error_reason': 'error_reason', 'intent_recognized': 'intent_recognized'})

			# x:1342 y:454
			OperatableStateMachine.add('get_answer',
										LisaGetPayloadKeyState(payload_key='confirm'),
										transitions={'done': 'check_finish', 'error': 'failed'},
										autonomy={'done': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'payload': 'payload', 'payload_value': 'answer'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
