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
from lisa_flexbe_states_flexbe_states.lisa_utter_actionlib_state import LisaUtterActionState
from lisa_flexbe_states_flexbe_states.lisa_intent_result_to_string import LisaRecognitionResultToStringState
from lisa_flexbe_states_flexbe_states.lisa_utter_state import LisaUtterState
from fzi_flexbe_states.log_userdata_state import LogUserdataState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Mon Nov 09 2020
@author: lawrence iviani
'''
class test_singleSM(Behavior):
	'''
	a test for block integrity
	'''


	def __init__(self):
		super(test_singleSM, self).__init__()
		self.name = 'test_single'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		# x:848 y:355, x:433 y:438
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.utter_intent = "Utter a valid intent"
		_state_machine.userdata.utter_success = "Result ok"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]

		# x:30 y:341, x:415 y:369
		_sm_container_0 = OperatableStateMachine(outcomes=['finished', 'failed'], input_keys=['payload', 'original_sentence', 'error_reason', 'intent_recognized'])

		with _sm_container_0:
			# x:87 y:61
			OperatableStateMachine.add('result_recogn',
										LisaRecognitionResultToStringState(context_id=None, wait_time=0),
										transitions={'done': 'log_text', 'error': 'failed'},
										autonomy={'done': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'payload': 'payload', 'original_sentence': 'original_sentence', 'error_reason': 'error_reason', 'intent_recognized': 'intent_recognized', 'text_to_utter': 'text_to_utter'})

			# x:83 y:176
			OperatableStateMachine.add('utter_result',
										LisaUtterState(context_id="Container", wait_time=10),
										transitions={'done': 'finished', 'preempt': 'finished', 'timeouted': 'failed', 'error': 'failed'},
										autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'text_to_utter', 'error_reason': 'error_reason'})

			# x:451 y:69
			OperatableStateMachine.add('log_text',
										LogUserdataState(severity=Logger.REPORT_HINT),
										transitions={'done': 'utter_result'},
										autonomy={'done': Autonomy.Off},
										remapping={'userdata_key': 'text_to_utter'})



		with _state_machine:
			# x:76 y:58
			OperatableStateMachine.add('UtterAndWaitForIntent',
										LisaUtterAndWaitForIntentState(context_id=None, intents=[], wait_time=10),
										transitions={'intent_recognized': 'Container', 'intent_not_recognized': 'UtterActionLib', 'preempt': 'failed', 'timeouted': 'failed', 'error': 'failed'},
										autonomy={'intent_recognized': Autonomy.Off, 'intent_not_recognized': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'utter_intent', 'payload': 'payload', 'original_sentence': 'original_sentence', 'error_reason': 'error_reason', 'intent_recognized': 'intent_recognized'})

			# x:59 y:255
			OperatableStateMachine.add('UtterActionLib',
										LisaUtterActionState(text_to_utter='Intent Not Recognized', wait_time=0),
										transitions={'uttered_all': 'finished', 'timeout': 'failed', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'error_reason': 'error_reason'})

			# x:426 y:47
			OperatableStateMachine.add('Container',
										_sm_container_0,
										transitions={'finished': 'finished', 'failed': 'failed'},
										autonomy={'finished': Autonomy.Inherit, 'failed': Autonomy.Inherit},
										remapping={'payload': 'payload', 'original_sentence': 'original_sentence', 'error_reason': 'error_reason', 'intent_recognized': 'intent_recognized'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
