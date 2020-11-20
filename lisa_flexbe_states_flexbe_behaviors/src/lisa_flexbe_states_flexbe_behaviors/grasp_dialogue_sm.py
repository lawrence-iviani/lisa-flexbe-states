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
from lisa_flexbe_states_flexbe_states.lisa_extract_payload_key import LisaGetPayloadKeyState
from lisa_flexbe_states_flexbe_states.lisa_intent_result_to_string import LisaRecognitionResultToStringState
from flexbe_states.calculation_state import CalculationState
from flexbe_states.check_condition_state import CheckConditionState
from flexbe_states.log_key_state import LogKeyState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Tue Nov 11 2020
@author: Lawrence Iviani
'''
class GraspDialogueSM(Behavior):
	'''
	Grasp Part Dialogue
	'''


	def __init__(self):
		super(GraspDialogueSM, self).__init__()
		self.name = 'Grasp Dialogue'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
        
        # [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		session_id = 'AskForPartSession'
		wait_for_part = 25
		wait_for_utter = 15
		intents = ['GraspPart']
		part_key = 'part_type'
		detail_levels = 'low'
		# x:73 y:608, x:539 y:365, x:575 y:287
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed', 'max_retry'], output_keys=['part_type'])
		_state_machine.userdata.grasp_part_question = 'Which part to grasp?'
		_state_machine.userdata.part_type = ''
		_state_machine.userdata.retry = 2

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
        
        # [/MANUAL_CREATE]

		# x:30 y:373, x:385 y:382
		_sm_continueretry_0 = OperatableStateMachine(outcomes=['true', 'false'], input_keys=['retry'], output_keys=['retry'])

		with _sm_continueretry_0:
			# x:158 y:82
			OperatableStateMachine.add('decrease_retry',
										CalculationState(calculation=lambda x: x-1),
										transitions={'done': 'continue_asking'},
										autonomy={'done': Autonomy.Off},
										remapping={'input_value': 'retry', 'output_value': 'retry'})

			# x:156 y:221
			OperatableStateMachine.add('continue_asking',
										CheckConditionState(predicate=lambda x: x > 0),
										transitions={'true': 'true', 'false': 'false'},
										autonomy={'true': Autonomy.Off, 'false': Autonomy.Off},
										remapping={'input_value': 'retry'})



		with _state_machine:
			# x:416 y:26
			OperatableStateMachine.add('AskForPart',
										LisaUtterAndWaitForIntentState(context_id=session_id, intents=intents, wait_time=wait_for_part),
										transitions={'intent_recognized': 'Recognized', 'intent_not_recognized': 'NotRecognized', 'preempt': 'failed', 'timeouted': 'failed', 'error': 'failed'},
										autonomy={'intent_recognized': Autonomy.Off, 'intent_not_recognized': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'grasp_part_question', 'payload': 'payload', 'original_sentence': 'original_sentence', 'error_reason': 'error_reason', 'intent_recognized': 'intent_recognized'})

			# x:819 y:368
			OperatableStateMachine.add('UtterRecognizedText',
										LisaUtterState(context_id=session_id, wait_time=wait_for_utter),
										transitions={'done': 'ContinueRetry', 'preempt': 'failed', 'timeouted': 'failed', 'error': 'failed'},
										autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'text_to_utter', 'error_reason': 'error_reason'})

			# x:52 y:390
			OperatableStateMachine.add('GetPartKey',
										LisaGetPayloadKeyState(payload_key=part_key),
										transitions={'done': 'finished', 'error': 'failed'},
										autonomy={'done': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'payload': 'payload', 'payload_value': 'part_type'})

			# x:783 y:27
			OperatableStateMachine.add('NotRecognized',
										LisaRecognitionResultToStringState(context_id=session_id, detail_levels=detail_levels),
										transitions={'done': 'UtterRecognizedText'},
										autonomy={'done': Autonomy.Off},
										remapping={'payload': 'payload', 'original_sentence': 'original_sentence', 'error_reason': 'error_reason', 'intent_recognized': 'intent_recognized', 'text_to_utter': 'text_to_utter'})

			# x:42 y:122
			OperatableStateMachine.add('Recognized',
										LisaRecognitionResultToStringState(context_id=session_id, detail_levels=detail_levels),
										transitions={'done': 'UtterRecognized'},
										autonomy={'done': Autonomy.Off},
										remapping={'payload': 'payload', 'original_sentence': 'original_sentence', 'error_reason': 'error_reason', 'intent_recognized': 'intent_recognized', 'text_to_utter': 'text_to_utter'})

			# x:87 y:244
			OperatableStateMachine.add('UtterRecognized',
										LisaUtterState(context_id=session_id, wait_time=wait_for_utter),
										transitions={'done': 'GetPartKey', 'preempt': 'failed', 'timeouted': 'failed', 'error': 'failed'},
										autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'text_to_utter', 'error_reason': 'error_reason'})

			# x:676 y:195
			OperatableStateMachine.add('ContinueRetry',
										_sm_continueretry_0,
										transitions={'true': 'log_retry_value', 'false': 'max_retry'},
										autonomy={'true': Autonomy.Inherit, 'false': Autonomy.Inherit},
										remapping={'retry': 'retry'})

			# x:635 y:94
			OperatableStateMachine.add('log_retry_value',
										LogKeyState(text="retry level is {}", severity=Logger.REPORT_HINT),
										transitions={'done': 'AskForPart'},
										autonomy={'done': Autonomy.Off},
										remapping={'data': 'retry'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
    
    # [/MANUAL_FUNC]
