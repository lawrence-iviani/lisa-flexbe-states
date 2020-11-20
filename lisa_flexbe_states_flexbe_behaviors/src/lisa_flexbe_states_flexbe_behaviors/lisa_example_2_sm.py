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
from flexbe_states.wait_state import WaitState
from lisa_flexbe_states_flexbe_states.lisa_utter_intent_result import LisaUtterIntentResultState
# Additional imports can be added inside the following tags
# [MANUAL_IMPORT]

# [/MANUAL_IMPORT]


'''
Created on Thu Sep 24 2020
@author: Lawrence Iviani
'''
class Lisa_Example_2SM(Behavior):
	'''
	A basic showcase of interaction states for Lisa.
Use of context is explained
	'''


	def __init__(self):
		super(Lisa_Example_2SM, self).__init__()
		self.name = 'Lisa_Example_2'

		# parameters of this behavior

		# references to used behaviors

		# Additional initialization code can be added inside the following tags
		# [MANUAL_INIT]
		
		# [/MANUAL_INIT]

		# Behavior comments:



	def create(self):
		context_id_1 = "ContextGrasp"
		context_id_2 = "ContextAny"
		intent_2 = []
		intent_1 = ["Grasp"]
		context_id_err = "ContextError"
		# x:30 y:341, x:413 y:470
		_state_machine = OperatableStateMachine(outcomes=['finished', 'failed'])
		_state_machine.userdata.intent_recognized_string = "Recognized intent, payload is "
		_state_machine.userdata.delimiter_composition_strings = ", "
		_state_machine.userdata.intent_recognized_string = ""
		_state_machine.userdata.grasp_utter = "Tell me intent, grasp part A or B or C"
		_state_machine.userdata.timeout_utter = "A timeout has happend"
		_state_machine.userdata.anyintent_utter = "Utter one of the available intents"
		_state_machine.userdata.success_utter = "All the operations were completed succesfully"
		_state_machine.userdata.fail_utter = "An error happened during execution"

		# Additional creation code can be added inside the following tags
		# [MANUAL_CREATE]
		
		# [/MANUAL_CREATE]


		with _state_machine:
			# x:212 y:20
			OperatableStateMachine.add('utter_and_wait',
										LisaUtterAndWaitForIntentState(context_id=context_id_1, intents=intent_1, wait_time=15),
										transitions={'intent_recognized': 'log_intent', 'intent_not_recognized': 'Utter_not_recogn', 'preempt': 'finished', 'timeouted': 'utter_timeout', 'error': 'log_error'},
										autonomy={'intent_recognized': Autonomy.Off, 'intent_not_recognized': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'grasp_utter', 'payload': 'payload', 'original_sentence': 'original_sentence', 'error_reason': 'error_reason', 'intent_recognized': 'intent_recognized'})

			# x:1111 y:7
			OperatableStateMachine.add('utter',
										LisaUtterState(context_id=context_id_1, wait_time=10),
										transitions={'done': 'wait_debug', 'preempt': 'failed', 'timeouted': 'utter_timeout', 'error': 'log_error'},
										autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'intent_recognized', 'error_reason': 'error_reason'})

			# x:918 y:421
			OperatableStateMachine.add('UtterErrorExit',
										LisaUtterActionState(text_to_utter='an error has rasied', wait_time=0),
										transitions={'uttered_all': 'finished', 'timeout': 'utter_timeout', 'command_error': 'failed'},
										autonomy={'uttered_all': Autonomy.Off, 'timeout': Autonomy.Off, 'command_error': Autonomy.Off},
										remapping={'error_reason': 'error_reason'})

			# x:124 y:404
			OperatableStateMachine.add('utter_timeout',
										LisaUtterState(context_id=context_id_err, wait_time=3),
										transitions={'done': 'finished', 'preempt': 'failed', 'timeouted': 'failed', 'error': 'log_error'},
										autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'timeout_utter', 'error_reason': 'error_reason'})

			# x:549 y:13
			OperatableStateMachine.add('log_intent',
										LogUserdataState(severity=Logger.REPORT_HINT),
										transitions={'done': 'speechlog_grasp'},
										autonomy={'done': Autonomy.Off},
										remapping={'userdata_key': 'payload'})

			# x:1126 y:132
			OperatableStateMachine.add('log_error',
										LogUserdataState(severity=Logger.REPORT_HINT),
										transitions={'done': 'UtterErrorExit'},
										autonomy={'done': Autonomy.Off},
										remapping={'userdata_key': 'error_reason'})

			# x:261 y:114
			OperatableStateMachine.add('Utter_not_recogn',
										LisaUtterState(context_id=context_id_1, wait_time=5),
										transitions={'done': 'finished', 'preempt': 'failed', 'timeouted': 'utter_timeout', 'error': 'log_error'},
										autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'fail_utter', 'error_reason': 'error_reason'})

			# x:1315 y:226
			OperatableStateMachine.add('utter_and_wait_2',
										LisaUtterAndWaitForIntentState(context_id=context_id_2, intents=[], wait_time=10),
										transitions={'intent_recognized': 'log_intent_2', 'intent_not_recognized': 'Utter_not_recogn', 'preempt': 'failed', 'timeouted': 'utter_timeout', 'error': 'log_error'},
										autonomy={'intent_recognized': Autonomy.Off, 'intent_not_recognized': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'anyintent_utter', 'payload': 'payload', 'original_sentence': 'original_sentence', 'error_reason': 'error_reason', 'intent_recognized': 'intent_recognized'})

			# x:1354 y:426
			OperatableStateMachine.add('log_intent_2',
										LogUserdataState(severity=Logger.REPORT_HINT),
										transitions={'done': 'utter_2'},
										autonomy={'done': Autonomy.Off},
										remapping={'userdata_key': 'payload'})

			# x:1313 y:593
			OperatableStateMachine.add('utter_2',
										LisaUtterState(context_id=context_id_err, wait_time=0),
										transitions={'done': 'finished', 'preempt': 'failed', 'timeouted': 'utter_timeout', 'error': 'log_error'},
										autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'success_utter', 'error_reason': 'error_reason'})

			# x:1351 y:14
			OperatableStateMachine.add('wait_debug',
										WaitState(wait_time=0.5),
										transitions={'done': 'utter_and_wait_2'},
										autonomy={'done': Autonomy.Off})

			# x:764 y:0
			OperatableStateMachine.add('speechlog_grasp',
										LisaUtterIntentResultState(context_id=None, wait_time=0),
										transitions={'done': 'utter', 'preempt': 'failed', 'timeouted': 'utter_timeout', 'error': 'log_error'},
										autonomy={'done': Autonomy.Off, 'preempt': Autonomy.Off, 'timeouted': Autonomy.Off, 'error': Autonomy.Off},
										remapping={'text_to_utter': 'original_sentence', 'error_reason': 'error_reason'})


		return _state_machine


	# Private functions can be added inside the following tags
	# [MANUAL_FUNC]
	
	# [/MANUAL_FUNC]
