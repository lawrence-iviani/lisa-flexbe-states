#!/usr/bin/env python
import rospy
from flexbe_core import Logger, EventState
# from threading import Lock



class LisaRecognitionResultToStringState(EventState):
    '''
    Utter a result from a previous intent recognition block. It is used for debug purposes	.
    
    -- context_id, 	string 	A idenitifier of a larger dialogue session.
    -- context_id, 	string  'low'|'mid'|'high' corresponds to the level of details in the string
    ># text_to_utter 		string 	A text to utter before waiting for intent (default empty text).
    #> error_reason 		string 	An eventual error.
    #> original_sentence 	string 	
    #> payload 			string 	
    #> intent_recognized	string 	

    <= done 			Uttering has finished without any error.


    '''

    def __init__(self, context_id=None, detail_levels='mid'): #, payload_keys):
	# Declare outcomes, input_keys, and output_keys by calling the super constructor with the corresponding arguments.

	super(LisaRecognitionResultToStringState, self).__init__(outcomes = ['done'], 
				     			input_keys=['payload', 'original_sentence', 'error_reason', 'intent_recognized'],
							output_keys=['text_to_utter']) 
	self._context = context_id
	self._text_to_utter = ''
	self._detail_levels = detail_levels

    def execute(self, userdata):
	userdata.text_to_utter = self._text_to_utter
	return 'done'

    def on_enter(self, userdata):
	# Possible outcome
	def _has_user_data_field(field):
		if not hasattr(userdata, field): return False
		if getattr(userdata, field) is None or getattr(userdata, field) == 'None': return False
		if len(getattr(userdata, field)) == 0: return False
		return True

	Logger.logdebug('payload={} - original_sentence={} -  error_reason={} - intent_recognized={}'.format(
			userdata.payload, userdata.original_sentence, userdata.error_reason, userdata.intent_recognized	))	
	if _has_user_data_field('error_reason'):
		if self._detail_levels=='low':
			self._text_to_utter = ''
		elif self._detail_levels=='mid':
			self._text_to_utter = 'Got an error: ' + str(userdata.error_reason)
		else:
			self._text_to_utter = 'During context: '+ self._context +', Got an error: ' + str(userdata.error_reason)
	elif _has_user_data_field('intent_recognized'):	
		if self._detail_levels=='low':
			self._text_to_utter = 'Intent recognized: ' + str(userdata.intent_recognized) + ', payload is ' + str(userdata.payload)
		elif self._detail_levels=='mid':
			self._text_to_utter = 'Intent recognized: ' + str(userdata.intent_recognized) + ', payload is ' + str(userdata.payload)
		else:
			self._text_to_utter = 'During context: '+ self._context + ', Intent recognized: ' + str(userdata.intent_recognized) +\
					      ', payload is ' + str(userdata.payload) +', original sentence' + str(userdata.original_sentence)
	else:
		if self._detail_levels=='low':
			self._text_to_utter = 'Intent not recognized'
		elif self._detail_levels=='mid':
			self._text_to_utter = 'Intent not recognized, original sentence' + str(userdata.original_sentence)
		else:
			self._text_to_utter = 'During context: '+ self._context +', Intent not recognized, original sentence' + str(userdata.original_sentence)

	Logger.loginfo('LisaRecognitionResultToStringState: userdata: |{}|\ntext_to_utter |{}|'.format(userdata, self._text_to_utter))

		
	

